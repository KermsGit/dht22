#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

 
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/delay.h>

/****************************************************************************/
/* Defines                                                                  */
/****************************************************************************/
#define DRIVER_AUTHOR  "Ing. Gerhard Scheutz <gerhard.scheutz@gmx.at>"
#define DRIVER_DESC    "DHT22 Driver"
#define PROC_DIR_NAME  "dht22"
#define PROC_FILE_NAME "gpio%02d"
 
// we want GPIO_17 (pin 11 on P5 pinout raspberry pi rev. 2 board)
// to generate interrupt
#define GPIO_ANY_GPIO                	4
 
// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_ANY_GPIO_DESC           	"gpio pin used for DHT22"
 
// below is optional, used in more complex code, in our case, this could be
// NULL
#define GPIO_ANY_GPIO_DEVICE_DESC    	"Read temperature and huminity"

#define MAX_TIMESTAMPS			43
// T LOW is typical 50 us 
#define T_LOW				50
// Time border between low/high bit 
#define BIT_LEVEL                       (53 + T_LOW)
// Macro to generate bit
#define MAKE_BIT(x)                     ((get_time_diff(x+3)) > BIT_LEVEL ? 1:0)                             

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int irq_any_gpio    = 0;
short int counted_edges   = 0;

ktime_t timestamps[MAX_TIMESTAMPS];
ktime_t read_timestamp; //    = ktime_set(0L, 0L);
u8      bytes[4];
u8      checksum;
bool    chksum_ok         = true;
 
/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {
 
   unsigned long flags;
   
   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);
 
   // NOTE:
   // Anonymous Sep 17, 2013, 3:16:00 PM:
   // You are putting printk while interupt are disabled. printk can block.
   // It's not a good practice.
   // 
   // hardware.coder:
   // http://stackoverflow.com/questions/8738951/printk-inside-an-interrupt-handler-is-it-really-that-bad
 
   if(counted_edges >= 0 && counted_edges < MAX_TIMESTAMPS) {
      timestamps[counted_edges++] = ktime_get();
      }
   // printk(KERN_NOTICE "DHT22 trigger %d times!.\n", count_edge);
 
   // restore hard interrupts
   local_irq_restore(flags);
 
   return IRQ_HANDLED;
   }

/****************************************************************************/
/* Debug and convert funktions                                              */
/****************************************************************************/
void print_timestamps(void) {
   short int i;

   printk("Counted falling edges = %d\n", counted_edges);
   for(i = 0; i < MAX_TIMESTAMPS; i++) {
      printk("Timestamp[%02d] %lld\n", i, ktime_to_ns(timestamps[i]));
   }
}

int get_time_diff(short int i) {
   // get T HIGH time in us
   return(ktime_to_us(timestamps[i]) - ktime_to_us(timestamps[i-1]));
}

void print_timediffs(void) {
   short int i;
   
   for(i = 3; i < MAX_TIMESTAMPS; i++) {
      printk("Timediffstamp[%02d] %d us\n", i, get_time_diff(i));
   }
}

void print_bytes(void) {
   short int i;
   printk("Counted falling edges = %d\n", counted_edges);
   for(i=0; i < 4; i++) {
      printk("bytes[%d] = %02x\n", i, bytes[i]);
   }
   printk("checksum = %02x\n", checksum);
   printk("chksum_ok = %d\n", chksum_ok);
}

void make_bytes(void) {
   u8 chk = 0;

   bytes[0] = (MAKE_BIT(7)       | (MAKE_BIT(6) << 1) | (MAKE_BIT(5) << 2) | (MAKE_BIT(4)  << 3) |
              (MAKE_BIT(3)  << 4)| (MAKE_BIT(2) << 5) | (MAKE_BIT(1) << 6) | (MAKE_BIT(0)  << 7));
   bytes[1] = (MAKE_BIT(15)      | (MAKE_BIT(14) << 1)| (MAKE_BIT(13) << 2)| (MAKE_BIT(12) << 3) |
              (MAKE_BIT(11) << 4)| (MAKE_BIT(10) << 5)| (MAKE_BIT(9) << 6) | (MAKE_BIT(8)  << 7));
   bytes[2] = (MAKE_BIT(23)      | (MAKE_BIT(22) << 1)| (MAKE_BIT(21) << 2)| (MAKE_BIT(20) << 3) |
              (MAKE_BIT(19) << 4)| (MAKE_BIT(18) << 5)| (MAKE_BIT(17) << 6)| (MAKE_BIT(16) << 7));
   bytes[3] = (MAKE_BIT(31)      | (MAKE_BIT(30) << 1)| (MAKE_BIT(29) << 2)| (MAKE_BIT(28) << 3) |
              (MAKE_BIT(27) << 4)| (MAKE_BIT(26) << 5)| (MAKE_BIT(25) << 6)| (MAKE_BIT(24) << 7));
   checksum = (MAKE_BIT(39)      | (MAKE_BIT(38) << 1)| (MAKE_BIT(37) << 2)| (MAKE_BIT(36) << 3) |
              (MAKE_BIT(35) << 4) |(MAKE_BIT(34) << 5)| (MAKE_BIT(33) << 6) |(MAKE_BIT(32) << 7));
   chk = bytes[0] + bytes[1] + bytes[2] +  bytes[3];
   chksum_ok = (checksum == chk);	   
}

void print_temperature(void){
   int h = bytes[2]*256 + bytes[3];

   printk("temperature = %d.%1d° C\n", h/10, h%2);
}

void print_humidity(void){
   int h = bytes[0]*256 + bytes[1];

   printk("humidity = %d.%1d%% RH\n", h/10, h%2);
}

/****************************************************************************/
/* This function starts the HT22 sensor                                     */
/****************************************************************************/
void send_start_bit(void){
   ktime_t new_timestamp;

   printk("Set start bit!\n");

   new_timestamp = ktime_get();
   // read only every 2000 ms
   if((ktime_to_ms(new_timestamp)-ktime_to_ms(read_timestamp)) >= 2000) {
      read_timestamp = new_timestamp;
      counted_edges = 0;
      memset(&timestamps[0], 0, sizeof(timestamps));

      gpio_direction_output(GPIO_ANY_GPIO,0);
      //__raw_writel(1 << GPIO_ANY_GPIO, gpio_cleardataout_addr);
      mdelay(2);
      //__raw_writel(1 << GPIO_ANY_GPIO, gpio_setdataout_addr);
      gpio_direction_output(GPIO_ANY_GPIO,1);
      udelay(20);
      gpio_direction_input(GPIO_ANY_GPIO);
      mdelay(15);
   }
}

/****************************************************************************/
/* This function configures interrupts.                                     */
/****************************************************************************/
void r_int_config(void) {
   
   if (gpio_request(GPIO_ANY_GPIO, GPIO_ANY_GPIO_DESC)) {
      printk("GPIO request faiure: %s\n", GPIO_ANY_GPIO_DESC);
      return;
   }
 
   if ( (irq_any_gpio = gpio_to_irq(GPIO_ANY_GPIO)) < 0 ) {
      printk("GPIO to IRQ mapping faiure %s\n", GPIO_ANY_GPIO_DESC);
      return;
   }
 
   printk(KERN_NOTICE "Mapped int %d\n", irq_any_gpio);
 
   if (request_irq(irq_any_gpio,
                   (irq_handler_t ) r_irq_handler,
                   IRQF_TRIGGER_FALLING,
                   GPIO_ANY_GPIO_DESC,
                   GPIO_ANY_GPIO_DEVICE_DESC)) {
      printk("Irq Request failure\n");
      return;
   }
   else {
      send_start_bit();
      print_timestamps();      
      print_timediffs();
      make_bytes();
      print_bytes();
      print_humidity();
      print_temperature();
   }
}
 
/****************************************************************************/
/* This function releases interrupts.                                       */
/****************************************************************************/
void r_int_release(void) {
 
   free_irq(irq_any_gpio, GPIO_ANY_GPIO_DEVICE_DESC);
   gpio_free(GPIO_ANY_GPIO);
}
 
/****************************************************************************/
/* Module proc filesystem block.                                      */
/****************************************************************************/

static int proc_show(struct seq_file *m, void *v) {
   short int h;
   short int t;   
   struct timeval tv;

   // Read sensor
   send_start_bit();
   make_bytes();
   tv = ktime_to_timeval(read_timestamp);
   
   t = bytes[2]*256 + bytes[3];
   h = bytes[0]*256 + bytes[1];
   
   seq_printf(m, "DHT22 on gpio pin %d:", GPIO_ANY_GPIO);
   seq_printf(m, "\n");
   seq_printf(m, "  temperature = %d.%1d° C\n", t/10, t%2);
   seq_printf(m, "  humidity = %d.%1d%% RH\n", h/10, h%2);
   seq_printf(m, "  timestamp = %ld\n", (long)tv.tv_sec);
   seq_printf(m, chksum_ok ?  "  no checksum error\n" : "  checksum error !\n");
   return 0;
}

static int proc_open(struct inode *inode, struct file *file) {
   
   return single_open(file, proc_show, NULL);
}

static struct proc_dir_entry *proc_dir, *proc_file;
static const struct file_operations proc_fileops = {
   .owner   = THIS_MODULE,
   .open    = proc_open,
   .read    = seq_read,
   .llseek  = seq_lseek,
   .release = single_release,
};

static void proc_init(void) {
   char name[16]; 

   proc_dir  = proc_mkdir(PROC_DIR_NAME, NULL);
   snprintf(name, sizeof name, PROC_FILE_NAME, GPIO_ANY_GPIO);
   proc_file = proc_create(name, S_IRUGO, proc_dir, &proc_fileops);
}

static void proc_cleanup(void) {
   char name[16]; 

   snprintf(name, sizeof name, PROC_FILE_NAME, GPIO_ANY_GPIO);
   if(proc_file) remove_proc_entry(name, proc_dir);   
   if(proc_dir)  remove_proc_entry(PROC_DIR_NAME, NULL);
}

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
int r_init(void) {
 
   printk(KERN_NOTICE "Hello DHT22!\n");
   r_int_config();
   proc_init();
 
   return 0;
}
 
void r_cleanup(void) {
   printk(KERN_NOTICE "Goodbye\n");
   proc_cleanup();
   r_int_release();
} 
 
module_init(r_init);
module_exit(r_cleanup);
 
/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
