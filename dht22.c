/*
 * DHT22 gpio device driver 
 * 
 * Copyright (c) Ing. Gerhad Scheutz <gerhard.scheutz@gmx.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/ktime.h>

#include <linux/platform_device.h>
#include <linux/iio/iio.h>

/****************************************************************************/
/* Defines                                                                  */
/****************************************************************************/
#define DRIVER_AUTHOR	"Ing. Gerhard Scheutz <gerhard.scheutz@gmx.at>"
#define DRIVER_NAME	"dht22"
#define DRIVER_DESC	"DHT22 Driver"
#define PROC_DIR_NAME	"dht22"
#define PROC_FILE_NAME	"gpio%02d"

// we want GPIO pin 4
// to generate interrupt
#define GPIO_ANY_GPIO                	4
#define GPIO_ANY_GPIO_DESC           	"gpio pin used for DHT22"
#define GPIO_ANY_GPIO_DEVICE_DESC    	"Read temperature and humidity"
#define DHT22_CONST_SCALE				1
#define DHT22_CONST_SCALE2				10

#define MAX_TIMESTAMPS					43
// T LOW is typical 50 us 
#define T_LOW							50
// Time border between low/high bit 
#define BIT_LEVEL                       (53 + T_LOW)
// Macro to generate bit
#define MAKE_BIT(x)                     ((get_time_diff(x+3)) > BIT_LEVEL ? 1:0)

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/

struct dht22_state {
	struct device *dev;
	struct mutex lock;

	short int gpio;
	short int irq;
	short int counted_edges;
	ktime_t timestamps[MAX_TIMESTAMPS];
	ktime_t read_timestamp;
	u8 bytes[4];
	u8 checksum;
	bool chksum_ok;
	int temperature;
	int humidity;
} *dht22;

/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{

	unsigned long flags;

	// disable hard interrupts (remember them in flag 'flags')
	local_irq_save(flags);

	// NOTE:
	//

	// You are putting printk while interupt are disabled. printk can block.
	// It's not a good practice.
	// 
	// hardware.coder:
	// http://stackoverflow.com/questions/8738951/printk-inside-an-interrupt-handler-is-it-really-that-bad

	if (dht22->counted_edges >= 0 && dht22->counted_edges < MAX_TIMESTAMPS) {
		dht22->timestamps[dht22->counted_edges++] = ktime_get();
	}
	// printk(KERN_NOTICE "DHT22 trigger %d times!.\n", count_edge);

	// restore hard interrupts
	local_irq_restore(flags);

	return IRQ_HANDLED;
}

/****************************************************************************/
/* Debug and convert funktions                                              */
/****************************************************************************/
void print_timestamps(void)
{
	short int i;

	printk("Counted falling edges = %d\n", dht22->counted_edges);
	for (i = 0; i < MAX_TIMESTAMPS; i++) {
		printk("Timestamp[%02d] %lld\n", i,
		       ktime_to_ns(dht22->timestamps[i]));
	}
}

int get_time_diff(short int i)
{
	// get T HIGH time in us
	return (ktime_to_us(dht22->timestamps[i]) -
		ktime_to_us(dht22->timestamps[i - 1]));
}

void print_timediffs(void)
{
	short int i;

	for (i = 3; i < MAX_TIMESTAMPS; i++) {
		printk("Timediffstamp[%02d] %d us\n", i, get_time_diff(i));
	}
}

void print_bytes(void)
{
	short int i;
	printk("Counted falling edges = %d\n", dht22->counted_edges);
	for (i = 0; i < 4; i++) {
		printk("bytes[%d] = %02x\n", i, dht22->bytes[i]);
	}
	printk("checksum = %02x\n", dht22->checksum);
	printk("chksum_ok = %d\n", dht22->chksum_ok);
}

void make_bytes(void)
{
	u8 chk = 0;

	dht22->bytes[0] =
	    (MAKE_BIT(7)       | (MAKE_BIT(6) << 1) | (MAKE_BIT(5) << 2) | (MAKE_BIT(4) << 3) |
        (MAKE_BIT(3) << 4) | (MAKE_BIT(2) << 5) | (MAKE_BIT(1) << 6) | (MAKE_BIT(0) << 7));
	dht22->bytes[1] =
	    (MAKE_BIT(15)       | (MAKE_BIT(14) << 1) | (MAKE_BIT(13) << 2) | (MAKE_BIT(12) << 3) |
        (MAKE_BIT(11) << 4) | (MAKE_BIT(10) << 5) | (MAKE_BIT(9)  << 6) | (MAKE_BIT(8)  << 7));
	dht22->bytes[2] =
	    (MAKE_BIT(23)       | (MAKE_BIT(22) << 1) | (MAKE_BIT(21) << 2) | (MAKE_BIT(20) << 3) |
        (MAKE_BIT(19) << 4) | (MAKE_BIT(18) << 5) | (MAKE_BIT(17) << 6) | (MAKE_BIT(16) << 7));
	dht22->bytes[3] =
	    (MAKE_BIT(31)       | (MAKE_BIT(30) << 1) | (MAKE_BIT(29) << 2) | (MAKE_BIT(28) << 3) |
        (MAKE_BIT(27) << 4) | (MAKE_BIT(26) << 5) | (MAKE_BIT(25) << 6) | (MAKE_BIT(24) << 7));
	dht22->checksum =
	    (MAKE_BIT(39)       | (MAKE_BIT(38) << 1) | (MAKE_BIT(37) << 2) | (MAKE_BIT(36) << 3) |
        (MAKE_BIT(35) << 4) | (MAKE_BIT(34) << 5) | (MAKE_BIT(33) << 6) | (MAKE_BIT(32) << 7));
	
	chk = dht22->bytes[0] + dht22->bytes[1] + dht22->bytes[2] + dht22->bytes[3];
	dht22->chksum_ok = (dht22->checksum == chk);
}

bool check_measurement(void)
{
	// got all edges
	if (dht22->counted_edges != MAX_TIMESTAMPS) {
		return (false);
	}
	make_bytes();
	return dht22->chksum_ok;
}

int get_temperature(void)
{
	int t = (dht22->bytes[2] & 0x7F) * 256 + dht22->bytes[3];
	if (dht22->bytes[2] & 0x80) {
		t = -t;
	}
	return t;
}

void print_temperature(void)
{
	int t = get_temperature();
	printk("temperature = %d.%1d° C\n", t / 10, t % 10);
}

int get_humidity(void)
{
	int h = dht22->bytes[0] * 256 + dht22->bytes[1];
	return h;
}

void print_humidity(void)
{
	int h = get_humidity();

	printk("humidity = %d.%1d%% RH\n", h / 10, h % 10);
}

/****************************************************************************/
/* This function starts the HT22 sensor                                     */
/****************************************************************************/
void send_start_bit(void)
{
	ktime_t new_timestamp;

	new_timestamp = ktime_get();
	// read only every 2000 ms
	if ((ktime_to_ms(new_timestamp) - ktime_to_ms(dht22->read_timestamp)) >=
	    2000) {
		printk("Set start bit!\n");

		dht22->read_timestamp = new_timestamp;
		dht22->counted_edges = 0;
		memset(&(dht22->timestamps[0]), 0, sizeof(dht22->timestamps));

		// set GPIO 
		gpio_direction_output(GPIO_ANY_GPIO, 0);
		mdelay(2);
		gpio_direction_output(GPIO_ANY_GPIO, 1);
		udelay(20);
		gpio_direction_input(GPIO_ANY_GPIO);
		mdelay(15);
	}
}

/****************************************************************************/
/* This function configures interrupts.                                     */
/****************************************************************************/
int setup_dht22(struct dht22_state *dht22_idev)
{
	dht22 = dht22_idev;

	if(dht22->gpio == 0) {
		dht22->gpio = GPIO_ANY_GPIO;
		if (gpio_request(GPIO_ANY_GPIO, GPIO_ANY_GPIO_DESC)) {
			printk(KERN_NOTICE "GPIO request faiure: %s\n", GPIO_ANY_GPIO_DESC);
			return -1;
		}
	}

	if(dht22->irq == 0) {
		if ((dht22->irq = gpio_to_irq(dht22->gpio)) < 0) {
			printk(KERN_NOTICE "GPIO to IRQ mapping faiure %s\n", GPIO_ANY_GPIO_DESC);
		return -1;
		}
	}

	printk(KERN_NOTICE "Mapped int %d\n", dht22->irq);

	if (request_irq(dht22->irq,
			(irq_handler_t) r_irq_handler,
			IRQF_TRIGGER_FALLING,
			GPIO_ANY_GPIO_DESC, GPIO_ANY_GPIO_DEVICE_DESC)) {
		printk(KERN_NOTICE "Irq Request failure\n");
		return -1;
	}

	mutex_init(&dht22->lock);

	mutex_lock(&dht22->lock);
	send_start_bit();
	// Try again if it fails
	if (!check_measurement()) {
		send_start_bit();
		check_measurement();
	}
	mutex_unlock(&dht22->lock);
	
	print_timestamps();
	print_timediffs();
	check_measurement();
	print_timestamps();
	print_timediffs();
	print_bytes();
	print_humidity();
	print_temperature();
	return 0;
}

/****************************************************************************/
/* This function releases module resources.                                       */
/****************************************************************************/
void m_int_release(void)
{
	free_irq(dht22->irq, GPIO_ANY_GPIO_DEVICE_DESC);
	gpio_free(dht22->gpio);
	kfree(dht22);
}

/****************************************************************************/
/* Module proc filesystem block.                                      */
/****************************************************************************/

static int proc_show(struct seq_file *m, void *v)
{
	short int h;
	short int t;
	ktime_t tv;

	// Read sensor
	send_start_bit();
	// Try again if it fails
	if (!check_measurement()) {
		send_start_bit();
		check_measurement();
	}

	tv = ktime_get();
	t = get_temperature();
	h = get_humidity();

	seq_printf(m, "DHT22 on gpio pin %d:", GPIO_ANY_GPIO);
	seq_printf(m, "\n");
	seq_printf(m, "  temperature = %d.%1d° C\n", t / 10, t % 10);
	seq_printf(m, "  humidity = %d.%1d%% RH\n", h / 10, h % 10);
	seq_printf(m, "timestamp = %lli", tv);
	seq_printf(m, dht22-> chksum_ok ? "  no checksum error\n" : "  checksum error !\n");
	return 0;
}

static int proc_open(struct inode *inode, struct file *file)
{

	return single_open(file, proc_show, NULL);
}

static struct proc_dir_entry *proc_dir, *proc_file;
static const struct proc_ops proc_fileops = {
	.proc_open = proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void proc_init(void)
{
	char name[16];

	proc_dir = proc_mkdir(PROC_DIR_NAME, NULL);
	snprintf(name, sizeof name, PROC_FILE_NAME, GPIO_ANY_GPIO);
	proc_file = proc_create(name, S_IRUGO, proc_dir, &proc_fileops);
}

static void proc_cleanup(void)
{
	char name[16];

	snprintf(name, sizeof name, PROC_FILE_NAME, GPIO_ANY_GPIO);
	if (proc_file)
		remove_proc_entry(name, proc_dir);
	if (proc_dir)
		remove_proc_entry(PROC_DIR_NAME, NULL);
}

static void read_sensor(struct dht22_state *dht22)
{
	mutex_lock(&dht22->lock);
    // Read sensor
	send_start_bit();
	// Try again if it fails
	if (!check_measurement()) {
		send_start_bit();
		check_measurement();
		}
	mutex_unlock(&dht22->lock);
}

static int read_raw(struct iio_dev *iio_dev,
		    const struct iio_chan_spec *chan,
		    int *val, int *val2, long mask)
{
	struct dht22_state *dht22 = iio_priv(iio_dev);

	switch(mask) {
 		case IIO_CHAN_INFO_RAW:
    		// Read sensor
			read_sensor(dht22);
			if (chan->type == IIO_TEMP){
				*val = get_temperature();
				}
			else if (chan->type == IIO_HUMIDITYRELATIVE){
				*val = get_humidity();
				}
			else
				return (-EINVAL);
			return (IIO_VAL_INT);
			break;

 		case IIO_CHAN_INFO_PROCESSED:
    		// Read sensor
			read_sensor(dht22);
			*val2 = DHT22_CONST_SCALE2;
			if (chan->type == IIO_TEMP){
				*val = get_temperature() * DHT22_CONST_SCALE;
				}
			else if (chan->type == IIO_HUMIDITYRELATIVE){
				*val = get_humidity() * DHT22_CONST_SCALE;
				}
			else
				return (-EINVAL);
			return (IIO_VAL_FRACTIONAL);
			break;			

		case IIO_CHAN_INFO_SCALE:
            *val  = DHT22_CONST_SCALE;
			*val2 = DHT22_CONST_SCALE2;
			return(IIO_VAL_FRACTIONAL);
			break;

		default:
		    dev_err(dht22->dev, "wrong mask %ld", mask);
			return (-EINVAL);
	}
}

/****************************************************************************/
/* iio_dev variables block                                               */
/****************************************************************************/
static const struct iio_info dht22_iio_info = {
	.read_raw = read_raw,
};

static const struct iio_chan_spec dht22_chan_spec[] = {
	{.type = IIO_TEMP,
	 .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED),
	 .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),},
	{.type = IIO_HUMIDITYRELATIVE,
	 .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED),
	 .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),}
};

static const struct of_device_id dht22_dt_ids[] = {
	{.compatible = DRIVER_NAME,},
	{}
};

/****************************************************************************/
/* Module probe block.                                                      */
/****************************************************************************/
static int probe_dht22(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    struct device_node *node = dev->of_node;
	struct dht22_state *dht22_idev;
	struct iio_dev *iio;
	int ret;

	iio = devm_iio_device_alloc(dev, sizeof(struct dht22_state));
	if (!iio) {
		dev_err(dev, "dht22: Failed to allocate IIO device\n");
		return (-ENOMEM);
	}
	dht22_idev = iio_priv(iio);
	dht22_idev->dev = dev;
	//get gpio_pin
	ret = of_get_gpio(node, 0);   
	if(ret < 0){
		dev_err(dev, "dht22: Failed to get GPIO pin\n");
	    	goto free_iio_device;
	}

	dht22_idev->gpio = ret;
	ret =
	    devm_gpio_request_one(dev, dht22_idev->gpio, GPIOF_IN, pdev->name);
	if (ret < 0) {
		dev_err(dev, "dht22: Failed to get GPIO %d\n",
			dht22_idev->gpio);
		goto free_iio_device;
	}

	dht22_idev->irq = gpio_to_irq(dht22_idev->gpio);
	if (dht22_idev->irq < 0) {
		dev_err(dev, "GPIO %d has no interrupt\n", dht22_idev->gpio);
		ret = -EINVAL;
		goto free_iio_gpio;
	}

	if(setup_dht22(dht22_idev) < 0) {
		dev_err(dev, "Setup DHT22 failed\n");
		ret = -EINVAL;
		goto free_iio_irq;
	}
	proc_init();

	platform_set_drvdata(pdev, iio);
	//mutex_init(&dht22_idev->lock);
	iio->name = pdev->name;
	iio->dev.parent = &pdev->dev;
	iio->info = &dht22_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = dht22_chan_spec;
	iio->num_channels = ARRAY_SIZE(dht22_chan_spec);

	return (devm_iio_device_register(dev, iio));

 free_iio_irq:
    free_irq(dht22_idev->irq, GPIO_ANY_GPIO_DEVICE_DESC);
 free_iio_gpio:
	devm_gpio_free(dev, dht22_idev->gpio);
 free_iio_device:
	devm_iio_device_alloc(dev, iio);
	return (ret);
}

static int remove_dht22(struct platform_device *pdev) {
	proc_cleanup();
    free_irq(dht22->irq, GPIO_ANY_GPIO_DEVICE_DESC);
	return (0);
}

static struct platform_driver dht22_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = dht22_dt_ids,
		   },
	.probe 	= probe_dht22,
	.remove = remove_dht22,
};

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
int m_init(void)
{
	printk(KERN_NOTICE "Hello DHT22!\n");

	dht22 = kmalloc(sizeof(struct dht22_state), GFP_KERNEL);
	if (!dht22) {
		printk("kmalloc request faiure: %s\n", GPIO_ANY_GPIO_DESC);
		return -1;
	}
	setup_dht22(dht22);
	proc_init();

	return 0;
}

void m_cleanup(void)
{
	printk(KERN_NOTICE "Goodbye\n");
	proc_cleanup();
	m_int_release();
}

//module_init(m_init);
//module_exit(m_cleanup);
module_platform_driver(dht22_driver);

/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_DEVICE_TABLE(of, dht22_dt_ids);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
