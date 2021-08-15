# my optional module name
MODULE = dht22
 
obj-m := ${MODULE}.o
 
module_upload=${MODULE}.ko
 
all:	clean compile dht22-overlay.dtbo

dht22-overlay.dtbo:
	dtc -I dts -O dtb -o dht22.dtbo dht22-overlay.dts
 
compile:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
 
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean
 
# this just copies a file to raspberry
#install:
#	scp ${module_upload} pi@raspberry:test/

# This will only work if compiling on the pi
install:
	cp ${MODULE}.ko /lib/modules/$(shell uname -r)/kernel/drivers/iio/humidity/
	cp ${MODULE}.dtbo /boot/overlays/
	depmod -a 

uninstall:
	rm -fv /lib/modules/$(shell uname -r)/kernel/drivers/iio/humidity/${MODULE}.ko
	rm -fv /boot/overlays/${MODULE}.dtbo

 
info:
	modinfo  ${module_upload}

