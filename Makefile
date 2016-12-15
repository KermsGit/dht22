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
 
info:
	modinfo  ${module_upload}

