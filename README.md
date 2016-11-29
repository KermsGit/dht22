# dht22 Kernel Driver

A DHT22 kernel driver for Raspberry Pi.
See the specufication on http://www.electrodragon.com/w/AM2302

Do a "make" and a "insmod dht22.ko" in the dmesg you should see:

[  273.097098] chksum_ok = 1
[  273.097107] humidity = 42.0% RH
[  273.097116] temperature = 21.0° C

A cat "/proc/dht22/gpio04"

root@pixel:~/Source/kernel/dht22# cat /proc/dht22/gpio04
DHT22 on gpio pin 4:
  temperature = 21.1° C
  humidity = 43.1% RH
  timestamp = 398
  no checksum error
  
At the moment the driver is in a early alpha stage.
the DHT22 is wired fixed(hardcoded in the source) to GPIO Pin 4.
 
