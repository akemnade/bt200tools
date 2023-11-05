# bt200tools
misc helper tools to use a recent linux with the Epson Moverio BT-200

## setup-bootchoice
change bootorder to UART, USB and uSD card and reboot immediately
after emergency remountly readonly

## write-bootmode
write the bootmode used by the factory u-boot to decide what to do

## read-gps
reads gps data from /dev/tigps (factory kernel) or /dev/gnssX (from new kernel)
in the AI2 protocol. Protocol is not fully understood, so just the basics
are there.

Usage:
read-gps device [nmea]

Using the nmea keyword enables output of some simple GPRMC
NMEA records generated from the AI2 data.
