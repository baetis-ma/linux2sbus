# linux2sbus

execute with --> gcc sbus.c -o sbus.exe

program code set up for port /dev/ttyUSB0

program flow
     - set up port with init()
     - write packet
     - receive packet
     - write packet
     - sleep 5ms
     - read packet
     - read packet

last packet read has nothing there so will time out after 500ms.

If rx and tx not shorted on usb serial device first packet will timeout.



