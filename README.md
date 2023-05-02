# LEVCAN
Light Electric Vehicle CAN protocol

Some ideas taken from J1939 and CanOpen

Uses 29bit CAN network (1MBps default), check dev branches for latest builds.

Features
----------------
 - Two transmission modes, "TCP" (controlled reception and data order) and "UDP"
 - Notifications for user
 - Dynamic network address
 - Configurable parameters for devices
 - Simple file i/o with file server
 - Static and dynamic memory supported
 - Up to 125 devices on bus
 - 10 bit message ID + length matching
 - Broadcast and adressed messages

System requirements
----------------
 - C11
 - Little endian
 - 8bit = int8_t
 - RTOS recommended for faster responce and needed for advanced features
 - Binary size for gcc -O2 on Cortex M4:
 
![alt text](https://i.imgur.com/G70JQeg.png)
 
Low level info
----------------
Each message that is sent by a Node contains source address. There are 128 possible addresses: 
 - 0..126 – Valid source addresses for Node 
 - 0..63 – Used for Node with Preferred Addresses
 - 64...125 – Available for all Node
 - 126 – Null (not set)
 - 127 – Global 
 
CAN 1Mbps
7812.5 msg/s at 8byte (total msg 128 bit)
62500 data bytes max

![alt text](https://i.imgur.com/L0YKIc9.png)
![alt text](https://i.imgur.com/CYgbNCG.png)
