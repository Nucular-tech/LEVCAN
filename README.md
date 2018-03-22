# LEVCAN
Light Electric Vehicle CAN protocol

Some ideas taken from J1939 and CanOpen

Uses 29bit CAN network and (currently) FreeRTOS functionality

Features
----------------
 - Two transmission modes, TCP (controlled reception and data order) and UDP 
 - Multiple nodes supported for one device
 - Dynamic network address
 - Configurable parameters for devices
 - Up to 125 devices on bus
 - 10 bit message ID + length matching
 - Broadcast and adressed messages

Each message that is sent by a Node contains source address. There are 128 possible addresses: 
 - 0..126 – Valid source addresses for Node 
 - 0..63 – Used for Node with Preferred Addresses
 - 64...125 – Available for all Node
 - 126 – Null (not set)
 - 127 – Global 
 
CAN 1Mbps
7812.5 msg/s at 8byte (total msg 128 bit)
62500 data bytes max
