I have taken refernce from embextronics for understanding how to write bootloader for STM32 and where to place the application program. (https://www.youtube.com/watch?v=O3M_4rrw1LI)
Have done my modifications on top of it to write the code in stm32 using cdc rather than uart.

Changes done in host application:
1. I have using RS232_SendBuf method so that we recieve data at one go and not byte by byte ove cdc.
2. I have reduced the maximum size of packet from 1024 bytes to 50 bytes of data as over USB CDC we can recieve data only in chunks of 64 bytes (data more than that is nroken in fragments).
3. After sending the first packet of data i have added delay of 3s as during first packet transfer flash is erased so before sending ack it takes time and if there is no sleep exe assumes we got nack.
4. Used pragma pack(push 1) to pack the struct __attribute__((packed)) did not work for me.
 
 Changes in bootloader program:
1. Before jumping to application make sure to deinit the peripherals used by botloader, stop the clock and also make sure the msp now gets its value from the applications vector table (0x08040000).
2. I have used printf over SWD and not UART.
3. Used CDC to get application from the pc.
4. Reduced reception size to maximum of 50+9 bytes due to USB limitation.(understood that for cdc)
5. Another thing learnt is if i recv bytes from pc it will be problem getting the data and arranging.

Changes in application program:
1. Again here used printf over SWD.
