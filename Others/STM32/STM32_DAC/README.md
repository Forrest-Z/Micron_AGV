# How To Use These Files
Last edited 2 May 2019  

1. Enable USB FS and DAC in CubeMX  
2. Find the appropriate files in Src and Inc and replace with these files  
3. Compile and flash

## Current Problems
Since we are using virtual com ports for communication with the main computer, there is difficulty in differentiating which STM32 is which if multiple STM32s are used. Perhaps need to use UART programming instead of the USB OTG.  
