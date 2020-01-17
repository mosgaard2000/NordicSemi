# NordicSemi
Guideslines for implementing a background DFU.

This project is made to show how to implement the Nordic Background DFU in a project reading the image file from a sd-card.
All Nordic examples uses some external interfaces (Bluetooth, USB or UART), but I was in need of a system that already has the image. I use a SPI to read the SD-card, but could just as well be read from Flash or Ram.

In the Nordic_Source folder I have my main function which is stripped for everything except the bootloader code.

