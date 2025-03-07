This code is developped to control a DDS AD910 from analog device with a nucleo-H723 board, and is tested with a module from amazon such as : https://www.amazon.fr/AD9910-Module-d%C3%A9chantillonnage-fr%C3%A9quence-sinueux/dp/B0CB5DWQMK?th=1  

Pinout :  
STM32 H723                       AD9910  
IO_Update : PF10 --------------->IOUP  
DDS_CSn : PB1 ------------------>CSB  
DDS_Reset : PB2 ---------------->RST  
SPI1_SCK :  PA5 ---------------->SCK                     
SPI1_MOSI : PD7 ---------------->SDIO  
SPI1_MISO : PA6 ---------------->SDO  

PWR to GND  
PD open  

En mode fréquence CW, si la PLL est activée, il faut programmer N et VCO. Fsysclk = N x REF_CLK.
Dans ce cas d'usage, REF_CLK = 40 MHz, VCO = VCO1, N=12, Fsysclk = 480MHz.
