/*!
    \file  README.TXT
    \brief description of the sdio ctest
    \version 2021-3-18, v1.0.1
*/

Function description:
    This demo requires an external SD Card .This demo shows how to use the SDIO firmware library and an associate driver 
to perform read/write operations on the SD Card memory  (SD Card V2.0 and SDHC (High Capacity) protocol) 

Test result:
    If the terminal displays the desired print, the execution succeeds.

IO Connection information:
    Connect SDIO0 SDIO0_SDCLK      to SDCard CLK
    Connect SDIO0 SDIO0_SDCMD      to SDCard CMD
    Connect SDIO0 SDIO0_SDDATA0    to SDCard DAT0
    Connect SDIO0 SDIO0_SDDATA1    to SDCard DAT1
    Connect SDIO0 SDIO0_SDDATA2    to SDCard DAT2
    Connect SDIO0 SDIO0_SDDATA3    to SDCard DAT3

Note:
    If we upload the program to operate SDCard which would be destory the FatFs.
