/*!
    \file  README.TXT
    \brief description of the sdio copy from SDCard
    \version 2021-3-18, v1.0.1
*/

Function description:
    This demo shows copying files on SDCard via SDIO0.
    
    NOTE:
    1.An SDCard in FAT32 format should be prepared first.
    2.There is a file named fatfs.txt on SDCard.

Test result:
    If the terminal displays the desired print and the file is successfully copied as filename cp.txt, 
the execution is considered successful, otherwise it fails.

IO Connection information:
    Connect SDIO0_SDCLK      to SDCard_CLK
    Connect SDIO0_SDCMD      to SDCard_CMD
    Connect SDIO0_SDDATA0    to SDCard_DAT0
    Connect SDIO0_SDDATA1    to SDCard_DAT1
    Connect SDIO0_SDDATA2    to SDCard_DAT2
    Connect SDIO0_SDDATA3    to SDCard_DAT3
