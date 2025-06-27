/*!
    \file  README.TXT
    \brief description of the sdio copy from EMMC
    \version 2021-3-18, v1.0.1
*/

Function description:
    This demo shows copying files on EMMC via SDIO0.
    
    NOTE:
    1.An EMMC in FAT32 format should be prepared first.
    2.There is a file named fatfs.txt on EMMC.

Test result:
    If the terminal displays the desired print and the file is successfully copied as filename cp.txt, 
the execution is considered successful, otherwise it fails.

IO Connection information:
    Connect SDIO0_SDCLK      to EMMC CLK
    Connect SDIO0_SDCMD      to EMMC CMD
    Connect SDIO0_SDDATA0    to EMMC DAT0
    Connect SDIO0_SDDATA1    to EMMC DAT1
    Connect SDIO0_SDDATA2    to EMMC DAT2
    Connect SDIO0_SDDATA3    to EMMC DAT3
    Connect SDIO0_SDDATA4    to EMMC DAT4
    Connect SDIO0_SDDATA5    to EMMC DAT5
    Connect SDIO0_SDDATA6    to EMMC DAT6
    Connect SDIO0_SDDATA7    to EMMC DAT7
