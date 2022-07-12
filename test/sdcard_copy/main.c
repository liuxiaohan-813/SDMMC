#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ff.h"
#include "diskio.h"

extern uint8_t CardType;
extern uint32_t CSD_Tab[4];
extern uint32_t CID_Tab[4];
extern uint32_t RCA;
extern SD_CardInfo SDCardInfo;
#define FATFS_WR_SIZE 1024 * 8

void show_sdcard_info(void)
{
    switch (SDCardInfo.CardType)
    {
    case SDIO_STD_CAPACITY_SD_CARD_V1_1:
        printf("Card Type:SDSC V1.1\r\n");
        break;
    case SDIO_STD_CAPACITY_SD_CARD_V2_0:
        printf("Card Type:SDSC V2.0\r\n");
        break;
    case SDIO_HIGH_CAPACITY_SD_CARD:
        printf("Card Type:SDHC V2.0\r\n");
        break;
    case SDIO_MULTIMEDIA_CARD:
        printf("Card Type:MMC Card\r\n");
        break;
    }
    printf("Card ManufacturerID:%d\r\n", SDCardInfo.SD_cid.ManufacturerID);
    printf("Card RCA:%d\r\n", SDCardInfo.RCA);
    printf("Card Capacity:%d MB\r\n", (uint32_t)(SDCardInfo.CardCapacity >> 20));
    printf("Card BlockSize:%d\r\n\r\n", SDCardInfo.CardBlockSize);
}

__attribute__((aligned(4))) uint8_t buffer_all[FATFS_WR_SIZE];
void copyfile(uint8_t *srcfilename, uint8_t *destfilename)
{

    UINT br, bw;
    uint32_t f_res;
    FIL file_src;
    FIL file_dest;

    f_res = f_open(&file_src, srcfilename, FA_OPEN_EXISTING | FA_READ | FA_OPEN_ALWAYS);
    if (f_res == FR_OK)
    {
        f_res = f_open(&file_dest, destfilename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
        if (f_res != FR_OK)
        {
            return;
        }
    }
    printf("start copy file.... \r\n");
    while (f_res == 0)
    {
        f_res = f_read(&file_src, buffer_all, FATFS_WR_SIZE, &br);
        printf("f_res = %d,br= %d   ", f_res, br);
        if (f_res || br == 0)
            break; /* ÎÄ¼þ½áÊø´íÎó */
        f_res = f_write(&file_dest, buffer_all, br, &bw);
        printf("f_res = %d,br= %d,bw=%d \r\n", f_res, br, bw);
        if (f_res || bw < br)
            break; /* ´ÅÅÌÂú´íÎó */
    }
    f_close(&file_src);
    f_close(&file_dest);
    printf("\r\ncopyfile finish\r\n");
}

int main(void)
{
    int i, j;
    FIL file;
    FIL file1;
    FATFS fatfs;
    static FRESULT res;
    unsigned int counter;

    res = f_mount(&fatfs, "0:", 1);
    if (res != RES_OK)
    {
        printf("f_mount error:%d!\r\n", res);
        while (1)
        {
        }
    }
    else
    {
        printf("f_mount successful!\r\n");
    }

    f_setlabel((const TCHAR *)"0:Nuclei");

    copyfile("0:/fatfs.txt", "0:/cp.txt");
}
