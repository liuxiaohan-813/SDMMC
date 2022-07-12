#include <stdio.h>

#define BLKSIZE 512
extern SD_CardInfo SDCardInfo;

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
    printf("Card Capacity:%d MB\r\n",
           (uint32_t)(SDCardInfo.CardCapacity >> 20));
    printf("Card BlockSize:%d\r\n\r\n", SDCardInfo.CardBlockSize);
}

int main(void)
{
    uint32_t buf_write[BLKSIZE] = {0};
    uint32_t buf_read[BLKSIZE] = {0};
    uint32_t j = 0, ret = 0;
    for (j = 0; j < BLKSIZE; j++)
    {
        buf_write[j] = 0x10 + j;
    }
    printf("sdio test\r\n");
    while (SD_Init())
    {
        printf("Please check your SD card!\r\n");
    }

    show_sdcard_info();

    ret = SD_WriteDisk((uint8_t *)buf_write, 0, 1);
    if (ret == 0)
    {
        printf("Write sector pass\r\n");
    }
    ret = SD_ReadDisk((uint8_t *)buf_read, 0, 1);
    if (ret == 0)
    {
        printf("Read sector pass\r\n");
    }

    for (int m = 0; m < BLKSIZE; m++)
    {
        if (buf_write[m] != buf_read[m])
        {
            printf("sdio read write fail\r\n");
            printf("buf_read [%d] :%d\r\n", m, buf_read[m]);
            printf("buf_write[%d] :%d\r\n", m, buf_write[m]);
            return -1;
        }
    }
    printf("sdio read write successfully\r\n");
    while (1)
        ;

    return 0;
}
