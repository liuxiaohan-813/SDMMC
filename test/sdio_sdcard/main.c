#include <stdio.h>
#include <string.h>

extern SD_CardInfo SDCardInfo;
#define BUFSIZE 128

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
    uint32_t j = 0, ret = 0;

    // we take dma mode, but dma cannot access core member, so must init data into a external sram.
    uint32_t sram_base[2] = {0x1781F800, 0x1781FC00};
    uint32_t dst_base_addr;
    uint32_t src_base_addr;
    src_base_addr = (uint32_t)(sram_base[0]);
    dst_base_addr = (uint32_t)(sram_base[1]);

    uint32_t buf_write[BUFSIZE] = {0};
    uint32_t buf_read[BUFSIZE] = {0};

    for (j = 0; j < BUFSIZE; j++)
    {
        buf_write[j] = 0x12345678 + j;
    }

    memset((uint32_t *)dst_base_addr, 0, BUFSIZE * 4);
    memcpy((uint32_t *)src_base_addr, (uint32_t *)buf_write, BUFSIZE * 4);

    printf("sdio test\r\n");
    while (SD_Init())
    {
        printf("Please check your SD card!\r\n");
    }

    ret = SD_WriteDisk((uint8_t *)src_base_addr, 0, 1);
    if (ret == 0)
    {
        printf("Write sector pass\r\n");
    }
    ret = SD_ReadDisk((uint8_t *)dst_base_addr, 0, 1);
    if (ret == 0)
    {
        printf("Read sector pass\r\n");
    }

    memcpy((uint32_t *)buf_read, (uint32_t *)dst_base_addr, BUFSIZE * 4);

    for (int m = 0; m < BUFSIZE; m++)
    {
        if (buf_write[m] != buf_read[m])
        {
            simulation_fail();
            printf("sdio read write fail\r\n");
            printf("buf_read %d :%d\r\n", m, buf_read[m]);
            printf("buf_write %d :%d\r\n", m, buf_write[m]);
            return -1;
        }
    }
    simulation_pass();
    printf("sdio read write successfully\r\n");
    while (1)
        ;

    return 0;
}
