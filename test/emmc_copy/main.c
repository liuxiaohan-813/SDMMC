#include "ns_sdk_hal.h"

#include "diskio.h"
#include "ff.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdmmc.h"

/*-------------------------- Variable ---------------------------*/

#define FATFS_WR_SIZE 1024*8

extern SDMMC_HandleTypeDef g_sd_handle;                    /* SDCard handle */
char SD_FileName[] = "hello.txt";
uint8_t write_cnt =0;
uint8_t WriteBuffer[] = "01 write buff to sd\r\n";

void sdio_config(void)
{
    SDIO_StructInit(&g_sd_handle.Sdio_init);
    g_sd_handle.Instance = SDIO0;
    g_sd_handle.Sdio_init.ClkDiv = 0x64;
    g_sd_handle.Sdio_init.AutoStop = DISABLE;
    g_sd_handle.Sdio_init.ClkStop = DISABLE;
    g_sd_handle.Sdio_init.HighWidthMode = ENABLE;
    g_sd_handle.Sdio_init.DataCrcChk = ENABLE;

    SDIO_Init(g_sd_handle.Instance, &g_sd_handle.Sdio_init);
    SDIO_DmaInterruptEn(g_sd_handle.Instance, SDIO_RX_FTRANS_IRQ_EN, ENABLE);
    SDIO_DmaInterruptEn(g_sd_handle.Instance, SDIO_TX_FTRANS_IRQ_EN, ENABLE);

    g_sd_handle.Sdio_cfg.BusMode = SDIO_SDR_MODE;
    g_sd_handle.Sdio_cfg.BusWidth = SDIO_8LINE;
    g_sd_handle.Sdio_cfg.DeviceMode = SDIO_DMA_MODE;
    g_sd_handle.Sdio_cfg.CardType = SDIO_MULTIMEDIA_CARD;
}

__attribute__((aligned(4))) uint8_t buffer_all[FATFS_WR_SIZE];
void copyfile(uint8_t *srcfilename, uint8_t *destfilename)
{

    UINT br, bw;
    uint32_t f_res;
    FIL file_src;
    FIL file_dest;

    f_res = f_open(&file_src, srcfilename,
                   FA_OPEN_EXISTING | FA_READ | FA_OPEN_ALWAYS);
    if (f_res == FR_OK) {
        f_res = f_open(&file_dest, destfilename,
                       FA_CREATE_ALWAYS | FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
        if (f_res != FR_OK) {
            return;
        }
    }
    printf("start copy file.... \r\n");
    while (f_res == 0) {
        f_res = f_read(&file_src, buffer_all, FATFS_WR_SIZE, &br);
        printf("f_res = %d,br= %d   ", f_res, br);
        if (f_res || br == 0)
            break;
        f_res = f_write(&file_dest, buffer_all, br, &bw);
        printf("f_res = %d,br= %d,bw=%d \r\n", f_res, br, bw);
        if (f_res || bw < br)
            break;
    }
    f_close(&file_src);
    f_close(&file_dest);
    printf("\r\ncopyfile finish\r\n");
}

int main(void)
{
    #ifdef MISC_HAS_SDIO0_HAS_CLK
    sdio0_clk_en(ENABLE);
    
    #endif
    #ifdef MISC_HAS_SDIO0_RST
    sdio0_set_rst(DISABLE);
    sdio0_set_rst(ENABLE);
    
    #endif
    
    sdio_config();

    FIL file;
    FATFS fatfs;
    static FRESULT res;
    UINT Bw;

    BYTE work[FF_MAX_SS]; /* Working buffer */

    /* filesystem parameter: format = FAT32, other use default val */
    MKFS_PARM fs_parm = {
    /* filesystem parameter: format = FAT32, other use default val */
        .fmt = FS_FAT32, .n_fat = 0, .au_size = 0, .align = 0, .n_root = 0,
    };

    res = f_mount(&fatfs, "0:", 1); 

    if (res == FR_NO_FILESYSTEM) {
        printf("flahs has no file system, start make filesystem...\n");
        res = f_mkfs((const TCHAR *)"0:", &fs_parm, work, sizeof(work));
        if (res != FR_OK) {
            printf("f_mkfs fail : %d!\r\n", res);
            goto fail;
        } else {
            printf("f_mkfs successful!\n");
            res = f_mount(&fatfs, "0:", 1); 
        }
    }
    else if(res == FR_OK){
        printf("mount success! \r\n");
    }
    else{
        printf("mount fail! \r\n");
        goto fail;
    }

    f_setlabel((const TCHAR *)"0:Nuclei"); 

    res = f_open(&file, SD_FileName, FA_OPEN_ALWAYS | FA_WRITE); 
    if((res & FR_DENIED) == FR_DENIED)
    {
        printf("Card storage is full, write failed!\r\n");
    }

    f_lseek(&file, f_size(&file));
    if(res == FR_OK)
    {
        printf("Open successfully/Create file successfully!\r\n");
        res = f_write(&file, WriteBuffer, sizeof(WriteBuffer), &Bw);
        if(res == FR_OK)
        {
            printf("file was written successfully! \r\n");
        }
        else
        {
            printf("file was written fail! \r\n");
            goto fail;
        }
    }
    else
    {
        printf("fail to open the file!\r\n");
        goto fail;
    }

    f_close(&file);

    copyfile("0:/hello.txt", "0:/hello_cp.txt");

success:
    simulation_pass();
    while (1) {}

fail:
    simulation_fail();
    while (1) {}
}
