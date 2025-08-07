/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "ns_sdk_hal.h"
#include "diskio.h"        /* FatFs lower layer API */
#include "malloc.h"
#include "sdmmc.h"

#define SDMMC_CARD     0
SDMMC_HandleTypeDef g_sd_handle;                    /* SDCard handle */
SDMMC_CardInfoTypeDef CardInfo;              		/* Card information */

DSTATUS disk_initialize (
    BYTE pdrv                /* Physical drive nmuber (0..) */
)
{
    uint8_t res = 0;
    res = SDMMC_Init(&g_sd_handle);
    if(res) {
        return  STA_NOINIT;
    } else {
        SDMMC_GetCardInfo(g_sd_handle, &CardInfo);
        /* i don't know why emmc cardcapacity > 1G, mount fatfs failed. */
        CardInfo.CardCapacity = 1024 * 1024 * 1024;
        return 0;
    }
}

DSTATUS disk_status (
    BYTE pdrv        /* Physical drive nmuber (0..) */
)
{
    return 0;
}

DRESULT disk_read (
    BYTE pdrv,        /* Physical drive nmuber (0..) */
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,    /* Sector address (LBA) */
    UINT count        /* Number of sectors to read (1..BLKSIZE) */
)
{
    uint8_t res = 0;
    if (!count)
        return RES_PARERR;
    switch (pdrv) {
        case SDMMC_CARD:
            res = SDMMC_ReadDisk(&g_sd_handle, buff, sector, count);
            while(res)
            {
                SDMMC_Init(&g_sd_handle);
                res = SDMMC_ReadDisk(&g_sd_handle, buff, sector, count);
            }
            break;
        default:
            res = 1;
    }
    if (res == 0x00)
        return RES_OK;
    else
        return RES_ERROR;
}

#if _USE_WRITE
DRESULT disk_write (
    BYTE pdrv,            /* Physical drive nmuber (0..) */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector address (LBA) */
    UINT count            /* Number of sectors to write (1..BLKSIZE) */
)
{
    uint8_t res = 0;
    if (!count)
        return RES_PARERR;
    switch (pdrv) {
        case SDMMC_CARD:
            res = SDMMC_WriteDisk(&g_sd_handle, (uint8_t*)buff, sector, count);
            while (res) {
                SDMMC_Init(&g_sd_handle);
                res = SDMMC_WriteDisk(&g_sd_handle, (uint8_t*)buff, sector, count);
            }
            break;
        default:
            res = 1;
    }

    if (res == 0x00)
        return RES_OK;
    else
        return RES_ERROR;
}
#endif

#if _USE_IOCTL
DRESULT disk_ioctl (
    BYTE pdrv,        /* Physical drive nmuber (0..) */
    BYTE cmd,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
    DRESULT res;
    if (pdrv == SDMMC_CARD) {
        switch (cmd) {
            case CTRL_SYNC:
                res = RES_OK;
                break;
            case GET_SECTOR_SIZE:
                *(DWORD *)buff = 512;
                res = RES_OK;
                break;
            case GET_BLOCK_SIZE:
                *(WORD *)buff = CardInfo.CardBlockSize;
                res = RES_OK;
                break;
            case GET_SECTOR_COUNT:
                *(DWORD *)buff = CardInfo.CardCapacity / 512;
                res = RES_OK;
                break;
            default:
                res = RES_PARERR;
                break;
        }
    } else
        res = RES_ERROR;
    return res;
}
#endif

DWORD get_fattime(void) { return 0; }
