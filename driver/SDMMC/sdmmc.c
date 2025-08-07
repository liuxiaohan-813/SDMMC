
/* Includes ------------------------------------------------------------------*/
#include "ns.h"
#include "ns_sdio.h"
#include "sdmmc.h"

#include <string.h>

SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataSetupTypeDef SDIO_DataSetupStruct;
SDIO_DmaCfgTypeDef SDIO_DmaCfgStruct;

static uint32_t SDMMC_PowerON(SDMMC_HandleTypeDef *hsd);
static uint32_t SDMMC_InitializeCards(SDMMC_HandleTypeDef *hsd);

static uint32_t SDMMC_SendStatus(SDMMC_HandleTypeDef *hsd);
static uint32_t SDMMC_SendSDStatus(SDMMC_HandleTypeDef *hsd, uint32_t *pSDstatus);

static uint32_t FindSCR(SDMMC_HandleTypeDef *hsd, uint16_t rca, uint32_t *pscr);
static uint32_t SD_WideBusConfig(SDMMC_HandleTypeDef *hsd);
static uint32_t MMC_WideBusConfig(SDMMC_HandleTypeDef *hsd);
static uint32_t MMC_ReadExtCsd(SDMMC_HandleTypeDef *hsd);

/**
  * @brief  Returns the current card's status.
  * @param  hsd: Pointer to SD handle
  * @param  pCardStatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register)
  * @retval error state
  */
static uint32_t SDMMC_SendStatus(SDMMC_HandleTypeDef *hsd)
{
    uint32_t errorstate;

    /* Send Status command */
    errorstate = SDIO_CmdSendStatus(hsd->Instance, (uint32_t)(hsd->CardInfo.RelCardAdd << 16U), &(hsd->CardStatus));
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    return HAL_SDMMC_ERROR_NONE;
}

/**
  * @brief  Send Status info command.
  * @param  hsd: pointer to SD handle
  * @param  pSDstatus: Pointer to the buffer that will contain the SD card status
  *         SD Status register
  * @retval error state
  */
static uint32_t SDMMC_SendSDStatus(SDMMC_HandleTypeDef *hsd, uint32_t *pSDstatus)
{
    uint32_t errorstate;
    uint32_t count;
    uint32_t *pData = pSDstatus;

    /* Check SD response */
    if((SDIO_GetResponse(hsd->Instance, SDIO_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED) {
        return HAL_SDMMC_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Set block size for card if it is not equal to current block size for card */
    errorstate = SDIO_CmdBlockLength(hsd->Instance, 64U);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_NONE;
        return errorstate;
    }

    /* Send CMD55 */
    errorstate = SDIO_CmdAppCommand(hsd->Instance, (uint32_t)(hsd->CardInfo.RelCardAdd << 16U));
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_NONE;
        return errorstate;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    SDIO_TxRxConfig(hsd->Instance, SDIO_DIR_READ, SDIO_1LINE, 1, 64, SDIO_NORMAL_MODE);

    /* Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA */
    errorstate = SDIO_CmdStatusRegister(hsd->Instance, 0);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_NONE;
        return errorstate;
    }

    /* Get status data */
    for(int i = 0; i < 8; i++) {
        while(SDIO_GET_IP_FLAG(hsd->Instance, SDIO_IP_RXEMPTY));
        pData[i] = SDIO_ReadData(hsd->Instance);
        i++;
    }

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(hsd->Instance, NULL);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Clear all the static status flags*/
    SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);

    return HAL_SDMMC_ERROR_NONE;
}

static uint32_t SDMMC_PowerON(SDMMC_HandleTypeDef *hsd)
{
    uint32_t i = 0;
    uint32_t SDType = SDMMC_STD_CAPACITY;
    uint32_t emmcType;
    uint32_t errorstate;
    uint32_t response = 0, count = 0, validvoltage = 0, status = 0;

    /* CMD0: GO_IDLE_STATE */
    errorstate = SDIO_CmdGoIdleState(hsd->Instance);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }
    if (SDIO_MULTIMEDIA_CARD == hsd->Sdio_cfg.CardType) {
        while (validvoltage == 0U) {
            /* SEND CMD1 APP_CMD with MMC_HIGH_VOLTAGE_RANGE(0xC0FF8080) as argument */
            do {
                errorstate = SDIO_CmdOpCondition(hsd->Instance, eMMC_HIGH_VOLTAGE_RANGE, &response);
            } while (errorstate != HAL_SDMMC_ERROR_NONE);

            /* Get operating voltage*/
            validvoltage = (((response >> 31U) == 1U) ? 1U : 0U);

            if (count++ == SDMMC_MAX_VOLT_TRIAL) {
                return HAL_SDMMC_ERROR_INVALID_VOLTRANGE;
            }
        }

        /* When power routine finished & command returns valid voltage */
        if (((response & (0xFF000000U)) >> 24) == 0xC0U) {
            hsd->CardInfo.CardType = MMC_HIGH_CAPACITY_CARD;
        } else {
            hsd->CardInfo.CardType = MMC_LOW_CAPACITY_CARD;
        }
    } else {
        /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
        errorstate = SDIO_CmdOperCond(hsd->Instance);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            
            hsd->CardInfo.CardVersion = CARD_V1_X;
            /* CMD0: GO_IDLE_STATE */
            errorstate = SDIO_CmdGoIdleState(hsd->Instance);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                return errorstate;
            }
        }

        if(errorstate == HAL_SDMMC_ERROR_NONE) {
            hsd->CardInfo.CardVersion = CARD_V2_X;
            
        }

        if(hsd->CardInfo.CardVersion == CARD_V2_X) {
            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDIO_CmdAppCommand(hsd->Instance, 0);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                return HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE;
            }
        }

        /* SD CARD */
        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while((!validvoltage) && (count<SDMMC_MAX_VOLT_TRIAL)) {
            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDIO_CmdAppCommand(hsd->Instance, 0);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                return errorstate;
            }
            /* Send CMD41 and Get command response */
            errorstate = SDIO_CmdAppOperCommand(hsd->Instance, SDIO_VOLTAGE_WINDOW_SD | SDIO_HIGH_CAPACITY | SD_SWITCH_1_8V_CAPACITY, &response);

            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                return errorstate;
            }
            /* Get operating voltage*/
            validvoltage = (((response  >>  31) == 1) ? 1U : 0U);
            count++;
        }
        if(count >= SDMMC_MAX_VOLT_TRIAL) {
            errorstate = HAL_SDMMC_ERROR_INVALID_VOLTRANGE;
            return errorstate;
        }

        if(response &= SDMMC_HIGH_CAPACITY) {
            
            hsd->CardInfo.CardType = CARD_SDHC_SDXC;
        } else {
            hsd->CardInfo.CardType = CARD_SDSC;
        }
    }
    return errorstate;
}

static uint32_t SDMMC_InitializeCards(SDMMC_HandleTypeDef *hsd)
{
    HAL_SD_CardStatusTypeDef CardStatus;
    HAL_SDMMC_CardCSDTypeDef CSD;
    HAL_SDMMC_CardCIDTypeDef CID;
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;
    uint16_t rca = 0x01U;

    if(SDIO_SECURE_DIGITAL_IO_CARD != hsd->Sdio_cfg.CardType) {
        /* Send CMD2 ALL_SEND_CID */
        errorstate = SDIO_CmdSendCID(hsd->Instance, hsd->CID);

        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }
    }

    if(SDIO_SECURE_DIGITAL_IO_CARD != hsd->Sdio_cfg.CardType) {
        if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == hsd->Sdio_cfg.CardType) ||    \
            (SDIO_STD_CAPACITY_SD_CARD_V2_0 == hsd->Sdio_cfg.CardType) ||   \
            (SDIO_HIGH_CAPACITY_SD_CARD == hsd->Sdio_cfg.CardType)) {
            /* Send CMD3 SET_REL_ADDR with argument 0 */
            /* SD Card publishes its RCA. */
            errorstate = SDIO_CmdGetRelAdd(hsd->Instance, &rca);
        } else if ((SDIO_MULTIMEDIA_CARD == hsd->Sdio_cfg.CardType) ||  \
            (SDIO_HIGH_CAPACITY_MMC_CARD == hsd->Sdio_cfg.CardType)) {
            /* Send CMD3 SET_REL_ADDR with argument rca to set mmc rca */
            errorstate = SDIO_CmdSetRelAdd(hsd->Instance, rca<<16);
        }

        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }
        /* Get the SD card RCA */
        hsd->CardInfo.RelCardAdd = rca;
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != hsd->Sdio_cfg.CardType) {
        /* Send CMD9 SEND_CSD with argument as card's RCA */
        errorstate = SDIO_CmdSendCSD(hsd->Instance, (uint32_t)(hsd->CardInfo.RelCardAdd << 16U), hsd->CSD);

        /* Get the Card Class */
        hsd->CardInfo.Class = (hsd->CSD[1] >> 20U);

        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }
    }

    /* Send CDM7:Select the Card */
    errorstate = SDIO_CmdSelDesel(hsd->Instance, (uint32_t)(((uint32_t)hsd->CardInfo.RelCardAdd) << 16U));
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Get CSD parameters */
    if (HAL_SDMMC_GetCardCSD(hsd, &CSD) != HAL_SDMMC_OK) {
        return HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE;
    }

    /* Get CID parameters */
    if (HAL_SDMMC_GetCardCID(hsd, &CID) != HAL_SDMMC_OK) {
        return HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE;
    }

    /* Check SD card status */
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == hsd->Sdio_cfg.CardType) ||    \
        (SDIO_STD_CAPACITY_SD_CARD_V2_0 == hsd->Sdio_cfg.CardType) ||   \
        (SDIO_HIGH_CAPACITY_SD_CARD == hsd->Sdio_cfg.CardType)) {
        errorstate = HAL_SD_GetCardStatus(hsd, &CardStatus);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }
    }

    /* All cards are initialized */
    return HAL_SDMMC_ERROR_NONE;
}

/**
  * @brief  Returns information the information of the card which are stored on
  *         the CID register.
  * @param  hsd: Pointer to SD handle
  * @param  pCID: Pointer to a HAL_SDMMC_CardCIDTypeDef structure that
  *         contains all CID register parameters
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SDMMC_GetCardCID(SDMMC_HandleTypeDef *hsd, HAL_SDMMC_CardCIDTypeDef *pCID)
{
    pCID->ManufacturerID = (uint8_t)((hsd->CID[0] & 0xFF000000U) >> 24U);

    pCID->OEM_AppliID = (uint16_t)((hsd->CID[0] & 0x00FFFF00U) >> 8U);

    pCID->ProdName1 = (((hsd->CID[0] & 0x000000FFU) << 24U) | ((hsd->CID[1] & 0xFFFFFF00U) >> 8U));

    pCID->ProdName2 = (uint8_t)(hsd->CID[1] & 0x000000FFU);

    pCID->ProdRev = (uint8_t)((hsd->CID[2] & 0xFF000000U) >> 24U);

    pCID->ProdSN = (((hsd->CID[2] & 0x00FFFFFFU) << 8U) | ((hsd->CID[3] & 0xFF000000U) >> 24U));

    pCID->Reserved1 = (uint8_t)((hsd->CID[3] & 0x00F00000U) >> 20U);

    pCID->ManufactDate = (uint16_t)((hsd->CID[3] & 0x000FFF00U) >> 8U);

    pCID->CID_CRC = (uint8_t)((hsd->CID[3] & 0x000000FEU) >> 1U);

    pCID->Reserved2 = 1U;

    return HAL_SDMMC_OK;
}

/**
  * @brief  Returns information the information of the card which are stored on
  *         the CSD register.
  * @param  hsd: Pointer to SD handle
  * @param  pCSD: Pointer to a HAL_SDMMC_CardCSDTypeDef structure that
  *         contains all CSD register parameters
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SDMMC_GetCardCSD(SDMMC_HandleTypeDef *hsd, HAL_SDMMC_CardCSDTypeDef *pCSD)
{
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;

    if (SDIO_MULTIMEDIA_CARD == hsd->Sdio_cfg.CardType) {
        /* MMC get mmc ext_csd register info */
        errorstate = MMC_ReadExtCsd(hsd);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            hsd->State = HAL_SDMMC_STATE_READY;
            hsd->ErrorCode |= errorstate;
            return HAL_SDMMC_ERR;
        }
    }

    pCSD->CSDStruct = (uint8_t)((hsd->CSD[0] & 0xC0000000U) >> 30U);

    pCSD->SysSpecVersion = (uint8_t)((hsd->CSD[0] & 0x3C000000U) >> 26U);

    pCSD->Reserved1 = (uint8_t)((hsd->CSD[0] & 0x03000000U) >> 24U);

    pCSD->TAAC = (uint8_t)((hsd->CSD[0] & 0x00FF0000U) >> 16U);

    pCSD->NSAC = (uint8_t)((hsd->CSD[0] & 0x0000FF00U) >> 8U);

    pCSD->MaxBusClkFrec = (uint8_t)(hsd->CSD[0] & 0x000000FFU);

    pCSD->CardComdClasses = (uint16_t)((hsd->CSD[1] & 0xFFF00000U) >> 20U);

    pCSD->RdBlockLen = (uint8_t)((hsd->CSD[1] & 0x000F0000U) >> 16U);

    pCSD->PartBlockRead   = (uint8_t)((hsd->CSD[1] & 0x00008000U) >> 15U);

    pCSD->WrBlockMisalign = (uint8_t)((hsd->CSD[1] & 0x00004000U) >> 14U);

    pCSD->RdBlockMisalign = (uint8_t)((hsd->CSD[1] & 0x00002000U) >> 13U);

    pCSD->DSRImpl = (uint8_t)((hsd->CSD[1] & 0x00001000U) >> 12U);

    pCSD->Reserved2 = 0U; /*!< Reserved */

    if(hsd->CardInfo.CardType == CARD_SDSC) {
        pCSD->DeviceSize = (((hsd->CSD[1] & 0x000003FFU) << 2U) | ((hsd->CSD[2] & 0xC0000000U) >> 30U));

        pCSD->MaxRdCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x38000000U) >> 27U);

        pCSD->MaxRdCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x07000000U) >> 24U);

        pCSD->MaxWrCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x00E00000U) >> 21U);

        pCSD->MaxWrCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x001C0000U) >> 18U);

        pCSD->DeviceSizeMul = (uint8_t)((hsd->CSD[2] & 0x00038000U) >> 15U);

        hsd->CardInfo.BlockNbr  = (pCSD->DeviceSize + 1U) ;
        hsd->CardInfo.BlockNbr *= (1UL << ((pCSD->DeviceSizeMul & 0x07U) + 2U));
        hsd->CardInfo.BlockSize = (1UL << (pCSD->RdBlockLen & 0x0FU));

        hsd->CardInfo.LogBlockNbr =  (hsd->CardInfo.BlockNbr) * ((hsd->CardInfo.BlockSize) / 512U);
        hsd->CardInfo.LogBlockSize = 512U;
    } else if(hsd->CardInfo.CardType == CARD_SDHC_SDXC) {
        /* Byte 7 */
        pCSD->DeviceSize = (((hsd->CSD[1] & 0x0000003FU) << 16U) | ((hsd->CSD[2] & 0xFFFF0000U) >> 16U));

        hsd->CardInfo.BlockNbr = ((pCSD->DeviceSize + 1U) * 1024U);
        hsd->CardInfo.LogBlockNbr = hsd->CardInfo.BlockNbr;
        hsd->CardInfo.BlockSize = 512U;
        hsd->CardInfo.LogBlockSize = hsd->CardInfo.BlockSize;
    } else if(hsd->CardInfo.CardType == MMC_LOW_CAPACITY_CARD) {
        pCSD->DeviceSize = (((hsd->CSD[1] & 0x000003FFU) << 2U) | ((hsd->CSD[2] & 0xC0000000U) >> 30U));

        pCSD->MaxRdCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x38000000U) >> 27U);

        pCSD->MaxRdCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x07000000U) >> 24U);

        pCSD->MaxWrCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x00E00000U) >> 21U);

        pCSD->MaxWrCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x001C0000U) >> 18U);

        pCSD->DeviceSizeMul = (uint8_t)((hsd->CSD[2] & 0x00038000U) >> 15U);

        hsd->CardInfo.BlockNbr  = (pCSD->DeviceSize + 1U) ;
        hsd->CardInfo.BlockNbr *= (1UL << ((pCSD->DeviceSizeMul & 0x07U) + 2U));
        hsd->CardInfo.BlockSize = (1UL << (pCSD->RdBlockLen & 0x0FU));

        hsd->CardInfo.LogBlockNbr =  (hsd->CardInfo.BlockNbr) * ((hsd->CardInfo.BlockSize) / 512U);
        hsd->CardInfo.LogBlockSize = 512U;
    } else if (hsd->CardInfo.CardType == MMC_HIGH_CAPACITY_CARD) {
        hsd->CardInfo.BlockNbr = hsd->MMCExtCSD[53];
        hsd->CardInfo.LogBlockNbr = hsd->CardInfo.BlockNbr;
        hsd->CardInfo.BlockSize = 512U;
        hsd->CardInfo.LogBlockSize = hsd->CardInfo.BlockSize;
    } else {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE;
        hsd->State = HAL_SDMMC_STATE_READY;
        return HAL_SDMMC_ERR;
    }

    pCSD->EraseGrSize = (uint8_t)((hsd->CSD[2] & 0x00004000U) >> 14U);

    pCSD->EraseGrMul = (uint8_t)((hsd->CSD[2] & 0x00003F80U) >> 7U);

    pCSD->WrProtectGrSize = (uint8_t)(hsd->CSD[2] & 0x0000007FU);

    pCSD->WrProtectGrEnable = (uint8_t)((hsd->CSD[3] & 0x80000000U) >> 31U);

    pCSD->ManDeflECC = (uint8_t)((hsd->CSD[3] & 0x60000000U) >> 29U);

    pCSD->WrSpeedFact = (uint8_t)((hsd->CSD[3] & 0x1C000000U) >> 26U);

    pCSD->MaxWrBlockLen= (uint8_t)((hsd->CSD[3] & 0x03C00000U) >> 22U);

    pCSD->WriteBlockPaPartial = (uint8_t)((hsd->CSD[3] & 0x00200000U) >> 21U);

    pCSD->Reserved3 = 0;

    pCSD->ContentProtectAppli = (uint8_t)((hsd->CSD[3] & 0x00010000U) >> 16U);

    pCSD->FileFormatGroup = (uint8_t)((hsd->CSD[3] & 0x00008000U) >> 15U);

    pCSD->CopyFlag = (uint8_t)((hsd->CSD[3] & 0x00004000U) >> 14U);

    pCSD->PermWrProtect = (uint8_t)((hsd->CSD[3] & 0x00002000U) >> 13U);

    pCSD->TempWrProtect = (uint8_t)((hsd->CSD[3] & 0x00001000U) >> 12U);

    pCSD->FileFormat = (uint8_t)((hsd->CSD[3] & 0x00000C00U) >> 10U);

    pCSD->ECC= (uint8_t)((hsd->CSD[3] & 0x00000300U) >> 8U);

    pCSD->CSD_CRC = (uint8_t)((hsd->CSD[3] & 0x000000FEU) >> 1U);

    pCSD->Reserved4 = 1;

    return HAL_SDMMC_OK;
}

/**
  * @brief  Gets the SD status info.
  * @param  hsd: Pointer to SD handle
  * @param  pStatus: Pointer to the HAL_SD_CardStatusTypeDef structure that 
  *         will contain the SD card status information
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SD_GetCardStatus(SDMMC_HandleTypeDef *hsd, HAL_SD_CardStatusTypeDef *pStatus)
{
    uint32_t errorstate;

    errorstate = SDMMC_SendSDStatus(hsd, hsd->SDCardStatus);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        /* Clear all the static flags */
        SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
        hsd->ErrorCode |= errorstate;
        hsd->State = HAL_SDMMC_STATE_READY;
        return HAL_SDMMC_ERR;
    } else {
        pStatus->DataBusWidth = (uint8_t)((hsd->SDCardStatus[0] & 0xC0U) >> 6U);

        pStatus->SecuredMode = (uint8_t)((hsd->SDCardStatus[0] & 0x20U) >> 5U);

        pStatus->CardType = (uint16_t)(((hsd->SDCardStatus[0] & 0x00FF0000U) >> 8U) | ((hsd->SDCardStatus[0] & 0xFF000000U) >> 24U));

        pStatus->ProtectedAreaSize = (((hsd->SDCardStatus[1] & 0xFFU) << 24U)    | ((hsd->SDCardStatus[1] & 0xFF00U) << 8U) |
                                    ((hsd->SDCardStatus[1] & 0xFF0000U) >> 8U) | ((hsd->SDCardStatus[1] & 0xFF000000U) >> 24U));

        pStatus->SpeedClass = (uint8_t)(hsd->SDCardStatus[2] & 0xFFU);

        pStatus->PerformanceMove = (uint8_t)((hsd->SDCardStatus[2] & 0xFF00U) >> 8U);

        pStatus->AllocationUnitSize = (uint8_t)((hsd->SDCardStatus[2] & 0xF00000U) >> 20U);

        pStatus->EraseSize = (uint16_t)(((hsd->SDCardStatus[2] & 0xFF000000U) >> 16U) | (hsd->SDCardStatus[3] & 0xFFU));

        pStatus->EraseTimeout = (uint8_t)((hsd->SDCardStatus[3] & 0xFC00U) >> 10U);

        pStatus->EraseOffset = (uint8_t)((hsd->SDCardStatus[3] & 0x0300U) >> 8U);

        pStatus->UhsSpeedGrade = (uint8_t)((hsd->SDCardStatus[3] & 0x00F0U) >> 4U);
        pStatus->UhsAllocationUnitSize = (uint8_t)(hsd->SDCardStatus[3] & 0x000FU) ;
        pStatus->VideoSpeedClass = (uint8_t)((hsd->SDCardStatus[4] & 0xFF000000U) >> 24U);
    }

    return HAL_SDMMC_ERROR_NONE;
}

static uint32_t MMC_ReadExtCsd(SDMMC_HandleTypeDef *hsd)
{
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;
    uint32_t count = 0;
    uint32_t *ExtCsdBuf;

    ExtCsdBuf = (uint32_t *)(&(hsd->MMCExtCSD));

    /* Config Date Channel */
    SDIO_ClearDataSetup(hsd->Instance);
    SDIO_TxRxConfig(hsd->Instance, SDIO_DIR_READ, SDIO_1LINE, 1, BLOCKSIZE, SDIO_NORMAL_MODE);

    /* Send CMD8 to get ext_csd */
    errorstate = SDIO_CmdSendEXTCSD(hsd->Instance, 0);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->State = HAL_SDMMC_STATE_READY;
        hsd->ErrorCode |= errorstate;
        return HAL_SDMMC_ERR;
    }

    while (count < (BLOCKSIZE / 4)) {
        while (SDIO_GET_IP_FLAG(hsd->Instance, SDIO_IP_RXEMPTY)) {
        }
        *(ExtCsdBuf + count) = SDIO_ReadData(hsd->Instance);
        count++;
    }

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(hsd->Instance, NULL);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
    SDIO_ClearDataSetup(hsd->Instance);

    return errorstate;
}

HAL_SDMMCStatusTypeDef SDMMC_Init(SDMMC_HandleTypeDef *hsd)
{
    volatile uint32_t errorstate = HAL_SDMMC_ERROR_NONE;

    SDIO_PowerState_ON(hsd->Instance);
    SDIO_ClockDivConfig(hsd->Instance, 0x2);

    /* Set Power State to ON */
    /* wait 74 Cycles: required power up waiting time before starting the SD initialization sequence(hardware opearation) */
    errorstate = SDMMC_PowerON(hsd);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->State = HAL_SDMMC_STATE_READY;
        hsd->ErrorCode |= errorstate;
        return HAL_SDMMC_ERR;
    }

    /* Card initialization */
    errorstate = SDMMC_InitializeCards(hsd);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->State = HAL_SDMMC_STATE_READY;
        hsd->ErrorCode |= errorstate;
        return HAL_SDMMC_ERR;
    }

    /* Configure the bus wide */
    errorstate = SDMMC_ConfigWideBusOperation(hsd);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        hsd->State = HAL_SDMMC_STATE_READY;
        hsd->ErrorCode |= errorstate;
        return HAL_SDMMC_ERR;
    }

    /* Initialize the error code */
    hsd->ErrorCode = HAL_SDMMC_ERROR_NONE;

    /* Initialize the SD state */
    hsd->State = HAL_SDMMC_STATE_READY;

    return HAL_SDMMC_OK;
}

void SDMMC_GetCardInfo(SDMMC_HandleTypeDef hsd, SDMMC_CardInfoTypeDef *CardInfo)
{
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;
    uint8_t tmp = 0;
    /* Get Card type and RCA */
    CardInfo->CardType = hsd.CardInfo.CardType;
    CardInfo->RCA = hsd.CardInfo.RelCardAdd;
    /* Get CSD structure */
    /*!< Byte 0 */
    tmp = ADDR8(((hsd.CSD[0] & 0xFF000000) >> 24));
    CardInfo->CardCSD.CSDStruct = (tmp & 0xC0) >> 6;
    CardInfo->CardCSD.SysSpecVersion = (tmp & 0x3C) >> 2;
    CardInfo->CardCSD.Reserved1 = tmp & 0x03;
    /*!< Byte 1 */
    tmp = ADDR8((hsd.CSD[0] & 0x00FF0000) >> 16);
    CardInfo->CardCSD.TAAC = tmp;
    /*!< Byte 2 */
    tmp = ADDR8((hsd.CSD[0] & 0x0000FF00) >> 8);
    CardInfo->CardCSD.NSAC = tmp;
    /*!< Byte 3 */
    tmp = ADDR8(hsd.CSD[0] & 0x000000FF);
    CardInfo->CardCSD.MaxBusClkFrec = tmp;
    /*!< Byte 4 */
    tmp = ADDR8((hsd.CSD[1] & 0xFF000000) >> 24);
    CardInfo->CardCSD.CardComdClasses = tmp << 4;
    /*!< Byte 5 */
    tmp = ADDR8((hsd.CSD[1] & 0x00FF0000) >> 16);
    CardInfo->CardCSD.CardComdClasses |= (tmp & 0xF0) >> 4;
    CardInfo->CardCSD.RdBlockLen = tmp & 0x0F;
    /*!< Byte 6 */
    tmp = ADDR8((hsd.CSD[1] & 0x0000FF00) >> 8);
    CardInfo->CardCSD.PartBlockRead = (tmp & 0x80) >> 7;
    CardInfo->CardCSD.WrBlockMisalign = (tmp & 0x40) >> 6;
    CardInfo->CardCSD.RdBlockMisalign = (tmp & 0x20) >> 5;
    CardInfo->CardCSD.DSRImpl = (tmp & 0x10) >> 4;
    CardInfo->CardCSD.Reserved2 = 0; /*!< Reserved */
    if((CardInfo->CardType == MMC_LOW_CAPACITY_CARD) || (CardInfo->CardType == CARD_SDSC)) {
        /*!< Byte 7 */
        CardInfo->CardCSD.DeviceSize = (tmp & 0x03) << 10;
        tmp = ADDR8(hsd.CSD[1] & 0x000000FF);
        /*!< Byte 8 */
        CardInfo->CardCSD.DeviceSize |= (tmp) << 2;
        tmp = ADDR8((hsd.CSD[2] & 0xFF000000) >> 24);
        CardInfo->CardCSD.DeviceSize |= (tmp & 0xC0) >> 6;
        CardInfo->CardCSD.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        CardInfo->CardCSD.MaxRdCurrentVDDMax = (tmp & 0x07);
		/*!< Byte 9 */
        tmp = ADDR8((hsd.CSD[2] & 0x00FF0000) >> 16);
        CardInfo->CardCSD.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        CardInfo->CardCSD.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        CardInfo->CardCSD.DeviceSizeMul = (tmp & 0x03) << 1;
		/*!< Byte 10 */
        tmp = ADDR8((hsd.CSD[2] & 0x0000FF00) >> 8);
        CardInfo->CardCSD.DeviceSizeMul |= (tmp & 0x80) >> 7;
        CardInfo->CardCapacity = (CardInfo->CardCSD.DeviceSize+1);
        CardInfo->CardCapacity *= (1 << (CardInfo->CardCSD.DeviceSizeMul+2));
        CardInfo->CardBlockSize = 1 << (CardInfo->CardCSD.RdBlockLen);
        CardInfo->CardCapacity *= CardInfo->CardBlockSize;
    } else if (CardInfo->CardType == CARD_SDHC_SDXC) {
        tmp = ADDR8(hsd.CSD[1] & 0x000000FF);
        CardInfo->CardCSD.DeviceSize = (tmp & 0x3F) << 16;
        tmp = ADDR8((hsd.CSD[2] & 0xFF000000) >> 24);
        CardInfo->CardCSD.DeviceSize |= (tmp << 8);
        tmp = ADDR8((hsd.CSD[2] & 0x00FF0000) >> 16);
        CardInfo->CardCSD.DeviceSize |= (tmp);
        tmp = ADDR8((hsd.CSD[2] & 0x0000FF00) >> 8);
        CardInfo->CardCapacity = (long long)(CardInfo->CardCSD.DeviceSize+1)*512*1024;
        CardInfo->CardBlockSize = 512;
    }

    CardInfo->CardCSD.EraseGrSize = (tmp & 0x40) >> 6;
    CardInfo->CardCSD.EraseGrMul = (tmp & 0x3F) << 1;
    /*!< Byte 11 */
    tmp = ADDR8(hsd.CSD[2] & 0x000000FF);
    CardInfo->CardCSD.EraseGrMul |= (tmp & 0x80) >> 7;
    CardInfo->CardCSD.WrProtectGrSize = (tmp & 0x7F);
    tmp = ADDR8((hsd.CSD[3] & 0xFF000000) >> 24);
    CardInfo->CardCSD.WrProtectGrEnable = (tmp & 0x80) >> 7;
    CardInfo->CardCSD.ManDeflECC = (tmp & 0x60) >> 5;
    CardInfo->CardCSD.WrSpeedFact = (tmp & 0x1C) >> 2;
    CardInfo->CardCSD.MaxWrBlockLen = (tmp & 0x03) << 2;
    /*!< Byte 13 */
    tmp = ADDR8((hsd.CSD[3] & 0x00FF0000) >> 16);
    CardInfo->CardCSD.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    CardInfo->CardCSD.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    CardInfo->CardCSD.Reserved3 = 0;
    CardInfo->CardCSD.ContentProtectAppli = (tmp & 0x01);
    /*!< Byte 14 */
    tmp = ADDR8((hsd.CSD[3] & 0x0000FF00) >> 8);
    CardInfo->CardCSD.FileFormatGroup = (tmp & 0x80) >> 7;
    CardInfo->CardCSD.CopyFlag = (tmp & 0x40) >> 6;
    CardInfo->CardCSD.PermWrProtect = (tmp & 0x20) >> 5;
    CardInfo->CardCSD.TempWrProtect = (tmp & 0x10) >> 4;
    CardInfo->CardCSD.FileFormat = (tmp & 0x0C) >> 2;
    CardInfo->CardCSD.ECC = (tmp & 0x03);
    /*!< Byte 15 */
    tmp = ADDR8(hsd.CSD[3] & 0x000000FF);
    CardInfo->CardCSD.CSD_CRC = (tmp & 0xFE) >> 1;
    CardInfo->CardCSD.Reserved4 = 1;
    tmp = ADDR8((hsd.CID[0] & 0xFF000000) >> 24);
    CardInfo->CardCID.ManufacturerID = tmp;
    /* Get CID structure */
    /*!< Byte 1 */
    tmp = ADDR8((hsd.CID[0] & 0x00FF0000) >> 16);
    CardInfo->CardCID.OEM_AppliID = tmp << 8;
    /*!< Byte 2 */
    tmp = ADDR8((hsd.CID[0] & 0x000000FF00) >> 8);
    CardInfo->CardCID.OEM_AppliID |= tmp;
    /*!< Byte 3 */
    tmp = ADDR8(hsd.CID[0] & 0x000000FF);
    CardInfo->CardCID.ProdName1 = tmp << 24;
    /*!< Byte 4 */
    tmp = ADDR8((hsd.CID[1] & 0xFF000000) >> 24);
    CardInfo->CardCID.ProdName1 |= tmp << 16;
    /*!< Byte 5 */
    tmp = ADDR8((hsd.CID[1] & 0x00FF0000) >> 16);
    CardInfo->CardCID.ProdName1 |= tmp << 8;
    /*!< Byte 6 */
    tmp = ADDR8((hsd.CID[1] & 0x0000FF00) >> 8);
    CardInfo->CardCID.ProdName1 |= tmp;
    /*!< Byte 7 */
    tmp = ADDR8(hsd.CID[1] & 0x000000FF);
    CardInfo->CardCID.ProdName2 = tmp;
    /*!< Byte 8 */
    tmp = ADDR8((hsd.CID[2] & 0xFF000000) >> 24);
    CardInfo->CardCID.ProdRev = tmp;
    /*!< Byte 9 */
    tmp = ADDR8((hsd.CID[2] & 0x00FF0000) >> 16);
    CardInfo->CardCID.ProdSN = tmp << 24;
    /*!< Byte 10 */
    tmp = ADDR8((hsd.CID[2] & 0x0000FF00) >> 8);
    CardInfo->CardCID.ProdSN |= tmp << 16;
    /*!< Byte 11 */
    tmp = ADDR8(hsd.CID[2] & 0x000000FF);
    CardInfo->CardCID.ProdSN |= tmp << 8;
    /*!< Byte 12 */
    tmp = ADDR8((hsd.CID[3] & 0xFF000000) >> 24);
    CardInfo->CardCID.ProdSN |= tmp;
    /*!< Byte 13 */
    tmp = ADDR8((hsd.CID[3] & 0x00FF0000) >> 16);
    CardInfo->CardCID.Reserved1 |= (tmp & 0xF0) >> 4;
    CardInfo->CardCID.ManufactDate = (tmp & 0x0F) << 8;
    /*!< Byte 14 */
    tmp = ADDR8((hsd.CID[3] & 0x0000FF00) >> 8);
    CardInfo->CardCID.ManufactDate |= tmp;
    /*!< Byte 15 */
    tmp = ADDR8(hsd.CID[3] & 0x000000FF);
    CardInfo->CardCID.CID_CRC = (tmp & 0xFE) >> 1;
    CardInfo->CardCID.Reserved2 = 1;

    if(MMC_HIGH_CAPACITY_CARD == CardInfo->CardType) {
        memcpy(&CardInfo->MMCExtCSD, hsd.MMCExtCSD, sizeof(hsd.MMCExtCSD));
        CardInfo->CardBlockSize = hsd.CardInfo.BlockSize;
        CardInfo->CardCapacity = (uint64_t)((uint64_t)(CardInfo->MMCExtCSD.EXT_CSD.SEC_COUNT[3] << 24 | \
                                CardInfo->MMCExtCSD.EXT_CSD.SEC_COUNT[2] << 16 | \
                                CardInfo->MMCExtCSD.EXT_CSD.SEC_COUNT[1] << 8 | \
                                CardInfo->MMCExtCSD.EXT_CSD.SEC_COUNT[0]) * CardInfo->CardBlockSize);
    }
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == hsd.Sdio_cfg.CardType) || \
        (SDIO_STD_CAPACITY_SD_CARD_V2_0 == hsd.Sdio_cfg.CardType) || \
        (SDIO_HIGH_CAPACITY_SD_CARD == hsd.Sdio_cfg.CardType)) {
        CardInfo->SDCardStatus.DataBusWidth = (uint8_t)((hsd.SDCardStatus[0] & 0xC0U) >> 6U);

        CardInfo->SDCardStatus.SecuredMode = (uint8_t)((hsd.SDCardStatus[0] & 0x20U) >> 5U);

        CardInfo->SDCardStatus.CardType = (uint16_t)(((hsd.SDCardStatus[0] & 0x00FF0000U) >> 8U) | ((hsd.SDCardStatus[0] & 0xFF000000U) >> 24U));

        CardInfo->SDCardStatus.ProtectedAreaSize = (((hsd.SDCardStatus[1] & 0xFFU) << 24U)    | ((hsd.SDCardStatus[1] & 0xFF00U) << 8U) |
                                    ((hsd.SDCardStatus[1] & 0xFF0000U) >> 8U) | ((hsd.SDCardStatus[1] & 0xFF000000U) >> 24U));

        CardInfo->SDCardStatus.SpeedClass = (uint8_t)(hsd.SDCardStatus[2] & 0xFFU);

        CardInfo->SDCardStatus.PerformanceMove = (uint8_t)((hsd.SDCardStatus[2] & 0xFF00U) >> 8U);

        CardInfo->SDCardStatus.AllocationUnitSize = (uint8_t)((hsd.SDCardStatus[2] & 0xF00000U) >> 20U);

        CardInfo->SDCardStatus.EraseSize = (uint16_t)(((hsd.SDCardStatus[2] & 0xFF000000U) >> 16U) | (hsd.SDCardStatus[3] & 0xFFU));

        CardInfo->SDCardStatus.EraseTimeout = (uint8_t)((hsd.SDCardStatus[3] & 0xFC00U) >> 10U);

        CardInfo->SDCardStatus.EraseOffset = (uint8_t)((hsd.SDCardStatus[3] & 0x0300U) >> 8U);

        CardInfo->SDCardStatus.UhsSpeedGrade = (uint8_t)((hsd.SDCardStatus[3] & 0x00F0U) >> 4U);
        CardInfo->SDCardStatus.UhsAllocationUnitSize = (uint8_t)(hsd.SDCardStatus[3] & 0x000FU) ;
        CardInfo->SDCardStatus.VideoSpeedClass = (uint8_t)((hsd.SDCardStatus[4] & 0xFF000000U) >> 24U);
    }
}

static uint32_t FindSCR(SDMMC_HandleTypeDef *hsd, uint16_t rca, uint32_t *pscr)
{
    uint32_t index = 0;
    uint32_t errorstate;
    uint32_t tempscr[2] = {0, 0};

    errorstate = SDIO_CmdBlockLength(hsd->Instance, 64);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Send CMD55 APP_CMD with argument as card's RCA.*/
    SDIO_CmdAppCommand(hsd->Instance, ADDR32( hsd->CardInfo.RelCardAdd << 16));
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    SDIO_TxRxConfig(hsd->Instance, SDIO_DIR_READ, hsd->Sdio_cfg.BusWidth, 1, 64, SDIO_NORMAL_MODE);

    errorstate = SDIO_CmdSendSCR(hsd->Instance);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    do {
        while(SDIO_GET_IP_FLAG(hsd->Instance, SDIO_IP_RXEMPTY));
        tempscr[index] = SDIO_ReadData(hsd->Instance);
        index ++;
    } while(index < 2);

    errorstate = SDIO_GetCmdResp1(hsd->Instance, NULL);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    *(pscr+1) = ((tempscr[0] & SDMMC_0TO7BITS) << 24) | ((tempscr[0] & SDMMC_8TO15BITS) << 8) | ((tempscr[0] & SDMMC_16TO23BITS) >> 8) | ((tempscr[0] & SDMMC_24TO31BITS) >> 24);
    *(pscr) = ((tempscr[1] & SDMMC_0TO7BITS) << 24) | ((tempscr[1] & SDMMC_8TO15BITS) << 8) | ((tempscr[1] & SDMMC_16TO23BITS) >> 8) | ((tempscr[1] & SDMMC_24TO31BITS) >> 24);
    return errorstate;
}

static uint32_t MMC_WideBusConfig(SDMMC_HandleTypeDef *hsd)
{
    /* SDMMC_CMD_APP_SD_SET_BUSWIDTH argument format:
    *
    *	[31:26] Always 0
    *	[25:24] Access Mode
    *	[23:16] Location of target Byte in EXT_CSD
    *	[15:08] Value Byte
    *	[07:03] Always 0
    *	[02:00] Command Set
    */
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;
    uint32_t response = 0U;
    uint32_t cmdArg;

    if (hsd->Sdio_cfg.BusMode == SDIO_DDR_MODE) {
        /* if cfg busmode ddr, enable DDR mode */
        SDIO_CfgDdrMode(hsd->Instance, ENABLE);

        /* send CMD6 to activate hs_timing before setting BUS_WIDTH for dual data rate operation*/
        errorstate = SDIO_CmdBusWidth(hsd->Instance, 0x03B90100);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }
    }

    if (hsd->Sdio_cfg.BusWidth == SDIO_8LINE) {
        if (hsd->Sdio_cfg.BusMode == SDIO_DDR_MODE) {
            cmdArg = 0x03B70601U;
        } else if (hsd->Sdio_cfg.BusMode == SDIO_SDR_MODE) {
            cmdArg = 0x03B70200U;
        } else{
            errorstate = HAL_SDMMC_ERROR_PARAM;
            return errorstate;
        }
    } else if (hsd->Sdio_cfg.BusWidth == SDIO_4LINE) {
        if (hsd->Sdio_cfg.BusMode == SDIO_DDR_MODE) {
            cmdArg = 0x03B70501U;
        } else if (hsd->Sdio_cfg.BusMode == SDIO_SDR_MODE) {
            cmdArg = 0x03B70100U;
        } else{
            errorstate = HAL_SDMMC_ERROR_PARAM;
            return errorstate;
        }
    } else if (hsd->Sdio_cfg.BusWidth == SDIO_1LINE) {
        cmdArg = 0x03B70000U;
    } else {
        /* WideMode is not a valid argument*/
        errorstate = HAL_SDMMC_ERROR_PARAM;
        return errorstate;
    }

    /* Send CMD6 to activate SDR50 Mode and Power Limit 1.44W */
    errorstate = SDIO_CmdBusWidth(hsd->Instance, cmdArg);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Check for switch error and violation of the trial number of sending CMD13 */
    errorstate = SDIO_CmdSendStatus(hsd->Instance, hsd->CardInfo.RelCardAdd << 16, NULL);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    return errorstate;
}

static uint32_t SD_WideBusConfig(SDMMC_HandleTypeDef *hsd)
{
    uint32_t errorstate;
    uint32_t scr[2] = {0, 0};
    uint8_t cmd_arg = hsd->Sdio_cfg.BusWidth ? 0X02 : 0x00;

    if(hsd->Instance->RSP0 & SDMMC_CARD_LOCKED) {
        return HAL_SDMMC_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Get SCR Register */
    errorstate = FindSCR(hsd, hsd->CardInfo.RelCardAdd, scr);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }
    /* Send CMD55 APP_CMD with argument as card's RCA.*/
    SDIO_CmdAppCommand(hsd->Instance, ADDR32( hsd->CardInfo.RelCardAdd << 16));
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
    errorstate = SDIO_CmdBusWidth(hsd->Instance, cmd_arg);
    if(errorstate != HAL_SDMMC_ERROR_NONE) {
        return errorstate;
    }

    return HAL_SDMMC_ERROR_NONE;
}

uint32_t SDMMC_ConfigWideBusOperation(SDMMC_HandleTypeDef *hsd)
{
    uint32_t errorstate = HAL_SDMMC_ERROR_NONE;
    if (SDIO_MULTIMEDIA_CARD == hsd->Sdio_cfg.CardType) {
        errorstate = MMC_WideBusConfig(hsd);
        return errorstate;
    } else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == hsd->Sdio_cfg.CardType) ||    \
               (SDIO_STD_CAPACITY_SD_CARD_V2_0 == hsd->Sdio_cfg.CardType) ||   \
               (SDIO_HIGH_CAPACITY_SD_CARD == hsd->Sdio_cfg.CardType)) {
        if (SDIO_8LINE == hsd->Sdio_cfg.BusWidth) {
            return HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE ;
        } else if ((SDIO_1LINE | SDIO_4LINE) == hsd->Sdio_cfg.BusWidth) {
            errorstate = SD_WideBusConfig(hsd);
            if (HAL_SDMMC_ERROR_NONE != errorstate) {
                return errorstate;
            }
        } else {
            /* WideMode is not a valid argument*/
            hsd->ErrorCode |= HAL_SDMMC_ERROR_PARAM;
        }
    }
    return errorstate;
}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by polling mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @param  hsd: Pointer to SD handle
  * @param  pData: pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  NumberOfBlocks: Number of SD blocks to read
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SDMMC_ReadBlocks(SDMMC_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    SDIO_DataSetupTypeDef SDIO_DataSetupStruct;
    uint32_t errorstate;
    uint32_t count, data, dataremaining;
    uint32_t add = BlockAdd;
    uint32_t *tempbuff = (uint32_t *) (pData);
    uint32_t dataindex = 0;

    if(NULL == pData) {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_PARAM;
        return HAL_SDMMC_ERR;
    }

    if(hsd->State == HAL_SDMMC_STATE_READY) {
        hsd->ErrorCode = HAL_SDMMC_ERROR_NONE;

        hsd->State = HAL_SDMMC_STATE_BUSY;

        /* Initialize data control register */
        SDIO_ClearDataSetup(hsd->Instance);

        if(hsd->CardInfo.CardType != CARD_SDHC_SDXC) {
            add *= 512U;
        }

        /* Set Block Size for Card */
        if(hsd->Sdio_cfg.BusMode != SDIO_DDR_MODE) {
            errorstate = SDIO_CmdBlockLength(hsd->Instance, BLOCKSIZE);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                /* Clear all the static flags */
                SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                hsd->ErrorCode |= errorstate;
                hsd->State = HAL_SDMMC_STATE_READY;
                return HAL_SDMMC_ERR;
            }
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        SDIO_TxRxConfig(hsd->Instance, SDIO_DIR_READ, hsd->Sdio_cfg.BusWidth, NumberOfBlocks, BLOCKSIZE, SDIO_NORMAL_MODE);
        
        SDIO_ClockDivConfig(hsd->Instance, 0x2);

        /* Read block(s) in polling mode */
        if(NumberOfBlocks > 1U) {
            if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
                hsd->Context = SDMMC_CONTEXT_READ_MULTIPLE_BLOCK;
            } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
                hsd->Context = (SDMMC_CONTEXT_READ_MULTIPLE_BLOCK | SDMMC_CONTEXT_DMA);
            }
            /* Set Read Multi Block Count command */
            
            /* Read Multi Block command */
            errorstate = SDIO_CmdReadMultiBlock(hsd->Instance, add);
        } else {
            if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
                hsd->Context = SDMMC_CONTEXT_READ_SINGLE_BLOCK;
            } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
                hsd->Context = (SDMMC_CONTEXT_READ_SINGLE_BLOCK | SDMMC_CONTEXT_DMA);
            }
            /* Read Single Block command */
            errorstate = SDIO_CmdReadSingleBlock(hsd->Instance, add);
        }
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            /* Clear all the static flags */
            SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SDMMC_STATE_READY;
            hsd->Context = SDMMC_CONTEXT_NONE;
            return HAL_SDMMC_ERR;
        }

        if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
            /* Polling data via SDIO flags */
            do {
                while(SDIO_GET_IP_FLAG(hsd->Instance, SDIO_IP_RXEMPTY));
                tempbuff[dataindex] = SDIO_ReadData(hsd->Instance);
                dataindex ++;
            } while(dataindex < (NumberOfBlocks * BLOCKSIZE / 4));
        } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
            SDIO_UdmaEnqueueChannel(hsd->Instance, tempbuff, NumberOfBlocks * BLOCKSIZE, SDIO_RX_CFG_EN_ENABLE|SDIO_RX_CFG_DATASIZE_WORD, RX_CHANNEL);
            SDIO_DmaStart(hsd->Instance, ENABLE);

            while(!SDIO_DmaGetIntStat(hsd->Instance, SDIO_RX_FTRANS_IRQ_STAT));
            SDIO_DmaInterruptClr(hsd->Instance, SDIO_RX_FTRANS_IRQ_CLR);
        }

        /* Check for error conditions */
        errorstate = SDIO_GetCmdResp1(hsd->Instance, NULL);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }

        /* Send stop transmission command in case of multiblock read */
        if(NumberOfBlocks > 1U) {
            if(hsd->CardInfo.CardType != CARD_SECURED) {
                /* Send stop transmission command */
                errorstate = SDIO_CmdStopTransfer(hsd->Instance);
                if(errorstate != HAL_SDMMC_ERROR_NONE) {
                    /* Clear all the static flags */
                    SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                    hsd->ErrorCode |= errorstate;
                    hsd->State = HAL_SDMMC_STATE_READY;
                    hsd->Context = SDMMC_CONTEXT_NONE;
                    return HAL_SDMMC_ERR;
                }
            }
        }

        hsd->State = HAL_SDMMC_STATE_READY;

        /* Clear data channel and all the static flags */
        SDIO_ClearDataSetup(hsd->Instance);
        SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
        return HAL_SDMMC_OK;
    } else {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_BUSY;
        return HAL_SDMMC_ERR;
    }
}

/**
  * @brief  Allows to write block(s) to a specified address in a card. The Data
  *         transfer is managed by polling mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @param  hsd: Pointer to SD handle
  * @param  pData: pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address where data will be written
  * @param  NumberOfBlocks: Number of SD blocks to write
  * @param  Timeout: Specify timeout value
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SDMMC_WriteBlocks(SDMMC_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    uint32_t errorstate;
    uint32_t count, data, dataremaining;
    uint32_t add = BlockAdd;
    uint32_t *tempbuff = (uint32_t *) pData;
    uint32_t writeindex = 0;
    uint32_t pCardStatus;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);
    if(NULL == pData) {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_PARAM;
        return HAL_SDMMC_ERR;
    }

    if(hsd->State == HAL_SDMMC_STATE_READY) {
        hsd->ErrorCode = HAL_SDMMC_ERROR_NONE;

        hsd->State = HAL_SDMMC_STATE_BUSY;

        /* Initialize data control register */
        SDIO_ClearDataSetup(hsd->Instance);

        if(hsd->CardInfo.CardType != CARD_SDHC_SDXC) {
            add *= 512U;
        }

        /* Set Block Size for Card */
        if(hsd->Sdio_cfg.BusMode != SDIO_DDR_MODE) {
            errorstate = SDIO_CmdBlockLength(hsd->Instance, BLOCKSIZE);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                /* Clear all the static flags */
                SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                hsd->ErrorCode |= errorstate;
                hsd->State = HAL_SDMMC_STATE_READY;
                return HAL_SDMMC_ERR;
            }
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        SDIO_TxRxConfig(hsd->Instance, SDIO_DIR_WRITE, hsd->Sdio_cfg.BusWidth, NumberOfBlocks, BLOCKSIZE, SDIO_NORMAL_MODE);
        
        SDIO_ClockDivConfig(hsd->Instance, 0x2);

        /* Write Blocks in Polling mode */
        if(NumberOfBlocks > 1U) {
            if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
                hsd->Context = SDMMC_CONTEXT_WRITE_MULTIPLE_BLOCK;
            } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
                hsd->Context = (SDMMC_CONTEXT_WRITE_MULTIPLE_BLOCK | SDMMC_CONTEXT_DMA);
            }
            /* Set Write Multi Block Conut command */
            
            /* Write Multi Block command */
            errorstate = SDIO_CmdWriteMultiBlock(hsd->Instance, add);
        } else {
            if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
                hsd->Context = SDMMC_CONTEXT_WRITE_SINGLE_BLOCK;
            } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
                hsd->Context = (SDMMC_CONTEXT_WRITE_SINGLE_BLOCK | SDMMC_CONTEXT_DMA);
            }

            /* Write Single Block command */
            errorstate = SDIO_CmdWriteSingleBlock(hsd->Instance, add);
        }

        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            /* Clear all the static flags */
            SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SDMMC_STATE_READY;
            hsd->Context = SDMMC_CONTEXT_NONE;
            return HAL_SDMMC_ERR;
        }

        if(hsd->Sdio_cfg.DeviceMode == SDIO_POLLING_MODE) {
            /* Write block(s) in polling mode */
            do {
                while(SDIO_GET_IP_FLAG(hsd->Instance, SDIO_IP_TXFULL));
                SDIO_SendData(hsd->Instance, tempbuff[writeindex]);
                writeindex ++;
            } while (writeindex < (NumberOfBlocks * BLOCKSIZE / 4));
        } else if(hsd->Sdio_cfg.DeviceMode == SDIO_DMA_MODE) {
            SDIO_UdmaEnqueueChannel(hsd->Instance, tempbuff, BLOCKSIZE * NumberOfBlocks, SDIO_TX_CFG_EN_ENABLE|SDIO_TX_CFG_DATASIZE_WORD, TX_CHANNEL);

            SDIO_DmaStart(hsd->Instance, ENABLE);

            while(!SDIO_DmaGetIntStat(hsd->Instance, SDIO_TX_FTRANS_IRQ_STAT));
            SDIO_DmaInterruptClr(hsd->Instance, SDIO_TX_FTRANS_IRQ_CLR);
        }

        /* Check for error conditions */
        errorstate = SDIO_GetCmdResp1(hsd->Instance, NULL);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            return errorstate;
        }

        /* Send stop transmission command in case of multiblock write */
        if(NumberOfBlocks > 1U) {
            if(hsd->CardInfo.CardType != CARD_SECURED) {
                /* Send stop transmission command */
                errorstate = SDIO_CmdStopTransfer(hsd->Instance);
                if(errorstate != HAL_SDMMC_ERROR_NONE) {
                    /* Clear all the static flags */
                    SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                    hsd->ErrorCode |= errorstate;
                    hsd->State = HAL_SDMMC_STATE_READY;
                    hsd->Context = SDMMC_CONTEXT_NONE;
                    return HAL_SDMMC_ERR;
                }
                SDIO_ClockDivConfig(hsd->Instance, 0x2);
                do {
                    SDMMC_SendStatus(hsd);
                } while(!((hsd->CardStatus >> 9) & HAL_SDMMC_TRANSFER) && (timeout --));
            }
        }

        hsd->State = HAL_SDMMC_STATE_READY;
        /* Clear data channel and all the static flags */
        SDIO_ClearDataSetup(hsd->Instance);
        SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
        return HAL_SDMMC_OK;
    } else {
        hsd->ErrorCode |= HAL_SDMMC_ERROR_BUSY;
        return HAL_SDMMC_ERR;
    }
}

/**
  * @brief  Erases the specified memory area of the given SD card.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @param  hsd: Pointer to SD handle
  * @param  BlockStartAdd: Start Block address
  * @param  BlockEndAdd: End Block address
  * @retval HAL status
  */
HAL_SDMMCStatusTypeDef HAL_SDMMC_Erase(SDMMC_HandleTypeDef *hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd)
{
    uint32_t errorstate;
    uint32_t start_add = BlockStartAdd;
    uint32_t end_add = BlockEndAdd;

    if(hsd->State == HAL_SDMMC_STATE_READY) {

        hsd->ErrorCode = HAL_SDMMC_ERROR_NONE;

        if(end_add < start_add) {
            hsd->ErrorCode |= HAL_SDMMC_ERROR_PARAM;
            return HAL_SDMMC_ERR;
        }

        if(end_add > (hsd->CardInfo.LogBlockNbr)) {
            hsd->ErrorCode |= HAL_SDMMC_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_SDMMC_ERR;
        }

        hsd->State = HAL_SDMMC_STATE_BUSY;

        /* Check if the card command class supports erase command */
        if(((hsd->CardInfo.Class) & SDMMC_CCCC_ERASE) == 0U) {
            /* Clear all the static flags */
            SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                hsd->ErrorCode |= HAL_SDMMC_ERROR_REQUEST_NOT_APPLICABLE;
                hsd->State = HAL_SDMMC_STATE_READY;
                return HAL_SDMMC_ERR;
        }

        errorstate == SDMMC_SendStatus(hsd);
        if(errorstate == HAL_SDMMC_ERROR_NONE) {
            if((hsd->CardStatus & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED) {
                    /* Clear all the static flags */
                    SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                    hsd->ErrorCode |= HAL_SDMMC_ERROR_LOCK_UNLOCK_FAILED;
                    hsd->State = HAL_SDMMC_STATE_READY;
                    return HAL_SDMMC_ERR;
            }
        }

        /* Get start and end block for high capacity cards */
        if(hsd->CardInfo.CardType != CARD_SDHC_SDXC) {
            start_add *= 512U;
            end_add   *= 512U;
        }

        /* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
        if(hsd->CardInfo.CardType != CARD_SECURED) {
            /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
            errorstate = SDIO_CmdSDEraseStartAdd(hsd->Instance, start_add);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                /* Clear all the static flags */
                SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                hsd->ErrorCode |= errorstate;
                hsd->State = HAL_SDMMC_STATE_READY;
                return HAL_SDMMC_ERR;
            }

            /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
            errorstate = SDIO_CmdSDEraseEndAdd(hsd->Instance, end_add);
            if(errorstate != HAL_SDMMC_ERROR_NONE) {
                /* Clear all the static flags */
                SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
                hsd->ErrorCode |= errorstate;
                hsd->State = HAL_SDMMC_STATE_READY;
                return HAL_SDMMC_ERR;
            }
        }

        /* Send CMD38 ERASE */
        errorstate = SDIO_CmdErase(hsd->Instance, ERASE_TYPE_BLOCK);
        if(errorstate != HAL_SDMMC_ERROR_NONE) {
            /* Clear all the static flags */
            SDIO_ClearFlag(hsd->Instance, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SDMMC_STATE_READY;
            return HAL_SDMMC_ERR;
        }

        hsd->State = HAL_SDMMC_STATE_READY;

        return HAL_SDMMC_OK;
    } else {
        return HAL_SDMMC_BUSY;
    }
}

__attribute__ ((aligned (4))) uint8_t SDIO_DATA_BUFFER[512];

HAL_SDMMCStatusTypeDef SDMMC_ReadDisk(SDMMC_HandleTypeDef *hsd, uint8_t*buf, uint32_t sector, uint32_t cnt)
{
    HAL_SDMMCStatusTypeDef errorstate;
    uint32_t lsector = sector;
    
    if (ADDR32(buf) % 4 != 0) {
        for (int n = 0; n < cnt; n++) {
            errorstate = HAL_SDMMC_ReadBlocks(hsd, SDIO_DATA_BUFFER, lsector+512*n, 1);
            memcpy(buf, SDIO_DATA_BUFFER, 512);
            buf += 512;
        }
    } else {
        errorstate = HAL_SDMMC_ReadBlocks(hsd, buf, lsector, cnt);
    }

    return errorstate;
}

HAL_SDMMCStatusTypeDef SDMMC_WriteDisk(SDMMC_HandleTypeDef *hsd, uint8_t*buf, uint32_t sector, uint32_t cnt)
{
    HAL_SDMMCStatusTypeDef errorstate;
    uint32_t lsector = sector;
    
    if (ADDR32(buf) % 4 != 0) {
        for (int n = 0; n < cnt; n++) {
            memcpy(SDIO_DATA_BUFFER, buf, 512);
            errorstate = HAL_SDMMC_WriteBlocks(hsd, SDIO_DATA_BUFFER, lsector+512*n, 1);
            buf += 512;
        }
    } else {
        errorstate = HAL_SDMMC_WriteBlocks(hsd, buf, lsector, cnt);
    }

    return errorstate;
}
