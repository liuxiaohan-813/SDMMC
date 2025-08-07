/**
 * @file sdio.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-07-12
 *
 * @copyright Copyright (c) 2022
 *
 */

 /* Includes ------------------------------------------------------------------*/
/**
  * \file ns_sdio.c
  * \brief source file for the SDIO
  */

#define DMA_IRQ_OFFSET  0x1C00U
#define DMA_LLA_RX_LO_OFFSET  0x1E00U
#define DMA_LLA_RX_HI_OFFSET  0x1E04U
#define DMA_LLA_TX_LO_OFFSET  0x1F00U
#define DMA_LLA_TX_HI_OFFSET  0x1F04U

/* Includes ------------------------------------------------------------------*/
#include "ns.h"
#include "ns_sdio.h"

void SDIO_StructInit(SDIO_InitTypeDef *sdio_init)
{
    sdio_init->ClkDiv = 0;

    sdio_init->AutoStop = DISABLE;
    sdio_init->ClkStop = ENABLE;
    sdio_init->HighWidthMode = ENABLE;
    sdio_init->DataCrcChk = ENABLE;

    sdio_init->DataTimeOutCnt = 0xffffff;
    sdio_init->PowerUpCnt = 0x49;
    sdio_init->WaitRspCnt = 0x3f;
    sdio_init->WaitEotCnt = 0xfffff;
    sdio_init->TxDelayCnt = 0x7;
}

void SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef *sdio_init)
{
    uint32_t reg_tmp = 0;
    /* timeout and delay cnt cfg */
    SDIOx->DATA_TIMEOUT = (sdio_init->DataTimeOutCnt & SDIO_DATA_TIMEOUT_CNT_MASK);
    SDIOx->CMD_POWERUP = (sdio_init->PowerUpCnt & SDIO_CMD_POWERUP_CNT_MASK);
    SDIOx->CMD_WAIT_RSP = (sdio_init->WaitRspCnt & SDIO_CMD_WAIT_RSP_CNT_MASK);
    SDIOx->CMD_WAIT_EOT = (sdio_init->WaitEotCnt & SDIO_CMD_WAIT_EOT_CNT_MASK);
    SDIOx->DATA_TX_DELAY = (sdio_init->TxDelayCnt & SDIO_DATA_TX_DELAY_CNT_MASK);

    /* clk div config */
    SDIO_ClockDivConfig(SDIOx, sdio_init->ClkDiv);

    /* control reg config */
    reg_tmp = SDIOx->CR;
    reg_tmp &= ~(SDIO_CR_AUTO_CMD12 | SDIO_CR_CLK_STOP |  \
                SDIO_CR_HIGH_WIDTH_MODE | SDIO_CR_DATA_CRC_CHECK);
    if (ENABLE == sdio_init->AutoStop) {
        reg_tmp |= SDIO_CR_AUTO_CMD12_ENABLE;
    }
    if (ENABLE == sdio_init->ClkStop) {
        reg_tmp |= SDIO_CR_CLK_STOP_ENABLE;
    }
    if (ENABLE == sdio_init->HighWidthMode) {
        reg_tmp |= SDIO_CR_HIGH_WIDTH_MODE_ENABLE;
    }
    if (ENABLE == sdio_init->DataCrcChk) {
        reg_tmp |= SDIO_CR_DATA_CRC_CHECK_ENABLE;
    }

    SDIOx->CR = reg_tmp;
}

/**
  * \brief  Set SDIO Power state to ON.
  * \param  SDIOx: Pointer to SDIO register base
  * \retval None
  */
void SDIO_PowerState_ON(SDIO_TypeDef *SDIOx)
{
    /* Set power state to ON */
    SDIOx->CR |= SDIO_POWER_ON;
}

/**
  * \brief  Set SDIO Power state to OFF.
  * \param  SDIOx: Pointer to SDIO register base
  * \retval None
  */
void SDIO_PowerState_OFF(SDIO_TypeDef *SDIOx)
{
  /* Set power state to OFF */
  SDIOx->CR &= ((~SDIO_CR_POWER_ON_MODE_ENABLE_MASK) | SDIO_POWER_OFF);
}

/**
  * \brief  Get SDIO Power state.
  * \param  SDIOx: Pointer to SDIO register base
  * \retval Power status of the controller. The returned value can be one of the
  *         following values:
  *            - 0x01: Power OFF
  *            - 0x03: Power ON
  */
uint32_t SDIO_GetPowerState(SDIO_TypeDef *SDIOx)
{
  return ((SDIOx->CR & SDIO_CR_POWER_ON_MODE_ENABLE_MASK) >> SDIO_CR_POWER_ON_MODE_ENABLE_OFS);
}

/**
 * \brief  SDIO dma enable control.
 * \param  SDIOx select the SDIO peripheral.
 * \param  control dma enable control status
 *         This parameter can be ENABLE or DISABLE.
 */
void SDIO_DmaStart(SDIO_TypeDef *SDIOx, ControlStatus control)
{
    if (ENABLE == control) {
        SDIOx->CR |= SDIO_CR_DMA_ENABLE;
    } else {
        SDIOx->CR &= ~(SDIO_CR_DMA_ENABLE);
    }
}

/**
  * \brief  Checks whether the specified SDIO flag is set or not.
  * \param  SDIOx select the SDIO peripheral.
  * \param  status specifies the flag to check. 
  *   \arg  SDIO_STATUS_EOT  Transmission end flag
  *   \arg  SDIO_STATUS_ERR  Transmission error flag
  *   \arg  SDIO_STATUS_TXUDR_ERR  tx udr error flag
  *   \arg  SDIO_STATUS_TXOVF_ERR  tx ovf error flag
  *   \arg  SDIO_STATUS_RXUDR_ERR  rx udr error flag
  *   \arg  SDIO_STATUS_RXOVF_ERR  rx udr error flag
  *   \arg  SDIO_STATUS_BUSY  sdio busy flag
  *   \arg  SDIO_STATUS_IRQ_CHECK  sdio interrupt check
  *   \arg  SDIO_STATUS_BLOCK_DONE  sdio block done
  *   \arg  SDIO_STATUS_CMD_ERR_NO  No error
  *   \arg  SDIO_STATUS_CMD_ERR_TIMEOUT  cmd timeout error
  *   \arg  SDIO_STATUS_CMD_ERR_DIR  cmd DIR error
  *   \arg  SDIO_STATUS_CMD_ERR_BUSY cmd BUSY
  *   \arg  SDIO_STATUS_CMD_ERR_CRCERR  cmd CRC error
  *   \arg  SDIO_STATUS_DATA_ERR_NO  No error
  *   \arg  SDIO_STATUS_DATA_ERR_TIMEOUT data TIMEOUT error
  *   \arg  SDIO_STATUS_DATA_ERR_BUSY  data BUSY
  *   \arg  SDIO_STATUS_DATA_ERR_CRCTIMEOUT data CRC TIMEOUT
  *   \arg  SDIO_STATUS_DATA_ERR_CRCERR  data CRC ERR
  * \retval The state of SDIO_STATUS.
  */
uint32_t SDIO_GetFlagStatus(SDIO_TypeDef *SDIOx, uint32_t status)
{
    return SDIOx->STATUS & status;
}

/**
  * \brief  Get interrupt enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  interrupt sdio interrupt enable
  *   \arg  SDIO_IE_TX_IRQ_EN  Transmit watermark enable
  *   \arg  SDIO_IE_RX_IRQ_EN  Receive watermark enable
  *   \arg  SDIO_IE_EOT_IRQ_EN  end interrupt enable
  *   \arg  SDIO_IE_ERR_IRQ_EN  error interrupt enable
  *   \arg  SDIO_IE_TXUDR_ERR_EN  tx udr error interrupt enable
  *   \arg  SDIO_IE_TXOVF_ERR_EN  tx ovf error interrupt enable
  *   \arg  SDIO_IE_RXUDR_ERR_EN  rx udr error interrupt enable
  *   \arg  SDIO_IE_RXOVF_ERR_EN  rx ovf error interrupt enable
  *   \arg  SDIO_IE_IRQ_CHECK_EN  sdio check interrupt enable
  *   \arg  SDIO_IE_BLOCK_DONE_EN   sdio blcok done
  * \param  control interrupt enable control status
  *                 This parameter can be ENABLE or DISABLE.
  */
uint32_t SDIO_GetInterruptEn(SDIO_TypeDef *SDIOx, uint32_t interrupt)
{
    return (SDIOx->IE & interrupt);
}

/**
  * \brief  Clears the SDIO's pending flags.
  * \param  SDIOx select the SDIO peripheral.
  * \param  status specifies the flag to clear.
  *   \arg  SDIO_STATUS_EOT  Transmission end flag
  *   \arg  SDIO_STATUS_ERR  Transmission error flag
  *   \arg  SDIO_STATUS_TXUDR_ERR  tx udr error flag
  *   \arg  SDIO_STATUS_TXOVF_ERR  tx ovf error flag
  *   \arg  SDIO_STATUS_RXUDR_ERR  rx udr error flag
  *   \arg  SDIO_STATUS_RXOVF_ERR  rx udr error flag
  */
void SDIO_ClearFlag(SDIO_TypeDef *SDIOx, uint32_t status)
{
    SDIOx->STATUS = status ;
}

/**
  * \brief  Checks whether the specified SDIO interrupt has occurred or not.
  * \param  SDIOx select the SDIO peripheral.
  * \param  status specifies the SDIO interrupt source to check.
  *   \arg  SDIO_IP_TX_IRQ Transmit watermark enable
  *   \arg  SDIO_IP_RX_IRQ Receive watermark enable
  *   \arg  SDIO_IP_RX_EMPTY receive fifo empt
  *   \arg  SDIO_IP_TX_FULL send fifo full
  *   \arg  SDIO_IP_TX_EMPTY send fifo empt
  *   \arg  SDIO_IP_RX_FULL receive fifo full
  * \retval The new state of SDIO_IT (SET or RESET).
  */
uint32_t SDIO_GetIPStatus(SDIO_TypeDef *SDIOx, uint32_t status)
{
    return SDIOx->IP & status;
}

/**
  * \brief  Fills each SDIO_CmdInitStruct member with its default value.
  * \param  SDIO_CmdInitStruct pointer to an SDIO_CmdInitTypeDef 
  *         structure which will be initialized.
  */
void SDIO_CmdInitStructInit(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
{
    SDIO_CmdInitStruct->SDIO_CmdIndex = 0;
    SDIO_CmdInitStruct->SDIO_CmdType = 0;
    SDIO_CmdInitStruct->SDIO_Response = 0;
    SDIO_CmdInitStruct->SDIO_RspLen = 0;
    SDIO_CmdInitStruct->SDIO_BusyCheck = 0;
    SDIO_CmdInitStruct->SDIO_CrcEn = 0;
    SDIO_CmdInitStruct->SDIO_PowerUp = 0;
    SDIO_CmdInitStruct->SDIO_CrcCheck= 0;
    SDIO_CmdInitStruct->SDIO_IsStopCmd = 0;
    SDIO_CmdInitStruct->SDIO_Argument= 0;
}

/**
  * \brief  Fills each SDIO_DataInitStruct member with its default value.
  * \param  SDIO_DataSetupStruct pointer to an SDIO_DataInitTypeDef structure 
  *         which will be initialized.
  */
void SDIO_DataSetupStructInit(SDIO_DataSetupTypeDef *SDIO_DataSetupStruct)
{
    SDIO_DataSetupStruct->DataChannelEn = 0;
    SDIO_DataSetupStruct->TransferDir = 0;
    SDIO_DataSetupStruct->DataBusWidth = 0;
    SDIO_DataSetupStruct->TransferMode = 0;
    SDIO_DataSetupStruct->BlockNum = 0;
    SDIO_DataSetupStruct->BlockSize = 0;
}

/**
  * \brief  Initializes the SDIO data setup according to the specified 
  *         parameters in the SDIO_DataSetupStruct.
  * \param  SDIOx select the SDIO peripheral.
  * \param  SDIO_DataSetupStruct pointer to a SDIO_DataInitTypeDef structure 
  *         that contains the configuration information for the SDIO command.
  */
void SDIO_DataSetup(SDIO_TypeDef *SDIOx, SDIO_DataSetupTypeDef *SDIO_DataSetupStruct)
{
    uint32_t reg_tmp = 0;

    if (SDIO_CR_STREAM_MODE_ENABLE == SDIO_DataSetupStruct->TransferMode) {
        SDIOx->CR |= SDIO_CR_STREAM_MODE;
    }

    reg_tmp = SDIO_DataSetupStruct->DataChannelEn 
              | SDIO_DataSetupStruct->TransferDir
              | SDIO_DataSetupStruct->DataBusWidth
              | SDIO_DataSetupStruct->BlockNum
              | SDIO_DataSetupStruct->BlockSize;
    SDIOx->DATA_SETUP = reg_tmp;
}

/**
  * \brief  config sdio tx rx
  * \param  SDIOx select the SDIO peripheral.
  * \param  dir transmit direction
  * \param  buswidth transmit proto
  * \param  blockcnt transmit blk cnt
  * \param  blocksize transmit blk size
  * \param  mode transmit mode
  */
void SDIO_TxRxConfig(SDIO_TypeDef *SDIOx, uint32_t dir, uint32_t buswidth, uint32_t blockcnt, uint32_t blocksize, uint32_t mode)
{
    SDIO_DataSetupTypeDef sdio_datasetup;
    SDIO_DataSetupStructInit(&sdio_datasetup);
    /* channel enable default */
    sdio_datasetup.DataChannelEn = SDIO_DATA_SETUP_CHANNEL_ENABLE;
    sdio_datasetup.TransferDir = dir;
    sdio_datasetup.DataBusWidth = buswidth;
    sdio_datasetup.BlockNum = SDIO_DATA_SETUP_BLOCK_NUM(blockcnt - 1);
    sdio_datasetup.BlockSize = SDIO_DATA_SETUP_BLOCK_SIZE(blocksize - 1);
    SDIO_DataSetup(SDIOx, &sdio_datasetup);
}

/**
  * \brief  Clear the SDIO data setup .
  * \param  SDIOx select the SDIO peripheral.
  */
void SDIO_ClearDataSetup(SDIO_TypeDef *SDIOx)
{
    uint32_t reg_tmp = 0;
    SDIOx->DATA_SETUP = reg_tmp;
}

/**
  * \brief  This bit field represents the SDIO write data timeout counter.
  * \param  SDIOx select the SDIO peripheral.
  * \param  timeout 32-bit data word to write.
  */
void SDIO_SetDateTimeout(SDIO_TypeDef *SDIOx, uint32_t timeout)
{
    SDIOx->DATA_TIMEOUT = timeout;
}

/**
  * \brief  Setup sdio clock divider.
  * \param  SDIOx select the SDIO peripheral.
  * \param  clkdiv clock div number.
  */
void SDIO_ClockDivConfig( SDIO_TypeDef *SDIOx, uint32_t clkdiv)
{
	  SDIOx->CLK_DIV = (clkdiv & SDIO_CLK_DIV_NUM_MASK);
}

/**
  * \brief  Write one data word to Tx FIFO.
  * \param  SDIOx select the SDIO peripheral.
  * \param  data 32-bit data word to write.
  */
void SDIO_SendData(SDIO_TypeDef *SDIOx, uint32_t data)
{
    SDIOx->TX_DATA = data;
}

/**
  * \brief  Read one data word from Rx FIFO.
  * \param  SDIOx select the SDIO peripheral.
  * \retval Data sdio received
  */
uint32_t SDIO_ReadData(SDIO_TypeDef *SDIOx)
{
    return SDIOx->RX_DATA;
}

/**
  * \brief  Config Tx FIFO watermark.
  * \param  SDIOx select the SDIO peripheral.
  * \param  depth Tx watermark
  */
uint32_t SDIO_SetTxMark(SDIO_TypeDef *SDIOx, uint32_t depth)
{
    SDIOx->TX_MARK = depth;
}

/**
  * \brief  Config Rx FIFO watermark.
  * \param  SDIOx select the SDIO peripheral.
  * \param  depth Rx watermark
  */
uint32_t SDIO_SetRxMark(SDIO_TypeDef *SDIOx, uint32_t depth)
{
    SDIOx->RX_MARK = depth;
}

/**
  * \brief  Config power count.
  * \param  SDIOx select the SDIO peripheral.
  * \param  count count
  */
void SDIO_SetCmdPowerUpCnt(SDIO_TypeDef *SDIOx, uint32_t count)
{
    SDIOx->CMD_POWERUP = (count - 1);
}

/**
  * \brief  Config interrupt enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  interrupt sdio interrupt enable
  *   \arg  SDIO_IE_TX_IRQ_EN  Transmit watermark enable
  *   \arg  SDIO_IE_RX_IRQ_EN  Receive watermark enable
  *   \arg  SDIO_IE_EOT_IRQ_EN  end interrupt enable
  *   \arg  SDIO_IE_ERR_IRQ_EN  error interrupt enable
  *   \arg  SDIO_IE_TXUDR_ERR_EN  tx udr error interrupt enable
  *   \arg  SDIO_IE_TXOVF_ERR_EN  tx ovf error interrupt enable
  *   \arg  SDIO_IE_RXUDR_ERR_EN  rx udr error interrupt enable
  *   \arg  SDIO_IE_RXOVF_ERR_EN  rx ovf error interrupt enable
  *   \arg  SDIO_IE_IRQ_CHECK_EN  sdio check interrupt enable
  *   \arg  SDIO_IE_BLOCK_DONE_EN   sdio blcok done
  * \param  control interrupt enable control status
  *                 This parameter can be ENABLE or DISABLE.
  */
uint32_t SDIO_InterruptEn(SDIO_TypeDef *SDIOx, SDIO_IntTypedef interrupt, ControlStatus control)
{
    if (ENABLE == control) {
        SDIOx->IE |= interrupt;
    } else {
        SDIOx->IE &= ~(interrupt);
    }
}

/**
  * \brief  Config Dma interrupt enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  interrupt select the SDIO DMA interrupt.
  *   \arg  PA2M_FTRANS_IRQ_EN  interrupt enable for Channel0 full transfer
  *   \arg  PA2M_HTRANS_IRQ_EN  interrupt enable for Channel0 half transfer
  *   \arg  PA2M_RSP_ERR_IRQ_EN interrupt enable for Channel0 DMA access error
  * \param  control interrupt enable control status
  *                 This parameter can be ENABLE or DISABLE.
  */
uint32_t SDIO_DmaInterruptEn(SDIO_TypeDef *SDIOx, SDIO_DmaIntTypedef interrupt, ControlStatus control)
{
    UDMA_P2M_CHx_Irq_TypeDef *SDIO_DMA = (UDMA_P2M_CHx_Irq_TypeDef *)(ADDR32(SDIOx) + DMA_IRQ_OFFSET);

    if (ENABLE == control) {
        SDIO_DMA->CHX_IRQ_EN |= interrupt;
    } else {
        SDIO_DMA->CHX_IRQ_EN &= ~(interrupt);
    }
}

/**
  * \brief  Clear Dma interrupt enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  interrupt_clr select the SDIO DMA interrupt_clr flag.
  *   \arg  PA2M_FTRANS_IRQ_CLEAR_STAT   clear irq status flag for Channel0 full transfer 
  *   \arg  PA2M_HTRANS_IRQ_CLEAR_STAT   clear irq status flag for Channel0 half transfer 
  *   \arg  PA2M_RSP_ERR_IRQ_CLEAR_STAT  clear irq status flag for Channel0 DMA access error
  */
uint32_t SDIO_DmaInterruptClr(SDIO_TypeDef *SDIOx, SDIO_DmaIntClrTypedef interrupt_clr)
{
    UDMA_P2M_CHx_Irq_TypeDef *SDIO_DMA = (UDMA_P2M_CHx_Irq_TypeDef *)(ADDR32(SDIOx) + DMA_IRQ_OFFSET);
    SDIO_DMA->CHX_IRQ_CLR = interrupt_clr;
}

/**
  * \brief  Get sdio Dma interrupt status.
  * \param  SDIOx select the SDIO peripheral.
  * \param  status select the SDIO DMA interrupt status.
  *   \arg  PA2M_FTRANS_IRQ_STAT  interrupt irq status flag for Channel0 full transfer
  *   \arg  PA2M_HTRANS_IRQ_STAT  interrupt irq status flag for Channel0 full transfer
  *   \arg  PA2M_RSP_ERR_IRQ_STAT  interrupt irq status flag for Channel0 DMA access error
  * \retval SDIO DMA interrupt status
  */
uint32_t SDIO_DmaGetIntStat(SDIO_TypeDef *SDIOx, SDIO_DmaIntStatTypedef status)
{
    UDMA_P2M_CHx_Irq_TypeDef *SDIO_DMA = (UDMA_P2M_CHx_Irq_TypeDef *)(ADDR32(SDIOx) + DMA_IRQ_OFFSET);
    return (SDIO_DMA->CHX_IRQ_STAT & status);
}

/**
  * \brief  Config sdio clk_stop enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  control clk_stop enable control status
  *                 This parameter can be ENABLE or DISABLE.
  */
void SDIO_StopClkEn(SDIO_TypeDef *SDIOx, ControlStatus control)
{
    if (ENABLE == control) {
        SDIOx->CR |= SDIO_CR_CLK_STOP_ENABLE;
    } else {
        SDIOx->CR &= ~(SDIO_CR_CLK_STOP_ENABLE);
    }
}

/**
  * \brief  Config read wait enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  control clk_stop enable control status
  *                 This parameter can be ENABLE or DISABLE.
  */
void SDIO_ReadWaitEn(SDIO_TypeDef *SDIOx, ControlStatus control)
{
    if (ENABLE == control) {
        SDIOx->CR |= SDIO_CR_READ_WAIT_REQ;
    } else {
        SDIOx->CR &= ~(SDIO_CR_READ_WAIT_REQ);
    }
}

/**
  * \brief  Config sdio DDR mode enable.
  * \param  SDIOx select the SDIO peripheral.
  * \param  control DDR mode enable
  *                 This parameter can be ENABLE or DISABLE.
  */
void SDIO_CfgDdrMode(SDIO_TypeDef *SDIOx, ControlStatus control)
{
    if (ENABLE == control) {
        SDIOx->CR |= SDIO_CR_DDR_ENABLE;
    } else {
        SDIOx->CR &= ~(SDIO_CR_DDR_ENABLE);
    }
}

/**
  * \brief  Clears the status of SDIO channel in udma mode.
  * \param  SDIOx: where x can be 0, 1 to select the SDIO peripheral.
  * \param  channel: channel of the DMA Request.
  *                  This parameter can be: COMMAND_CHANNEL,TX_CHANNEL or RX_CHANNEL.
  */
void SDIO_UdmaClearChannel(SDIO_TypeDef* SDIOx, uint32_t channel)
{
    switch (channel) {
        case RX_CHANNEL:
            SDIOx->RX_CFG = SDIO_RX_CFG_CLR;
            break;
        case TX_CHANNEL:
            SDIOx->TX_CFG = SDIO_TX_CFG_CLR;
            break;
        default :
            break;
    }
}

/**
  * \brief  Initializes the UDMA enqueue channel.
  * \param  SDIOx: where x can be to select the SDIO peripheral.
  * \param  addr: channel UDMA transfer address of associated buffer.
  * \param  size: channel UDMA transfer size of buffer.
  * \param  config:  channel UDMA transfer configuration.
  * \param  channel: dst_base_addr of the UDMA Request sources.
  *  This parameter can be one of the following values:
  *     \arg RX_CHANNEL
  *     \arg TX_CHANNEL
  */

void SDIO_UdmaEnqueueChannel(SDIO_TypeDef* SDIOx, uint32_t* addr, uint32_t size, uint32_t config, uint32_t channel)
{
    
    switch (channel) {
        /* push to rx dst_base_addr */
        case (RX_CHANNEL):
            SDIOx->RX_SADDR = CAL_ADDR_LO(addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
            SDIOx->RX_SADDR_HI = CAL_ADDR_HI(addr);
#endif
            SDIOx->RX_SIZE = size;
            SDIOx->RX_CFG = config;
            break;
        /* push to tx dst_base_addr */
        case (TX_CHANNEL):
            SDIOx->TX_SADDR = CAL_ADDR_LO(addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
            SDIOx->TX_SADDR_HI = CAL_ADDR_HI(addr);
#endif
            SDIOx->TX_SIZE = size;
            SDIOx->TX_CFG = config;
            break;
        default:
            break;
    }
}

/**
  * \brief  Initializes the UDMA link list mode enqueue channel.
  * \param  SDIOx: where x can be to select the SDIO peripheral.
  * \param  dma_cfg_list: channel UDMA lla config list.
  */
void SDIO_UdmaListModeEnqueueChannel(SDIO_TypeDef* SDIOx, SDIO_DmaConfigListTypeDef dma_cfg_list)
{
    SDIO_DmaLinkListNodeTypeDef *current = dma_cfg_list.link_list;
    if(NULL != current) {
        switch (dma_cfg_list.channel) {
            case (RX_CHANNEL):
                REG32(ADDR32(SDIOx) + DMA_LLA_RX_LO_OFFSET) = CAL_ADDR_LO(current->config_addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
                REG32(ADDR32(SDIOx) + DMA_LLA_RX_HI_OFFSET) = CAL_ADDR_HI(current->config_addr);
#endif
                break;
            case (TX_CHANNEL):
                REG32(ADDR32(SDIOx) + DMA_LLA_TX_LO_OFFSET) = CAL_ADDR_LO(current->config_addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
                REG32(ADDR32(SDIOx) + DMA_LLA_TX_HI_OFFSET) = CAL_ADDR_HI(current->config_addr);
#endif
                break;
            default:
                break;
        }

        do {
            *(current->config_addr) = CAL_ADDR_LO(current->addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
            *(current->config_addr+5) = CAL_ADDR_HI(current->addr);
#endif
            *(current->config_addr+1) = current->block_size;
            *(current->config_addr+2) = current->data_size;
            if(NULL != current->next) {
                *(current->config_addr+3) = CAL_ADDR_LO(current->next->config_addr);
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
                *(current->config_addr+4) = CAL_ADDR_HI(current->next->config_addr);   
#endif
            } else {
                *(current->config_addr+3) = 0x00U;
#if defined(SOC_ADDRESS_HIGH_VALID) && SOC_ADDRESS_HIGH_VALID == 1
                *(current->config_addr+4) = 0x00U;   
#endif
            }

            current = current->next;
        } while (current);
    }

    SDIO_UdmaEnqueueChannel(SDIOx, dma_cfg_list.addr, dma_cfg_list.block_size, dma_cfg_list.dma_config, dma_cfg_list.channel);
}

/**
  * \brief  Initializes the SDIO Command according to the specified parameters
  *         in the SDIO_CmdInitStruct and send the command.
  * \param  SDIOx select the SDIO peripheral.
  * \param  SDIO_CmdInitStruct  pointer to a SDIO_CmdInitTypeDef structure that
  *         contains the configuration information for the SDIO command.
  */
void SDIO_SendCommand(SDIO_TypeDef *SDIOx, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
{
    uint32_t tmpreg = 0;

    /* close data channel enable */
    if(SDIO_CMD_TYPE_COMMON == SDIO_CmdInitStruct->SDIO_CmdType) {
        SDIOx->DATA_SETUP &= ~(SDIO_DATA_SETUP_CHANNEL_ENABLE);
    }

    /* SDIO ARG Configuration */
    SDIOx->CMD_ARG = SDIO_CmdInitStruct->SDIO_Argument;

    /* SDIO CMD Configuration */
    tmpreg = SDIOx->CMD_OP;

    tmpreg &= ~(SDIO_CMD_OP_RSP | SDIO_CMD_OP_RSP_LEN | SDIO_CMD_OP_CRC | \
                SDIO_CMD_OP_BUSY | SDIO_CMD_OP_POWER_UP | SDIO_CMD_OP_CRC_CHECK | \
                SDIO_CMD_OP_STOP_CMD | SDIO_CMD_OP_CMD_ENABLE | SDIO_CMD_OP_INDEX_MASK );

    tmpreg = SDIO_CmdInitStruct->SDIO_Response | SDIO_CmdInitStruct->SDIO_RspLen | SDIO_CmdInitStruct->SDIO_CrcEn | \
              SDIO_CmdInitStruct->SDIO_BusyCheck | SDIO_CmdInitStruct->SDIO_PowerUp | SDIO_CmdInitStruct->SDIO_CrcCheck | \
              SDIO_CmdInitStruct->SDIO_IsStopCmd | SDIO_CmdInitStruct->SDIO_CmdIndex;

    /* cmd seng enable default when send cmd */
    tmpreg |= SDIO_CMD_OP_CMD_ENABLE_ENABLE;

    /* Write to SDIO CMD_OP */
    SDIOx->CMD_OP = tmpreg;

    /* Start transmission CMD */
    SDIOx->START = SDIO_START_ENABLE;
}

/**
  * @brief  Return the response received from the card for the last command
  * @param  SDIOx select the SDIO peripheral.
  * @param  Response: Specifies the SDIO response register.
  *          This parameter can be one of the following values:
  *            @arg SDIO_RESP1: Response Register 1
  *            @arg SDIO_RESP2: Response Register 2
  *            @arg SDIO_RESP3: Response Register 3
  *            @arg SDIO_RESP4: Response Register 4
  * @retval The Corresponding response register value
  */
uint32_t SDIO_GetResponse(SDIO_TypeDef *SDIOx, uint32_t Response)
{
    /* Get the response */
    return (*(ADDR32P(&(SDIOx->RSP0)) + Response));
}

/* ------------------------------------------- CMD functions ------------------------------------------- */
/**
  * @brief  Send the Go Idle State command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdGoIdleState(SDIO_TypeDef *SDIOx)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD0 GO_IDLE_STATE */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_GO_IDLE_STATE;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_NO;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_PowerUp = SDIO_CMD_OP_POWER_UP_ENABLE;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdError(SDIOx);
    return errorstate;
}

/**
  * @brief  Sends host capacity support information and activates the card's
  *         initialization process. Send SDIO_CMD_SEND_OP_COND command
  * @param  SDIOx: Pointer to SDIO register base
  * @parame Argument: Argument used for the command
  * @retval HAL status
  */
uint32_t SDIO_CmdOpCondition(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD1 SEND_OP_COND */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SEND_OP_COND;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp3(SDIOx, Response);

    return errorstate;
}

/**
  * @brief  Send the Send CID command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdSendCID(SDIO_TypeDef *SDIOx, uint32_t *CID)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;
    uint32_t reg_status;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    /* Send CMD2 ALL_SEND_CID */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_ALL_SEND_CID;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_136;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);
    
    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp2(SDIOx);

    CID[0] = SDIO_GetResponse(SDIOx, SDIO_RESP1);
    CID[1] = SDIO_GetResponse(SDIOx, SDIO_RESP2);
    CID[2] = SDIO_GetResponse(SDIOx, SDIO_RESP3);
    CID[3] = SDIO_GetResponse(SDIOx, SDIO_RESP4);

    return errorstate;
}

/**
  * @brief  Send the Send RCA command to get sdcard rca.
  * @param  SDIOx: Pointer to SDMMC register base
  * @param  pRCA: Card RCA
  * @retval HAL status
  */
uint32_t SDIO_CmdGetRelAdd(SDIO_TypeDef *SDIOx, uint16_t *pRCA)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD3 SET_REL_ADDR */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SET_REL_ADDR;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp6(SDIOx, SDMMC_CMD_SET_REL_ADDR, pRCA);

    return errorstate;
}

/**
  * @brief  Send the Send RCA command to set mmc rca.
  * @param  SDIOx: Pointer to SDMMC register base
  * @param  pRCA: Card RCA
  * @retval HAL status
  */
uint32_t SDIO_CmdSetRelAdd(SDIO_TypeDef *SDIOx, uint32_t RCA)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD3 SET_REL_ADDR */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = RCA;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SET_REL_ADDR;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Select Deselect command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @param  addr: Address of the card to be selected
  * @retval HAL status
  */
uint32_t SDIO_CmdSelDesel(SDIO_TypeDef *SDIOx, uint64_t Addr)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD7 SEL_DESEL_CARD */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SEL_DESEL_CARD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_BusyCheck = SDIO_CMD_OP_BUSY_ENABLE;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Bus Width command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @param  BusWidth: BusWidth
  * @retval HAL status
  */
uint32_t SDIO_CmdBusWidth(SDIO_TypeDef *SDIOx, uint32_t BusWidth)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD6 SD_SET_BUSWIDTH */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = BusWidth;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_APP_SD_SET_BUSWIDTH;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_BusyCheck = SDIO_CMD_OP_BUSY_ENABLE;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Send CSD command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDIO_CmdSendCSD(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *CSD)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD9 SEND_CSD */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SEND_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_136;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp2(SDIOx);
    
    /* Get Card Specific Data */
    CSD[3] = SDIO_GetResponse(SDIOx, SDIO_RESP1);
    CSD[2] = SDIO_GetResponse(SDIOx, SDIO_RESP2);
    CSD[1] = SDIO_GetResponse(SDIOx, SDIO_RESP3);
    CSD[0] = SDIO_GetResponse(SDIOx, SDIO_RESP4);

    return errorstate;
}

/**
  * @brief  Send the Operating Condition command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdOperCond(SDIO_TypeDef *SDIOx)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD8 to verify SD card interface operating condition */
    /* Argument:
    - [31:12]: Reserved (shall be set to '0')
    - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    - [7:0]: Check Pattern (recommended 0xAA) */
    /* CMD Response: R7 */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = SDIO_CHECK_PATTERN;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_HS_SEND_EXT_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp7(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Send EXT_CSD command and check the response.
  * @param  SDIOx: Pointer to SDMMC register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDIO_CmdSendEXTCSD(SDIO_TypeDef *SDIOx, uint32_t Argument)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD8 SEND_EXT_CSD(get ext_csd) */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_HS_SEND_EXT_CSD;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return SDIO_ERROR_NONE;
}

/**
  * @brief  Send the Stop Transfer command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdStopTransfer(SDIO_TypeDef *SDIOx)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD12 STOP_TRANSMISSION */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_STOP_TRANSMISSION;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_CmdInitStructure.SDIO_BusyCheck = SDIO_CMD_OP_BUSY_ENABLE;
    SDIO_CmdInitStructure.SDIO_IsStopCmd = SDIO_CMD_OP_STOP_CMD_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    /* Ignore Address Out Of Range Error, Not relevant at end of memory */
    if (errorstate == SDIO_ERROR_ADDR_OUT_OF_RANGE) {
        errorstate = SDIO_ERROR_NONE;
    }

    return errorstate;
}

/**
  * @brief  Send the Status command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDIO_CmdSendStatus(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD13 SEND_STATUS */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SEND_STATUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, Response);

    return errorstate;
}

/**
  * @brief  Send the Status register command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdStatusRegister(SDIO_TypeDef *SDIOx, uint32_t Argument)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD13 SD_APP_STATUS */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SD_APP_STATUS;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return errorstate;
}

/**
  * @brief  Send the Data Block Lenght command and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdBlockLength(SDIO_TypeDef *SDIOx, uint32_t BlockSize)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD16 SET_BLOCKLEN */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Read Single Block command and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdReadSingleBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD16 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = ReadAdd;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return SDIO_ERROR_NONE;
}

/**
  * @brief  Send the Read Multi Block command and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdReadMultiBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD18 READ_MULT_BLOCK */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = ReadAdd;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_READ_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return SDIO_ERROR_NONE;
}

/**
  * @brief  Set Multi-Block Cmd block len for SDCard and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdMultiBlockCount(SDIO_TypeDef *SDIOx, uint32_t BlockCount)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD23 SET_BLOCK_COUNT */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)BlockCount;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SET_BLOCK_COUNT;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Write Single Block command and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdWriteSingleBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)WriteAdd;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return SDIO_ERROR_NONE;
}

/**
  * @brief  Send the Write Multi Block command and check the response
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdWriteMultiBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD25 WRITE_MULT_BLOCK */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)WriteAdd;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_WRITE_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return SDIO_ERROR_NONE;
}

/**
  * @brief  Send the Start Address Erase command for SD and check the response
  * @param  {ip_inst_name_upper}x: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDIO_CmdSDEraseStartAdd(SDIO_TypeDef *SDIOx, uint32_t StartAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD32 ERASE_GRP_START */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)StartAdd;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SD_ERASE_GRP_START;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the End Address Erase command for SD and check the response
  * @param  SDIOx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDIO_CmdSDEraseEndAdd(SDIO_TypeDef *SDIOx, uint32_t EndAdd)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD33 ERASE_GRP_END */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)EndAdd;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SD_ERASE_GRP_END;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the Erase command and check the response
  * @param  SDIOx Pointer to SDMMC register base
  * @param  EraseType Type of erase to be performed
  * @retval HAL status
  */
uint32_t SDIO_CmdErase(SDIO_TypeDef *SDIOx, uint32_t EraseType)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD38 ERASE */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)EraseType;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_ERASE;
    SDIO_CmdInitStructure.SDIO_BusyCheck = SDIO_CMD_OP_BUSY_ENABLE;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/**
  * @brief  Send the command asking the accessed card to send its operating
  *         condition register (OCR)
  * @param  SDIOx: Pointer to SDIO register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDIO_CmdAppOperCommand(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD41 SD_APP_OP_COND */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = Argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SD_APP_OP_COND;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    errorstate = SDIO_GetCmdResp3(SDIOx, Response);

    return errorstate;
}

/**
  * @brief  Send the Send SCR command and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDIO_CmdSendSCR(SDIO_TypeDef *SDIOx)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD51 SD_APP_SEND_SCR */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_SD_APP_SEND_SCR;
    SDIO_CmdInitStructure.SDIO_CmdType = SDIO_CMD_TYPE_DATA;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;

    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    return errorstate;
}

/**
  * @brief  Send the Application command to verify that that the next command
  *         is an application specific com-mand rather than a standard command
  *         and check the response.
  * @param  SDIOx: Pointer to SDIO register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDIO_CmdAppCommand(SDIO_TypeDef *SDIOx, uint32_t Argument)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    uint32_t errorstate;

    /* Send CMD55 APP_CMD */
    SDIO_CmdInitStructInit(&SDIO_CmdInitStructure);
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)Argument;;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDMMC_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_CMD_OP_RSP_YES;
    SDIO_CmdInitStructure.SDIO_RspLen = SDIO_CMD_OP_RSP_LEN_48;
    SDIO_CmdInitStructure.SDIO_CrcEn = SDIO_CMD_OP_CRC_ENABLE;
    SDIO_SendCommand(SDIOx, &SDIO_CmdInitStructure);

    /* Check for error conditions */
    /* If there is a HAL_ERROR, it is a MMC card, else
    it is a SD card: SD card 2.0 (voltage range mismatch)
        or SD card 1.x */
    errorstate = SDIO_GetCmdResp1(SDIOx, NULL);

    return errorstate;
}

/* ------------------------------------------- static functions ------------------------------------------- */
 uint32_t SDIO_GetCmdError(SDIO_TypeDef *SDIOx)
{
    uint32_t timeout = SDIO_CMD0TIMEOUT;
    do{
        if(timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
    } while (!SDIO_GetFlagStatus(SDIOx, SDIO_STATUS_EOT));

    SDIO_ClearFlag(SDIOx, SDIO_STATIC_FLAGS);
    return SDIO_ERROR_NONE;
}

 uint32_t SDIO_GetCmdResp1(SDIO_TypeDef *SDIOx, uint32_t *pResponse)
{
    uint32_t response_r1;
    uint32_t reg_status;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do {
        if (timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
        reg_status = SDIO_GET_STATUS(SDIOx);
    } while (!(reg_status & (SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR | SDIO_STATUS_DATA_ERR)));

    if((timeout == 0) || (reg_status & SDIO_STATUS_CMD_ERR_TIMEOUT)) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_TIMEOUT);
        return SDIO_ERROR_CMD_RSP_TIMEOUT;
    } else if (reg_status & SDIO_STATUS_CMD_ERR_CRCERR) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_CRCERR);
        return SDIO_ERROR_CMD_CRC_FAIL;
    } else {
        /* Nothing to do */
    }

    /* We have received response, retrieve it for analysis  */
    response_r1 = SDIO_GetResponse(SDIOx, SDIO_RESP1);
    if (pResponse) {
        *pResponse = response_r1;
    }

    SDIO_ClearFlag(SDIOx, SDIO_STATIC_FLAGS);

    if((response_r1 & SDIO_OCR_ERRORBITS) == 0U)
    {
        return SDIO_ERROR_NONE;
    }
    else if((response_r1 & SDIO_OCR_ADDR_OUT_OF_RANGE) == SDIO_OCR_ADDR_OUT_OF_RANGE)
    {
        return SDIO_ERROR_ADDR_OUT_OF_RANGE;
    }
    else if((response_r1 & SDIO_OCR_ADDR_MISALIGNED) == SDIO_OCR_ADDR_MISALIGNED)
    {
        return SDIO_ERROR_ADDR_MISALIGNED;
    }
    else if((response_r1 & SDIO_OCR_BLOCK_LEN_ERR) == SDIO_OCR_BLOCK_LEN_ERR)
    {
        return SDIO_ERROR_BLOCK_LEN_ERR;
    }
    else if((response_r1 & SDIO_OCR_ERASE_SEQ_ERR) == SDIO_OCR_ERASE_SEQ_ERR)
    {
        return SDIO_ERROR_ERASE_SEQ_ERR;
    }
    else if((response_r1 & SDIO_OCR_BAD_ERASE_PARAM) == SDIO_OCR_BAD_ERASE_PARAM)
    {
        return SDIO_ERROR_BAD_ERASE_PARAM;
    }
    else if((response_r1 & SDIO_OCR_WRITE_PROT_VIOLATION) == SDIO_OCR_WRITE_PROT_VIOLATION)
    {
        return SDIO_ERROR_WRITE_PROT_VIOLATION;
    }
    else if((response_r1 & SDIO_OCR_LOCK_UNLOCK_FAILED) == SDIO_OCR_LOCK_UNLOCK_FAILED)
    {
        return SDIO_ERROR_LOCK_UNLOCK_FAILED;
    }
    else if((response_r1 & SDIO_OCR_COM_CRC_FAILED) == SDIO_OCR_COM_CRC_FAILED)
    {
        return SDIO_ERROR_COM_CRC_FAILED;
    }
    else if((response_r1 & SDIO_OCR_ILLEGAL_CMD) == SDIO_OCR_ILLEGAL_CMD)
    {
        return SDIO_ERROR_ILLEGAL_CMD;
    }
    else if((response_r1 & SDIO_OCR_CARD_ECC_FAILED) == SDIO_OCR_CARD_ECC_FAILED)
    {
        return SDIO_ERROR_CARD_ECC_FAILED;
    }
    else if((response_r1 & SDIO_OCR_CC_ERROR) == SDIO_OCR_CC_ERROR)
    {
        return SDIO_ERROR_CC_ERR;
    }
    else if((response_r1 & SDIO_OCR_STREAM_READ_UNDERRUN) == SDIO_OCR_STREAM_READ_UNDERRUN)
    {
        return SDIO_ERROR_STREAM_READ_UNDERRUN;
    }
    else if((response_r1 & SDIO_OCR_STREAM_WRITE_OVERRUN) == SDIO_OCR_STREAM_WRITE_OVERRUN)
    {
        return SDIO_ERROR_STREAM_WRITE_OVERRUN;
    }
    else if((response_r1 & SDIO_OCR_CID_CSD_OVERWRITE) == SDIO_OCR_CID_CSD_OVERWRITE)
    {
        return SDIO_ERROR_CID_CSD_OVERWRITE;
    }
    else if((response_r1 & SDIO_OCR_WP_ERASE_SKIP) == SDIO_OCR_WP_ERASE_SKIP)
    {
        return SDIO_ERROR_WP_ERASE_SKIP;
    }
    else if((response_r1 & SDIO_OCR_CARD_ECC_DISABLED) == SDIO_OCR_CARD_ECC_DISABLED)
    {
        return SDIO_ERROR_CARD_ECC_DISABLED;
    }
    else if((response_r1 & SDIO_OCR_ERASE_RESET) == SDIO_OCR_ERASE_RESET)
    {
        return SDIO_ERROR_ERASE_RESET;
    }
    else if((response_r1 & SDIO_OCR_AKE_SEQ_ERROR) == SDIO_OCR_AKE_SEQ_ERROR)
    {
        return SDIO_ERROR_AKE_SEQ_ERR;
    }
    else
    {
        return SDIO_ERROR_GENERAL_UNKNOWN_ERR;
    }
}

 uint32_t SDIO_GetCmdResp2(SDIO_TypeDef *SDIOx)
{
    uint32_t reg_status;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do {
        if (timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
        reg_status = SDIO_GET_STATUS(SDIOx);
    } while (!(reg_status & (SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR | SDIO_STATUS_DATA_ERR)));

    if((timeout == 0) || (reg_status & SDIO_STATUS_CMD_ERR_TIMEOUT))
    {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_TIMEOUT);
        return SDIO_ERROR_CMD_RSP_TIMEOUT;
    } else if (reg_status & SDIO_STATUS_CMD_ERR_CRCERR) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_CRCERR);
        return SDIO_ERROR_CMD_CRC_FAIL;
    } else {
        SDIO_ClearFlag(SDIOx, SDIO_STATIC_CMD_FLAGS);
    }

    return SDIO_ERROR_NONE;
}

 uint32_t SDIO_GetCmdResp3(SDIO_TypeDef *SDIOx, uint32_t *pResponse)
{
    uint32_t reg_status;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do {
        if (timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
        reg_status = SDIO_GET_STATUS(SDIOx);
    } while (!(reg_status & (SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR | SDIO_STATUS_DATA_ERR)));

    /* Get command response */
    if(NULL != pResponse) {
        *pResponse = SDIO_GetResponse(SDIOx, SDIO_RESP1);
    }

    if((timeout == 0) || (reg_status & SDIO_STATUS_CMD_ERR_TIMEOUT))
    {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_TIMEOUT);
        return SDIO_ERROR_CMD_RSP_TIMEOUT;
    } else {
        SDIO_ClearFlag(SDIOx, SDIO_STATIC_CMD_FLAGS);
    }

    return SDIO_ERROR_NONE;
}

 uint32_t SDIO_GetCmdResp7(SDIO_TypeDef *SDIOx)
{
    uint32_t reg_status;
    /* 8 is the number of required instructions cycles for the below loop statement.The SDIO_CMDTIMEOUT is expressed in ms */
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do {
        if (timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
        reg_status = SDIO_GET_STATUS(SDIOx);
    } while (!(reg_status & (SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR | SDIO_STATUS_DATA_ERR)));

    if((timeout == 0) || (reg_status & SDIO_STATUS_CMD_ERR_TIMEOUT))
    {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_TIMEOUT);
        return SDIO_ERROR_CMD_RSP_TIMEOUT;
    } else if (reg_status & SDIO_STATUS_CMD_ERR_CRCERR) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_CRCERR);
        return SDIO_ERROR_CMD_CRC_FAIL;
    } else {
        /* Nothing to do */
    }

    if(reg_status & SDIO_STATUS_EOT) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_EOT);
    }

    return SDIO_ERROR_NONE;
}

uint32_t SDIO_GetCmdResp6(SDIO_TypeDef *SDIOx, uint16_t SD_CMD, uint16_t *pRCA)
{
    uint32_t response_r1;
    uint32_t reg_status;
    uint32_t timeout = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do {
        if (timeout-- == 0U) {
            return SDIO_ERROR_TIMEOUT;
        }
        reg_status = SDIO_GET_STATUS(SDIOx);
    } while (!(reg_status & (SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR | SDIO_STATUS_DATA_ERR)));

    if((timeout == 0) || (reg_status & SDIO_STATUS_CMD_ERR_TIMEOUT))
    {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_TIMEOUT);
        return SDIO_ERROR_CMD_RSP_TIMEOUT;
    } else if (reg_status & SDIO_STATUS_CMD_ERR_CRCERR) {
        SDIO_ClearFlag(SDIOx, SDIO_STATUS_CMD_ERR_CRCERR);
        return SDIO_ERROR_CMD_CRC_FAIL;
    } else {
        /* Nothing to do */
    }

    /* We have received response, retrieve it.  */
    response_r1 = SDIO_GetResponse(SDIOx, SDIO_RESP1);
    SDIO_ClearFlag(SDIOx, SDIO_STATIC_CMD_FLAGS);

    if((response_r1 & (SDIO_R6_GENERAL_UNKNOWN_ERROR | SDIO_R6_ILLEGAL_CMD | SDIO_R6_COM_CRC_FAILED)) == 0U)
    {
        *pRCA = (uint16_t) (response_r1 >> 16);

        return SDIO_ERROR_NONE;
    }
    else if((response_r1 & SDIO_R6_ILLEGAL_CMD) == SDIO_R6_ILLEGAL_CMD)
    {
        return SDIO_ERROR_ILLEGAL_CMD;
    }
    else if((response_r1 & SDIO_R6_COM_CRC_FAILED) == SDIO_R6_COM_CRC_FAILED)
    {
        return SDIO_ERROR_COM_CRC_FAILED;
    }
    else
    {
        return SDIO_ERROR_GENERAL_UNKNOWN_ERR;
    }
}