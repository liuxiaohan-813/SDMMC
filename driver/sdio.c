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
 * \brief source file for the SDIO
 */

 /* Includes ------------------------------------------------------------------*/
#include "sdio.h"

/**
 * @brief  SDIO dma config SDIO_DmaCfgStruct member with its default value.
 * @param  SDIO: select the SDIO peripheral.
 * @param  SDIO_DmaCfgStruct: pointer to an SDIO_DmaCfgTypeDef structure
 *         which will be initialized.
 * @retval None
 */

void SDIO_DmaCfgStructInit(SDIO_TypeDef *SDIO,
                           SDIO_DmaCfgTypeDef *SDIO_DmaCfgStruct)
 {
     SDIO_DmaCfgStruct->Dma_en = 0;
     SDIO_DmaCfgStruct->Rx_addr = 0;
     SDIO_DmaCfgStruct->Rx_size = 0;
     SDIO_DmaCfgStruct->Rx_en = 0;
     SDIO_DmaCfgStruct->Rx_datasize = 2;

     SDIO_DmaCfgStruct->Tx_addr = 0;
     SDIO_DmaCfgStruct->Tx_size = 0;
     SDIO_DmaCfgStruct->Tx_en = 0;
     SDIO_DmaCfgStruct->Tx_datasize = 2;
 }

 /**
  * @brief  Initializes the SDIO dma config according to the specified
  *         parameters in the SDIO_DmaCfgStruct.
  * @param  SDIO: select the SDIO peripheral.
  * @param  SDIO_DmaCfgStruct : pointer to a SDIO_DmaCfgTypeDef structure
  *         that contains the configuration information for the SDIO dma.
  * @retval None
  */
 void SDIO_DMA_Config(SDIO_TypeDef *SDIO, SDIO_DmaCfgTypeDef *SDIO_DmaCfgStruct)
 {
     uint32_t rxdma_cfg = 0;
     uint32_t txdma_cfg = 0;
     if ((SDIO_DmaCfgStruct->Rx_en) & SDIO_RX_CFG_EN_ENABLE)
     {
         SDIO->RX_SADDR = ADDR32(SDIO_DmaCfgStruct->Rx_addr);
         SDIO->RX_SIZE = SDIO_DmaCfgStruct->Rx_size;

         rxdma_cfg = SDIO->RX_CFG;
         rxdma_cfg |=
             (SDIO_DmaCfgStruct->Rx_en | SDIO_DmaCfgStruct->Rx_datasize);
         SDIO->RX_CFG = rxdma_cfg;
     }
     else if ((SDIO_DmaCfgStruct->Tx_en) & SDIO_TX_CFG_EN_ENABLE)
     {
         SDIO->TX_SADDR = ADDR32(SDIO_DmaCfgStruct->Tx_addr);
         SDIO->TX_SIZE = SDIO_DmaCfgStruct->Tx_size;

         txdma_cfg = SDIO->TX_CFG;
         txdma_cfg |=
             (SDIO_DmaCfgStruct->Tx_en | SDIO_DmaCfgStruct->Tx_datasize);
         SDIO->TX_CFG = txdma_cfg;
     }

     SDIO->CR |= SDIO_DmaCfgStruct->Dma_en;
 }

 /**
  * @brief  Checks whether the specified SDIO flag is set or not.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  SDIO_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg SDIO_CMDBusyTimeout: Command response timeout
  *            @arg SDIO_DATATIMEOUT: Data timeout
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check
  * passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is
  * zero)
  *            @arg SDIO_DATA_RUNNING:  Data transmit/receive in progress
  *            @arg  SDIO_STATUS_EOT  :  Transmission end flag
  *            @arg  SDIO_STATUS_ERR  :  Transmission error flag
  *            @arg SDIO_CMDWrongDirection : Response Wrong Direction
  * @retval The new state of SDIO_FLAG (SET or RESET).
  */

 uint32_t SDIO_GetFlagStatus(SDIO_TypeDef *SDIO, uint32_t status)
 {
     return SDIO0->STATUS & status;
 }
 /**
  * @brief  Clears the SDIO's pending flags.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  SDIO_FLAG: specifies the flag to clear.
  *          This parameter can be one or a combination of the following values:
  *            @arg  SDIO_STATUS_EOT: Transmission end flag
  *            @arg  SDIO_STATUS_ERR: Transmission error flag
  * @retval None
  */
 void SDIO_ClearFlag(SDIO_TypeDef *SDIO, uint32_t status)
 {
     SDIO0->STATUS |= status;
 }
 /**
  * @brief  Checks whether the specified SDIO interrupt has occurred or not.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  SDIO_IT: specifies the SDIO interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SDIO_IP_RXEMPTY: receive fifo empt
  *            @arg SDIO_IP_TXFULL: send fifo full
  * @retval The new state of SDIO_IT (SET or RESET).
  */
 uint32_t SDIO_GetIPStatus(SDIO_TypeDef *SDIO, uint32_t status)
 {
     return SDIO->IP & status;
 }

 /**
  * @brief  Fills each SDIO_CmdInitStruct member with its default value.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  SDIO_CmdInitStruct: pointer to an SDIO_CmdInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
 void SDIO_CmdInitStructInit(SDIO_TypeDef *SDIO,
                             SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
 {
     SDIO_CmdInitStruct->SDIO_CmdIndex = 0;
     SDIO_CmdInitStruct->SDIO_No_Rsp = 0;
     SDIO_CmdInitStruct->SDIO_No_Rsp_Len = 0;
     SDIO_CmdInitStruct->SDIO_Busy = 0;
     SDIO_CmdInitStruct->SDIO_CrcEn = 0;
     SDIO_CmdInitStruct->SDIO_Power_Up = 0;
     SDIO_CmdInitStruct->SDIO_Argument = 0;
 }

 /**
  * @brief  Fills each SDIO_DataInitStruct member with its default value.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  SDIO_DataInitStruct: pointer to an SDIO_DataInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */

 void SDIO_DataSetupStructInit(SDIO_TypeDef *SDIO,
                               SDIO_DataSetupTypeDef *SDIO_DataSetupStruct)
 {
     SDIO_DataSetupStruct->Data_en = 0;
     SDIO_DataSetupStruct->Data_rwn = 0;
     SDIO_DataSetupStruct->Data_mode = 0;
     SDIO_DataSetupStruct->Block_num = 0;
     SDIO_DataSetupStruct->Block_size = 0;
 }

 /**
  * @brief  Initializes the SDIO data setup according to the specified
  *         parameters in the SDIO_DataInitStruct.
  * @param  SDIO: select the SDIO peripheral.
  * @param  SDIO_DataInitStruct : pointer to a SDIO_DataInitTypeDef structure
  *         that contains the configuration information for the SDIO command.
  * @retval None
  */
 void SDIO_DataSetup(SDIO_TypeDef *SDIO,
                     SDIO_DataSetupTypeDef *SDIO_DataSetupStruct)
 {
     uint32_t tmp = 0;
     tmp = SDIO_DataSetupStruct->Data_en | SDIO_DataSetupStruct->Data_rwn |
           SDIO_DataSetupStruct->Data_mode | SDIO_DataSetupStruct->Block_num |
           SDIO_DataSetupStruct->Block_size;
     SDIO->DATA_SETUP = tmp;
 }

 /**
  * @brief  Clear the SDIO data setup .
  * @param  SDIO: select the SDIO peripheral.
  * @retval None
  */
 void SDIO_ClearDataSetup(SDIO_TypeDef *SDIO)
 {
     uint32_t tmp = 0;
     SDIO->DATA_SETUP = tmp;
 }

 /**
  * @brief  This bit field represents the SDIO write data timeout counter.
  * @param  SDIO: select the SDIO peripheral.
  * @param  Data: 32-bit data word to write.
  * @retval None
  */
 void SDIO_SetDateTimeout(SDIO_TypeDef *SDIO, uint32_t timeout)
 {
     SDIO->DATA_TIMEOUT_CNT = timeout;
 }

 /**
  * @brief  Initializes the SDIO Command according to the specified
  *         parameters in the SDIO_CmdInitStruct and send the command.
  * @param  SDIO: select the SDIO peripheral.
  * @param  SDIO_CmdInitStruct : pointer to a SDIO_CmdInitTypeDef
  *         structure that contains the configuration information for the SDIO
  *         command.
  * @retval None
  */
 void SDIO_SendCommand(SDIO_TypeDef *SDIO,
                       SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
 {
     uint32_t tmpreg = 0;

     /*---------------------------- SDIO ARG Configuration
      * ------------------------*/
     /* Set the SDIO Argument value */
     SDIO->CMD_ARG = SDIO_CmdInitStruct->SDIO_Argument;

     /*---------------------------- SDIO CMD Configuration
      * ------------------------*/
     /* Get the SDIO CMD value */
     tmpreg = SDIO->CMD_OP;
     tmpreg &= ~(BITS(0, 4) | BITS(8, 13));
     tmpreg =
         SDIO_CmdInitStruct->SDIO_No_Rsp | SDIO_CmdInitStruct->SDIO_No_Rsp_Len |
         SDIO_CmdInitStruct->SDIO_CrcEn | SDIO_CmdInitStruct->SDIO_Busy |
         SDIO_CmdInitStruct->SDIO_Power_Up | SDIO_CmdInitStruct->SDIO_CmdIndex;

     /* Write to SDIO CMD */
     SDIO->CMD_OP = tmpreg;

     /* Start transmission */
     SDIO->START = SDIO_START_ENABLE;
 }

 /**
  * \brief  Setup sdio clock divider.
  * @param  SDIO: select the SDIO peripheral.
  * @param  Data: 32-bit data word to write.
  */
 void SDIO_Clock_Set(SDIO_TypeDef *SDIO, uint32_t clkdiv)
 {
     uint32_t tmpreg = SDIO->CLK_DIV;
     tmpreg &= 0x0;
     tmpreg |= clkdiv;
     SDIO->CLK_DIV = tmpreg;
 }

 /**
  * @brief  Write one data word to Tx FIFO.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  Data: 32-bit data word to write.
  * @retval None
  */
 void SDIO_SendData(SDIO_TypeDef *SDIO, uint32_t data) { SDIO->TX_DATA = data; }

 /**
  * @brief  Read one data word from Rx FIFO.
  * @param  SDIO:  select the SDIO peripheral.
  * @param  None
  * @retval Data received
  */
 uint32_t SDIO_ReadData(SDIO_TypeDef *SDIO) { return SDIO->RX_DATA; }

 /**
  * @brief  Config Tx FIFO watermark.
  * @param  SDIO: select the SDIO peripheral.
  * @param  depth Tx watermark
  * @retval None
  */
 uint32_t SDIO_SetTxMark(SDIO_TypeDef *SDIO, uint32_t depth)
 {
     SDIO->TX_MARK = depth;
 }

 /**
  * @brief  Config Rx FIFO watermark.
  * @param  SDIO: select the SDIO peripheral.
  * @param  depth Rx watermark
  * @retval None
  */
 uint32_t SDIO_SetRxMark(SDIO_TypeDef *SDIO, uint32_t depth)
 {
     SDIO->RX_MARK = depth;
 }

 /**
  * @brief  Config interrupt enable.
  * @param  SDIO: select the SDIO peripheral.
  * @param  irq_en interrupt enable
  * @retval None
  */
 uint32_t SDIO_InterruptEn(SDIO_TypeDef *SDIO, SDIO_IntTypedef interrupt,
                           ControlStatus control)
 {
     if (ENABLE == control)
     {
         SDIO->IE |= interrupt;
     }
     else
     {
         SDIO->IE &= ~(interrupt);
     }
 }
