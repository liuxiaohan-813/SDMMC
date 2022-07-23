
/*!
 * @file     sdio.h
 * @brief    This file contains all the functions prototypes for the SDIO firmware
 */

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdio.h>

#ifdef __cplusplus
    extern "C"
    {
#endif

#include "sdcard.h"

/**
  * @brief SDIO
  */

typedef struct {
    __IOM uint32_t RX_SADDR;                            /*!< Offset: 0x000 (R/W) uDMA RX SDIO buffer base address configuration register */
    __IOM uint32_t RX_SIZE;                             /*!< Offset: 0x004 (R/W) uDMA RX SDIO buffer size configuration register */
    __IOM uint32_t RX_CFG;                              /*!< Offset: 0x008 (R/W) uDMA RX SDIO stream configuration register */
    __IOM uint32_t CR;                                  /*!< Offset: 0x00c (R/W) SDIO control register */
    __IOM uint32_t TX_SADDR;                            /*!< Offset: 0x010 (R/W) uDMA TX SDIO buffer base address configuration register */
    __IOM uint32_t TX_SIZE;                             /*!< Offset: 0x014 (R/W) uDMA TX SDIO buffer size configuration register */
    __IOM uint32_t TX_CFG;                              /*!< Offset: 0x018 (R/W) uDMA TX SDIO stream configuration register */
    __IOM uint32_t VERSION;                             /*!< Offset: 0x01c (R/W) SDIO version */
    __IOM uint32_t CMD_OP;                              /*!< Offset: 0x020 (R/W) SDIO command */
    __IOM uint32_t CMD_ARG;                             /*!< Offset: 0x024 (R/W) SDIO argument */
    __IOM uint32_t DATA_SETUP;                          /*!< Offset: 0x028 (R/W) SDIO Data transfer setup */
    __IOM uint32_t START;                               /*!< Offset: 0x02c (R/W) SDIO Start */
    __IOM uint32_t RSP0;                                /*!< Offset: 0x030 (R/W) SDIO Response 0 */
    __IOM uint32_t RSP1;                                /*!< Offset: 0x034 (R/W) SDIO Response 1 */
    __IOM uint32_t RSP2;                                /*!< Offset: 0x038 (R/W) SDIO Response 2 */
    __IOM uint32_t RSP3;                                /*!< Offset: 0x03c (R/W) SDIO Response 3 */
    __IOM uint32_t CLK_DIV;                             /*!< Offset: 0x040 (R/W) SDIO Clock Divider */
    __IOM uint32_t STATUS;                              /*!< Offset: 0x044 (R/W) SDIO STATUS register */
    __IOM uint32_t STOP_CMD_OP;                         /*!< Offset: 0x048 (R/W) SDIO STOP command op */
    __IOM uint32_t STOP_CMD_ARG;                        /*!< Offset: 0x04c (R/W) SDIO STOP command arg */
    __IOM uint32_t DATA_TIMEOUT_CNT;                    /*!< Offset: 0x050 (R/W) SDIO data timeout delay counter register */
    __IOM uint32_t CMD_POWERUP_CNT;                     /*!< Offset: 0x054 (R/W) SDIO command power up counter config */
    __IOM uint32_t CMD_WAIT_RSP_CNT;                    /*!< Offset: 0x058 (R/W) SDIO wait respone counter congfig */
    __IOM uint32_t CMD_WAIT_EOT_CNT;                    /*!< Offset: 0x05c (R/W) SDIO wait eot counter congfig */
    __IOM uint32_t TX_DATA;                             /*!< Offset: 0x060 (R/W) SDIO send data */
    __IOM uint32_t RX_DATA;                             /*!< Offset: 0x064 (R/W) SDIO receive data */
    __IOM uint32_t TX_MARK;                             /*!< Offset: 0x068 (R/W) SDIO Tx FIFO watermark */
    __IOM uint32_t RX_MARK;                             /*!< Offset: 0x06c (R/W) SDIO Rx FIFO watermark */
    __IOM uint32_t IP;                                  /*!< Offset: 0x070 (R/W) SDIO interrupt pending */
    __IOM uint32_t IE;                                  /*!< Offset: 0x074 (R/W) SDIO interrupt enable */
    __IOM uint32_t SAMPLE_DDR;                          /*!< Offset: 0x078 (R/W) SDIO ddr sample select */
    __IOM uint32_t RESERVED0[2];                        /*!< RESERVED */
    __IOM uint32_t DATA_TX_DELAY_CNT;                   /*!< Offset: 0x084 (R/W) SDIO data tx delay counter register */
    __IOM uint32_t DATA_CRC_TOKEN_CNT;                  /*!< Offset: 0x088 (R/W) SDIO crc token timeout delay counter register */
    __IOM uint32_t CRC_VALUE;                           /*!< Offset: 0x08c (R/W) SDIO cmd crc value */
}SDIO_TypeDef;

typedef struct {
    __IOM uint32_t DMA_CFG_PCTRL;                       /*!< Clock Gating Register (only present when peripherals
                                                            support is configured on rtl level) */
    __IOM uint32_t DMA_CFG_EVENT;                       /*!< Event ID Register (only present when peripherals support
                                                            is configured on rtl level) */
    __IM  uint32_t RESERVED0[766];                      /*!< RESERVED */
    __IOM uint32_t DMA_PCH0_IRQ_EN;                     /*!< P2M Channel0 Interrupt Enable Register */
    __IOM uint32_t DMA_PCH0_IRQ_STAT;                   /*!< P2M Channel0 Interrupt Status Flag Register */
    __IOM uint32_t DMA_PCH0_IRQ_CLR;                    /*!< P2M Channel0 Interrupt Clear Status Register */
} SDIO_DMA_TypeDef;


/* ===== SDIO_CR Register Definition===== */
#define SDIO_SDIO_CR_DMA_EN_MASK BIT(0) /*!< uDMA mode enable */
#define SDIO_SDIO_CR_DMA_EN(regval) (SDIO_SDIO_CR_DMA_EN_MASK & ((uint32_t)(regval) << 0))
#define SDIO_CR_DMA_ENABLE SDIO_SDIO_CR_DMA_EN(1)
#define SDIO_CR_DMA_DISABLE SDIO_SDIO_CR_DMA_EN(0)

#define SDIO_SDIO_CR_DDR_EN_MASK BIT(1) /*!< Double Data Rate (DDR) mode enable */
#define SDIO_SDIO_CR_DDR_EN(regval) (SDIO_SDIO_CR_DDR_EN_MASK & ((uint32_t)(regval) << 1))
#define SDIO_CR_DDR_ENABLE SDIO_SDIO_CR_DDR_EN(1)
#define SDIO_CR_DDR_DISABLE SDIO_SDIO_CR_DDR_EN(0)

#define SDIO_SDIO_CR_AUTO_CMD12_EN_MASK BIT(2) /*!< This bit field is used to enable automatic transmission of cmd12 mode */
#define SDIO_SDIO_CR_AUTO_CMD12_EN(regval) (SDIO_SDIO_CR_AUTO_CMD12_EN_MASK & ((uint32_t)(regval) << 2))
#define SDIO_CR_AUTO_CMD12_ENABLE SDIO_SDIO_CR_AUTO_CMD12_EN(1)
#define SDIO_CR_AUTO_CMD12_DISABLE SDIO_SDIO_CR_AUTO_CMD12_EN(0)

#define SDIO_SDIO_CR_BLOCK_EN_MASK BIT(3) /*!< This bit field is used to enable block_num mode */
#define SDIO_SDIO_CR_BLOCK_EN(regval) (SDIO_SDIO_CR_BLOCK_EN_MASK & ((uint32_t)(regval) << 3))
#define SDIO_CR_BLOCK_ENABLE SDIO_SDIO_CR_BLOCK_EN(1)
#define SDIO_CR_BLOCK_DISABLE SDIO_SDIO_CR_BLOCK_EN(0)

#define SDIO_SDIO_CR_DAT_LINE_BSY_CHECK_EN_MASK BIT(4) /*!< This bit field is used to check data line busy when cmd is active */
#define SDIO_SDIO_CR_DAT_LINE_BSY_CHECK_EN(regval) (SDIO_SDIO_CR_DAT_LINE_BSY_CHECK_EN_MASK & ((uint32_t)(regval) << 4))
#define SDIO_CR_DAT_LINE_BSY_CHECK_ENABLE SDIO_SDIO_CR_DAT_LINE_BSY_CHECK_EN(1)
#define SDIO_CR_DAT_LINE_BSY_CHECK_DISABLE SDIO_SDIO_CR_DAT_LINE_BSY_CHECK_EN(0)

#define SDIO_SDIO_CR_CLK_STOP_EN_MASK BIT(5) /*!< If this bit domain is turned on, the SD card clock will be                                                                         stopped each time the transfer is completed */
#define SDIO_SDIO_CR_CLK_STOP_EN(regval) (SDIO_SDIO_CR_CLK_STOP_EN_MASK & ((uint32_t)(regval) << 5))
#define SDIO_CR_CLK_STOP_ENABLE SDIO_SDIO_CR_CLK_STOP_EN(1)
#define SDIO_CR_CLK_STOP_DISABLE SDIO_SDIO_CR_CLK_STOP_EN(0)

#define SDIO_SDIO_CR_HIGH_WIDTH_MODE_EN_MASK BIT(6) /*!< High bandwidth mode is turned on, the data will be four bytes                                                                         bandwidth throughput data, when there is no 4 bytes of aligned data,                                                                         the data will only send and receive the number of valid bytes */
#define SDIO_SDIO_CR_HIGH_WIDTH_MODE_EN(regval) (SDIO_SDIO_CR_HIGH_WIDTH_MODE_EN_MASK & ((uint32_t)(regval) << 6))
#define SDIO_CR_HIGH_WIDTH_MODE_ENABLE SDIO_SDIO_CR_HIGH_WIDTH_MODE_EN(1)
#define SDIO_CR_HIGH_WIDTH_MODE_DISABLE SDIO_SDIO_CR_HIGH_WIDTH_MODE_EN(0)

/* ===== SDIO_RX_CFG Register Definition===== */
#define SDIO_SDIO_RX_CFG_CONTINUOUS_MASK BIT(0) /*!< RX channel continuous mode bitfield */
#define SDIO_SDIO_RX_CFG_CONTINUOUS_EN(regval) (SDIO_SDIO_RX_CFG_CONTINUOUS_MASK & ((uint32_t)(regval) << 0))
#define SDIO_RX_CFG_CONTINUOUS_ENABLE SDIO_SDIO_RX_CFG_CONTINUOUS_EN(1)
#define SDIO_RX_CFG_CONTINUOUS_DISABLE SDIO_SDIO_RX_CFG_CONTINUOUS_EN(0)

#define SDIO_SDIO_RX_CFG_DATASIZE_MASK BITS(1, 2) /*!< Channel transfer size used to increment uDMA buffer address pointer */
#define SDIO_SDIO_RX_CFG_DATASIZE_EN(regval) (SDIO_SDIO_RX_CFG_DATASIZE_MASK & ((uint32_t)(regval) << 1))
#define SDIO_RX_CFG_DATASIZE_8BIT SDIO_SDIO_RX_CFG_DATASIZE_EN(0)
#define SDIO_RX_CFG_DATASIZE_16BIT SDIO_SDIO_RX_CFG_DATASIZE_EN(1)
#define SDIO_RX_CFG_DATASIZE_32BIT SDIO_SDIO_RX_CFG_DATASIZE_EN(2)
#define SDIO_RX_CFG_DATASIZE_0BIT SDIO_SDIO_RX_CFG_DATASIZE_EN(3)

#define SDIO_SDIO_RX_CFG_EN_MASK BIT(4) /*!< RX channel enable and start transfer bitfield */
#define SDIO_SDIO_RX_CFG_EN(regval) (SDIO_SDIO_RX_CFG_EN_MASK & ((uint32_t)(regval) << 4))
#define SDIO_RX_CFG_EN_ENABLE SDIO_SDIO_RX_CFG_EN(1)
#define SDIO_RX_CFG_EN_DISABLE SDIO_SDIO_RX_CFG_EN(0)

#define SDIO_SDIO_RX_CFG_PENDING_MASK BIT(5) /*!< RX transfer pending in queue status flag */
#define SDIO_SDIO_RX_CFG_PENDING_EN(regval) (SDIO_SDIO_RX_CFG_PENDING_MASK & ((uint32_t)(regval) << 5))
#define SDIO_RX_CFG_PENDING_ENABLE SDIO_SDIO_RX_CFG_PENDING_EN(1)
#define SDIO_RX_CFG_PENDING_DISABLE SDIO_SDIO_RX_CFG_PENDING_EN(0)

#define SDIO_SDIO_RX_CFG_CLR_MASK BIT(6) /*!< RX channel clear and stop transfer bitfield */
#define SDIO_SDIO_RX_CFG_CLR_EN(regval) (SDIO_SDIO_RX_CFG_CLR_MASK & ((uint32_t)(regval) << 6))
#define SDIO_RX_CFG_CLR_ENABLE SDIO_SDIO_RX_CFG_CLR_EN(1)
#define SDIO_RX_CFG_CLR_DISABLE SDIO_SDIO_RX_CFG_CLR_EN(0)

/* ===== SDIO_TX_CFG Register Definition===== */
#define SDIO_SDIO_TX_CFG_CONTINUOUS_MASK BIT(0) /*!< TX channel continuous mode bitfield */
#define SDIO_SDIO_TX_CFG_CONTINUOUS_EN(regval) (SDIO_SDIO_TX_CFG_CONTINUOUS_MASK & ((uint32_t)(regval) << 0))
#define SDIO_TX_CFG_CONTINUOUS_ENABLE SDIO_SDIO_TX_CFG_CONTINUOUS_EN(1)
#define SDIO_TX_CFG_CONTINUOUS_DISABLE SDIO_SDIO_TX_CFG_CONTINUOUS_EN(0)

#define SDIO_SDIO_TX_CFG_DATASIZE_MASK BITS(1, 2) /*!< Channel transfer size used to increment uDMA buffer address pointer */
#define SDIO_SDIO_TX_CFG_DATASIZE_EN(regval) (SDIO_SDIO_TX_CFG_DATASIZE_MASK & ((uint32_t)(regval) << 1))
#define SDIO_TX_CFG_DATASIZE_8BIT SDIO_SDIO_TX_CFG_DATASIZE_EN(0)
#define SDIO_TX_CFG_DATASIZE_16BIT SDIO_SDIO_TX_CFG_DATASIZE_EN(1)
#define SDIO_TX_CFG_DATASIZE_32BIT SDIO_SDIO_TX_CFG_DATASIZE_EN(2)
#define SDIO_TX_CFG_DATASIZE_0BIT SDIO_SDIO_TX_CFG_DATASIZE_EN(3)

#define SDIO_SDIO_TX_CFG_EN_MASK BIT(4) /*!< TX channel enable and start transfer bitfield */
#define SDIO_SDIO_TX_CFG_EN(regval) (SDIO_SDIO_TX_CFG_EN_MASK & ((uint32_t)(regval) << 4))
#define SDIO_TX_CFG_EN_ENABLE SDIO_SDIO_TX_CFG_EN(1)
#define SDIO_TX_CFG_EN_DISABLE SDIO_SDIO_TX_CFG_EN(0)

#define SDIO_SDIO_TX_CFG_PENDING_MASK BIT(5) /*!< TX transfer pending in queue status flag */
#define SDIO_SDIO_TX_CFG_PENDING_EN(regval) (SDIO_SDIO_TX_CFG_PENDING_MASK & ((uint32_t)(regval) << 5))
#define SDIO_TX_CFG_PENDING_ENABLE SDIO_SDIO_TX_CFG_PENDING_EN(1)
#define SDIO_TX_CFG_PENDING_DISABLE SDIO_SDIO_TX_CFG_PENDING_EN(0)

#define SDIO_SDIO_TX_CFG_CLR_MASK BIT(6) /*!< TX channel clear and stop transfer bitfield */
#define SDIO_SDIO_TX_CFG_CLR_EN(regval) (SDIO_SDIO_TX_CFG_CLR_MASK & ((uint32_t)(regval) << 6))
#define SDIO_TX_CFG_CLR_ENABLE SDIO_SDIO_TX_CFG_CLR_EN(1)
#define SDIO_TX_CFG_CLR_DISABLE SDIO_SDIO_TX_CFG_CLR_EN(0)

/* ===== SDIO_DATA_SETUP Register Definition===== */
#define SDIO_SDIO_DATA_SETUP_DATA_EN_MASK BIT(0) /*!< SDIO data channel enable */
#define SDIO_SDIO_DATA_SETUP_DATA_EN(regval) (SDIO_SDIO_DATA_SETUP_DATA_EN_MASK & ((uint32_t)(regval) << 0))
#define SDIO_DATA_CHANNEL_ENABLE SDIO_SDIO_DATA_SETUP_DATA_EN(1)
#define SDIO_DATA_CHANNEL_DISABLE SDIO_SDIO_DATA_SETUP_DATA_EN(0)

#define SDIO_SDIO_DATA_SETUP_DATA_RWN_MASK BIT(1) /*!< SDIO read / write enable */
#define SDIO_SDIO_DATA_SETUP_DATA_RWN(regval) (SDIO_SDIO_DATA_SETUP_DATA_RWN_MASK & ((uint32_t)(regval) << 1))
#define SDIO_DATA_RWN_WRITE (SDIO_SDIO_DATA_SETUP_DATA_RWN(0))
#define SDIO_DATA_RWN_READ (SDIO_SDIO_DATA_SETUP_DATA_RWN(1))

#define SDIO_SDIO_DATA_SETUP_DATA_MODE_MASK BITS(2, 3) /*!< Data mode selection */
#define SDIO_SDIO_DATA_SETUP_DATA_MODE(regval) (SDIO_SDIO_DATA_SETUP_DATA_MODE_MASK & ((uint32_t)(regval) << 2))
#define SDIO_SINGLE_LINE SDIO_SDIO_DATA_SETUP_DATA_MODE(0)
#define SDIO_QUAD_LINE SDIO_SDIO_DATA_SETUP_DATA_MODE(1)
#define SDIO_OCTOL_LINE SDIO_SDIO_DATA_SETUP_DATA_MODE(2)

#define SDIO_SDIO_DATA_SETUP_BLOCK_NUM_MASK BITS(4, 19) /*!< This field indicates the number of SDIO blocks,                                                                            block_num = actual number of blocks - 1 */
#define SDIO_SDIO_DATA_SETUP_BLOCK_NUM(regval) (SDIO_SDIO_DATA_SETUP_BLOCK_NUM_MASK & ((uint32_t)(regval) << 4))

#define SDIO_SDIO_DATA_SETUP_BLOCK_SIZE_MASK BITS(20, 31) /*!< This field indicates that each block size (byte)                                                                             of SDIOis 4 bytes aligned, the range is 3-1023,                                                                             and the real block = block_size + 1 (bytes) */
#define SDIO_SDIO_DATA_SETUP_BLOCK_SIZE(regval) (SDIO_SDIO_DATA_SETUP_BLOCK_SIZE_MASK & ((uint32_t)(regval) << 20))

/* ===== SDIO_START Register Definition===== */
#define SDIO_SDIO_START_EN_MASK BIT(0) /*!< Start transmission, write 1 to generate pulse and reset                                                                     automatically */
#define SDIO_SDIO_START_EN(regval) (SDIO_SDIO_START_EN_MASK & ((uint32_t)(regval) << 0))
#define SDIO_START_ENABLE SDIO_SDIO_START_EN(1)
#define SDIO_START_DISABLE SDIO_SDIO_START_EN(0)

/* ===== SDIO_CMD_OP Register Definition===== */
#define SDIO_SDIO_CMD_OP_NO_RSP_MASK BIT(0) /*!< This bit field indicates whether to receive a response */
#define SDIO_SDIO_CMD_OP_NO_RSP(regval) (SDIO_SDIO_CMD_OP_NO_RSP_MASK & ((uint32_t)(regval) << 0))
#define SDIO_RESPONSE_NO SDIO_SDIO_CMD_OP_NO_RSP(0)
#define SDIO_RESPONSE_YES SDIO_SDIO_CMD_OP_NO_RSP(1)

#define SDIO_SDIO_CMD_OP_RSP_LEN_MASK BIT(1) /*!< This bit field represents the received response length selection */
#define SDIO_SDIO_CMD_OP_RSP_LEN(regval) (SDIO_SDIO_CMD_OP_RSP_LEN_MASK & ((uint32_t)(regval) << 1))
#define SDIO_RESPONSE_LEN_48 SDIO_SDIO_CMD_OP_RSP_LEN(0)
#define SDIO_RESPONSE_LEN_136 SDIO_SDIO_CMD_OP_RSP_LEN(1)

#define SDIO_SDIO_CMD_OP_CRC_MASK BIT(2) /*!< This bit field indicates whether to enable CRC checking */
#define SDIO_SDIO_CMD_OP_CRC(regval) (SDIO_SDIO_CMD_OP_CRC_MASK & ((uint32_t)(regval) << 2))
#define SDIO_CRC_ENABLE SDIO_SDIO_CMD_OP_CRC(1)
#define SDIO_CRC_DISABLE SDIO_SDIO_CMD_OP_CRC(0)

#define SDIO_SDIO_CMD_OP_BUSY_MASK BIT(3) /*!< This bit field indicates whether to enable busy check */
#define SDIO_SDIO_CMD_OP_BUSY(regval) (SDIO_SDIO_CMD_OP_BUSY_MASK & ((uint32_t)(regval) << 3))
#define SDIO_BUSY_ENABLE SDIO_SDIO_CMD_OP_BUSY(1)
#define SDIO_BUSY_DISABLE SDIO_SDIO_CMD_OP_BUSY(0)

#define SDIO_SDIO_CMD_OP_POWER_UP_MASK BIT(4) /*!< This bit field indicates whether to enable busy check */
#define SDIO_SDIO_CMD_OP_POWER_UP(regval) (SDIO_SDIO_CMD_OP_POWER_UP_MASK & ((uint32_t)(regval) << 4))
#define SDIO_POWER_UP_ENABLE SDIO_SDIO_CMD_OP_POWER_UP(1)
#define SDIO_POWER_UP_DISABLE SDIO_SDIO_CMD_OP_POWER_UP(0)

#define SDIO_SDIO_CMD_OP_CRC_CHECK_ERR_MASK BIT(5) /*!< This bit field indicates whether to enable crc error check */
#define SDIO_SDIO_CMD_OP_CRC_CHECK_ERR(regval) (SDIO_SDIO_CMD_OP_CRC_CHECK_ERR_MASK & ((uint32_t)(regval) << 5))
#define SDIO_CRC_CHECK_ERR_ENABLE SDIO_SDIO_CMD_OP_CRC_CHECK_ERR(1)
#define SDIO_CRC_CHECK_ERR_DISABLE SDIO_SDIO_CMD_OP_CRC_CHECK_ERR(0)

#define SD_CHECK_PATTERN ((uint32_t)0x000001AA)

/* ===== SDIO Status Register definition ===== */
#define SDIO_STATUS_EOT BIT(0)       /*!< SDIO transmission end flag */
#define SDIO_STATUS_ERR BIT(1)       /*!< SDIO transmission error flag */
#define SDIO_STATUS_TXUDR_ERR BIT(2) /*!< SDIO tx udr error flag */
#define SDIO_STATUS_TXOVF_ERR BIT(3) /*!< SDIO tx ovf error flag */
#define SDIO_STATUS_RXUDR_ERR BIT(4) /*!< SDIO rx udr error flag */
#define SDIO_STATUS_RXOVF_ERR BIT(5) /*!< SDIO rx ovf error flag */

#define SDIO_STATUS_CMD_ERR_MASK BITS(16, 21)                                         /*!< SDIO CMD ERROR STATUS: Bits Mask */
#define SDIO_STATUS_CMD_ERR(regval) (SDIO_STATUS_CMD_ERR_MASK & ((uint32_t)(regval))) /*!< SDIO CMD ERROR STATUS: Bits Value */
#define SDIO_STATUS_CMD_ERR_NOERR SDIO_STATUS_CMD_ERR(0)                              /*!< SDIO CMD ERROR STATUS: No Error */
#define SDIO_STATUS_CMD_ERR_TIMEOUT SDIO_STATUS_CMD_ERR(1)                            /*!< SDIO CMD ERROR STATUS: Response Time Out */
#define SDIO_STATUS_CMD_ERR_BUSYOUT SDIO_STATUS_CMD_ERR(2)                            /*!< SDIO CMD ERROR STATUS: Response Busy Out */
#define SDIO_STATUS_CMD_ERR_CRCSTA_TIMEOUT SDIO_STATUS_CMD_ERR(4)                     /*!< SDIO CMD ERROR STATUS: TX crcsta Time Out */
#define SDIO_STATUS_CMD_ERR_CRC SDIO_STATUS_CMD_ERR(8)                                /*!< SDIO CMD ERROR STATUS: Response CRC check Error */

#define SDIO_STATUS_DATA_ERR_MASK BITS(24, 29)                                          /*!< SDIO DATA ERROR STATUS: Bits Mask */
#define SDIO_STATUS_DATA_ERR(regval) (SDIO_STATUS_DATA_ERR_MASK & ((uint32_t)(regval))) /*!< SDIO DATA ERROR STATUS: Bits Value */
#define SDIO_STATUS_DATA_ERR_NOERR SDIO_STATUS_DATA_ERR(0)                              /*!< SDIO DATA ERROR STATUS: No Error */
#define SDIO_STATUS_DATA_ERR_TIMEOUT SDIO_STATUS_DATA_ERR(1)                            /*!< SDIO DATA ERROR STATUS: Response Time Out */
#define SDIO_STATUS_DATA_ERR_BUSYOUT SDIO_STATUS_DATA_ERR(2)                            /*!< SDIO DATA ERROR STATUS: Response Busy Out */
#define SDIO_STATUS_DATA_ERR_CRCSTA_TIMEOUT SDIO_STATUS_DATA_ERR(3)                     /*!< SDIO DATA ERROR STATUS: TX crcsta Time Out */
#define SDIO_STATUS_DATA_ERR_CRC SDIO_STATUS_DATA_ERR(4)                                /*!< SDIO DATA ERROR STATUS: Response CRC check Error */

#define SDIO_STATUS_DATA_RUNNING BIT(30) /*!< This bit field indicates whether the data channel is working */

        /* ===== SDIO_IP Register Definition===== */
        typedef enum
        {
            SDIO_IP_TXFULL = 0x1 << 3,  /*!< SDIO IP: send fifo full */
            SDIO_IP_RXEMPTY = 0x1 << 2, /*!< SDIO IP: receive fifo empty */
            SDIO_IP_RXIRQ = 0x1 << 1,   /*!< SDIO IP: Receive watermark enable */
            SDIO_IP_TXIRQ = 0x1 << 0,   /*!< SDIO IP: Transmit watermark enable */
        } SDIO_IntFlagTypedef;

        /* ===== SDIO_IE Register Definition===== */
        typedef enum
        {
            SDIO_IE_RXOVF_ERR = 0x1 << 7, /*!< SDIO IE: Rx ovf error interrupt enable */
            SDIO_IE_RXUDR_ERR = 0x1 << 6, /*!< SDIO IE: Rx udr error interrupt enable */
            SDIO_IE_TXOVF_ERR = 0x1 << 5, /*!< SDIO IE: Tx ovf error interrupt enable */
            SDIO_IE_TXUDR_ERR = 0x1 << 4, /*!< SDIO IE: Tx udr error interrupt enable */
            SDIO_IE_ERR_IRQ = 0x1 << 3,   /*!< SDIO IE: Error interrupt enable */
            SDIO_IE_EOT_IRQ = 0x1 << 2,   /*!< SDIO IE: End interrupt enable */
            SDIO_IE_RX_IRQ = 0x1 << 1,    /*!< SDIO IE: Receive watermark enable */
            SDIO_IE_TX_IRQ = 0x1 << 0,    /*!< SDIO IE: Transmit watermark enable */
        } SDIO_IntTypedef;

        typedef struct
        {
            uint32_t SDIO_Argument; /*!< Specifies the SDIO command argument which is sent
                                       to a card as part of a command message. If a command
                                       contains an argument, it must be loaded into this register
                                       before writing the command to the command register */
            uint32_t SDIO_No_Rsp;   /*!< Specifies the SDIO response type.
                                        This parameter can be a value of @ref SDIO_Response_Type */
            uint32_t SDIO_No_Rsp_Len;
            uint32_t SDIO_CrcEn;
            uint32_t SDIO_Busy; /*!< Specifies whether SDIO wait for interrupt request is enabled or disabled.
                                    This parameter can be a value of @ref SDIO_Wait_Interrupt_State */
            uint32_t SDIO_Power_Up;
            uint32_t SDIO_CmdIndex; /*!< Specifies the SDIO command index. It must be lower than 0x40. */
        } SDIO_CmdInitTypeDef;

        typedef struct
        {
            uint32_t Dma_en;
            uint32_t Rx_addr;
            uint32_t Rx_size;
            uint32_t Rx_en;
            uint32_t Rx_datasize;

            uint32_t Tx_addr;
            uint32_t Tx_size;
            uint32_t Tx_en;
            uint32_t Tx_datasize;
        } SDIO_DmaCfgTypeDef;

        typedef struct
        {
            uint32_t Data_en;
            uint32_t Data_rwn;
            uint32_t Data_mode;
            uint32_t Block_num;
            uint32_t Block_size;
        } SDIO_DataSetupTypeDef;

#define SDIO_FLAG_TXDAVL ((uint32_t)0x00100000)
#define SDIO_FLAG_RXDAVL ((uint32_t)0x00200000)
#define SDIO_FLAG_SDIOIT ((uint32_t)0x00400000)
#define SDIO_FLAG_CEATAEND ((uint32_t)0x00800000)

#define SD_POLLING_MODE 0
#define SD_DMA_MODE 1
#define SDIO_SEND_IF_COND ((uint32_t)0x00000008)

#define SDIO_GET_STATUS(__SDIO__) __SDIO__->STATUS
#define SDIO_GET_STATU_FLAG(__SDIO__, __FLAG__) (((__SDIO__)->STATUS & (__FLAG__)) == (__FLAG__))
#define SDIO_GET_IP_FLAG(__SDIO__, __IP__) (((__SDIO__)->IP & (__IP__)) == (__IP__))

        void SDIO_DmaCfgStructInit(SDIO_TypeDef *SDIO, SDIO_DmaCfgTypeDef *SDIO_DmaCfgStruct);
        void SDIO_DMA_Config(SDIO_TypeDef *SDIO, SDIO_DmaCfgTypeDef *SDIO_DmaCfgStruct);
        void SDIO_CmdInitStructInit(SDIO_TypeDef *SDIO, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
        void SDIO_SendCommand(SDIO_TypeDef *SDIO, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
        void SDIO_DataSetupStructInit(SDIO_TypeDef *SDIO, SDIO_DataSetupTypeDef *SDIO_DataSetupStruct);
        void SDIO_DataSetup(SDIO_TypeDef *SDIO, SDIO_DataSetupTypeDef *SDIO_DataSetupStruct);
        void SDIO_Clock_Set(SDIO_TypeDef *SDIO, uint32_t clkdiv);
        void SDIO_ClearDataSetup(SDIO_TypeDef *SDIO);
        void SDIO_SetDateTimeout(SDIO_TypeDef *SDIO, uint32_t timeout);
        void SDIO_SendData(SDIO_TypeDef *SDIO, uint32_t data);
        uint32_t SDIO_ReadData(SDIO_TypeDef *SDIO);
        uint32_t SDIO_GetFlagStatus(SDIO_TypeDef *SDIO, uint32_t status);
        void SDIO_ClearFlag(SDIO_TypeDef *SDIO, uint32_t status);
        uint32_t SDIO_GetIPStatus(SDIO_TypeDef *SDIO, uint32_t status);
        uint32_t SDIO_SetTxMark(SDIO_TypeDef *SDIO, uint32_t depth);
        uint32_t SDIO_SetRxMark(SDIO_TypeDef *SDIO, uint32_t depth);
        uint32_t SDIO_InterruptEn(SDIO_TypeDef *SDIO, SDIO_IntTypedef interrupt, ControlStatus control);
#ifdef __cplusplus
    }
#endif

#endif
