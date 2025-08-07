
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
 uint32_t SDIO_GetCmdError(SDIO_TypeDef *SDIOx);
 uint32_t SDIO_GetCmdResp1(SDIO_TypeDef *SDIOx, uint32_t *pResponse);
 uint32_t SDIO_GetCmdResp2(SDIO_TypeDef *SDIOx);
 uint32_t SDIO_GetCmdResp3(SDIO_TypeDef *SDIOx, uint32_t *pResponse);
 uint32_t SDIO_GetCmdResp7(SDIO_TypeDef *SDIOx);
 uint32_t SDIO_GetCmdResp6(SDIO_TypeDef *SDIOx, uint16_t SD_CMD, uint16_t *pRCA);

/* ===== SDIO_IP Register Definition===== */
typedef enum {
    SDIO_IP_TXFULL                 = 0x1 << 3,      /*!< SDIO IP: send fifo full */
    SDIO_IP_RXEMPTY                = 0x1 << 2,      /*!< SDIO IP: receive fifo empty */
    SDIO_IP_RXIRQ                  = 0x1 << 1,      /*!< SDIO IP: Receive watermark enable */
    SDIO_IP_TXIRQ                  = 0x1 << 0,      /*!< SDIO IP: Transmit watermark enable */
} SDIO_IntFlagTypedef;

/* ===== SDIO_IE Register Definition===== */
typedef enum {
    SDIO_IE_IRQ_CHECK_EN           = 0x1 << 8,    /*!< SDIO IE: sdio check interrupt enable */
    SDIO_IE_RXOVF_ERR              = 0x1 << 7,    /*!< SDIO IE: Rx ovf error interrupt enable */
    SDIO_IE_RXUDR_ERR              = 0x1 << 6,    /*!< SDIO IE: Rx udr error interrupt enable */
    SDIO_IE_TXOVF_ERR              = 0x1 << 5,    /*!< SDIO IE: Tx ovf error interrupt enable */
    SDIO_IE_TXUDR_ERR              = 0x1 << 4,    /*!< SDIO IE: Tx udr error interrupt enable */
    SDIO_IE_ERR_IRQ                = 0x1 << 3,    /*!< SDIO IE: Error interrupt enable */
    SDIO_IE_EOT_IRQ                = 0x1 << 2,    /*!< SDIO IE: End interrupt enable */
    SDIO_IE_RX_IRQ                 = 0x1 << 1,    /*!< SDIO IE: Receive watermark enable */
    SDIO_IE_TX_IRQ                 = 0x1 << 0,    /*!< SDIO IE: Transmit watermark enable */
} SDIO_IntTypedef;

/* ===== SDIO_DMA_IE Register Definition===== */
typedef enum {
    SDIO_TX_RSP_ERR_IRQ_EN               = 0x1 << 5,    /*!< SDIO DMA IE: Tx interrupt enable for channel DMA access error */
    SDIO_TX_HTRANS_IRQ_EN                = 0x1 << 4,    /*!< SDIO DMA IE: Tx interrupt enable for channel half transfer */
    SDIO_TX_FTRANS_IRQ_EN                = 0x1 << 3,    /*!< SDIO DMA IE: Tx interrupt enable for Channel0 full transfer */
    SDIO_RX_RSP_ERR_IRQ_EN               = 0x1 << 2,    /*!< SDIO DMA IE: Rx interrupt enable for channel DMA access error */
    SDIO_RX_HTRANS_IRQ_EN                = 0x1 << 1,    /*!< SDIO DMA IE: Rx interrupt enable for channel half transfer */
    SDIO_RX_FTRANS_IRQ_EN                = 0x1 << 0,    /*!< SDIO DMA IE: Rx interrupt enable for Channel0 full transfer */
} SDIO_DmaIntTypedef;

/* ===== SDIO_DMA_IE_STAT Register Definition===== */
typedef enum {
    SDIO_TX_RSP_ERR_IRQ_STAT               = 0x1 << 5,    /*!< SDIO DMA IE STAT: Tx interrupt irq status flag for Channel0 DMA access error */
    SDIO_TX_HTRANS_IRQ_STAT                = 0x1 << 4,    /*!< SDIO DMA IE STAT: Tx interrupt irq status flag for Channel0 half transfer */
    SDIO_TX_FTRANS_IRQ_STAT                = 0x1 << 3,    /*!< SDIO DMA IE STAT: Tx interrupt irq status flag for Channel0 full transfer */
    SDIO_RX_RSP_ERR_IRQ_STAT               = 0x1 << 2,    /*!< SDIO DMA IE STAT: Rx interrupt irq status flag for Channel0 DMA access error */
    SDIO_RX_HTRANS_IRQ_STAT                = 0x1 << 1,    /*!< SDIO DMA IE STAT: Rx interrupt irq status flag for Channel0 half transfer */
    SDIO_RX_FTRANS_IRQ_STAT                = 0x1 << 0,    /*!< SDIO DMA IE STAT: Rx interrupt irq status flag for Channel0 full transfer */
} SDIO_DmaIntStatTypedef;

/* ===== SDIO_DMA_IE_CLR Register Definition===== */
typedef enum {
    SDIO_TX_RSP_ERR_IRQ_CLR               = 0x1 << 5,    /*!< SDIO DMA IE CLR: Tx clear irq status flag for Channel0 DMA access error */
    SDIO_TX_HTRANS_IRQ_CLR                = 0x1 << 4,    /*!< SDIO DMA IE CLR: Tx clear irq status flag for Channel0 half transfer */
    SDIO_TX_FTRANS_IRQ_CLR                = 0x1 << 3,    /*!< SDIO DMA IE CLR: Tx clear irq status flag for Channel0 full transfer */
    SDIO_RX_RSP_ERR_IRQ_CLR               = 0x1 << 2,    /*!< SDIO DMA IE CLR: Rx clear irq status flag for Channel0 DMA access error */
    SDIO_RX_HTRANS_IRQ_CLR                = 0x1 << 1,    /*!< SDIO DMA IE CLR: Rx clear irq status flag for Channel0 half transfer */
    SDIO_RX_FTRANS_IRQ_CLR                = 0x1 << 0,    /*!< SDIO DMA IE CLR: Rx clear irq status flag for Channel0 full transfer */
} SDIO_DmaIntClrTypedef;

typedef struct
{
    uint32_t SDIO_Argument;     /*!< Specifies the SDIO command argument which is sent to a card as part of a command message. If a command
                                    contains an argument, it must be loaded into this register before writing the command to the command register */
    uint32_t SDIO_Response;        /*!< Specifies the SDIO response type. */
    uint32_t SDIO_RspLen;       /*!< Specifies the SDIO response length if has response. */
    uint32_t SDIO_CrcEn;        /*!< Specifies weather the SDIO send crc or not */
    uint32_t SDIO_BusyCheck;    /*!< Specifies whether the SDIO wait for device response crc status */
    uint32_t SDIO_PowerUp;      /*!< Specifies whether the SDIO enable power on initialization */
    uint32_t SDIO_CrcCheck;     /*!< Specifies whether the SDIO enable power on initialization */
    uint32_t SDIO_IsStopCmd;    /*!< Specifies whether the SDIO cmd is abort command or not*/
    uint32_t SDIO_CmdIndex;     /*!< Specifies the SDIO command index. It must be lower than 0x40. */
    uint32_t SDIO_CmdType;     /*!< Specifies the SDIO command index. It must be lower than 0x40. */
} SDIO_CmdInitTypeDef;

typedef struct
{
    uint32_t DmaEn;         /*!< Specifies the SDIO dma enable. */
    uint32_t RxAddr;        /*!< Specifies the SDIO read addr(low). */
    uint32_t RxAddrH;       /*!< Specifies the SDIO read addr(high). */
    uint32_t RxSize;        /*!< Specifies the SDIO rx num. */
    uint32_t RxEn;          /*!< Specifies the SDIO rx enable. */
    uint32_t RxDatasize;    /*!< Specifies the SDIO rx data size. */
    uint32_t TxAddr;        /*!< Specifies the SDIO write addr(low). */
    uint32_t TxAddrH;       /*!< Specifies the SDIO write addr(high). */
    uint32_t TxSize;        /*!< Specifies the SDIO tx num. */
    uint32_t TxEn;          /*!< Specifies the SDIO tx enable. */
    uint32_t TxDatasize;    /*!< Specifies the SDIO tx data size. */
} SDIO_DmaCfgTypeDef;

typedef struct
{
    uint32_t DataChannelEn;     /*!< Specifies the SDIO data channel enable. */
    uint32_t TransferDir;           /*!< Specifies the SDIO data dir. */
    uint32_t DataBusWidth;         /*!< Specifies the SDIO data proto. */
    uint32_t TransferMode;        /*!< Specifies whether data transfer is in stream or block mode. */
    uint32_t BlockNum;          /*!< Specifies the SDIO data block num. */
    uint32_t BlockSize;         /*!< Specifies the SDIO data block size. */
} SDIO_DataSetupTypeDef;

typedef struct
{
    
    uint32_t ClkDiv;
    uint32_t AutoStop;
    uint32_t ClkStop;
    uint32_t HighWidthMode;
    uint32_t DataCrcChk;
    
    uint32_t DataTimeOutCnt;
    uint32_t PowerUpCnt;
    uint32_t WaitRspCnt;
    uint32_t WaitEotCnt;
    uint32_t TxDelayCnt;
} SDIO_InitTypeDef;

typedef struct SDIO_DmaLinkListNodeTypeDef {
    uint32_t *addr;                                 /* physical address */
    uint32_t block_size;                            /* block size */
    uint32_t data_size;                             /* data size */
    uint32_t *config_addr;                          /* cfg_addr */
    struct SDIO_DmaLinkListNodeTypeDef *next;       /* next node*/
} SDIO_DmaLinkListNodeTypeDef;

typedef struct {
    uint32_t *addr;
    uint32_t block_size;
    uint32_t dma_config;
    uint32_t channel;
    struct SDIO_DmaLinkListNodeTypeDef *link_list;
} SDIO_DmaConfigListTypeDef;

#define SDIO_POWER_OFF      SDIO_CR_POWER_ON_MODE_ENABLE(2)
#define SDIO_POWER_ON       SDIO_CR_POWER_ON_MODE_ENABLE(3)

#define SDIO_DIR_WRITE      SDIO_DATA_SETUP_RWN_WRITE
#define SDIO_DIR_READ       SDIO_DATA_SETUP_RWN_READ

#define SDIO_1LINE          SDIO_DATA_SETUP_MODE_SINGLE
#define SDIO_4LINE          SDIO_DATA_SETUP_MODE_QUAD
#define SDIO_8LINE          SDIO_DATA_SETUP_MODE_OCTOL

#define SDIO_STREAM_MODE    SDIO_CR_STREAM_MODE_ENABLE
#define SDIO_NORMAL_MODE    SDIO_CR_STREAM_MODE_DISABLE

#define SDIO_FLAG_TXDAVL                    ((uint32_t)0x00100000)
#define SDIO_FLAG_RXDAVL                    ((uint32_t)0x00200000)
#define SDIO_FLAG_SDIOIT                    ((uint32_t)0x00400000)
#define SDIO_FLAG_CEATAEND                  ((uint32_t)0x00800000)

/** @defgroup SDIO_Response_Registers Response Register
  * @{
  */
#define SDIO_RESP1                          ((uint32_t)0x00000000U)
#define SDIO_RESP2                          ((uint32_t)0x00000001U)
#define SDIO_RESP3                          ((uint32_t)0x00000002U)
#define SDIO_RESP4                          ((uint32_t)0x00000003U)

#define IS_SDIO_RESP(RESP) (((RESP) == SDIO_RESP1) ||   \
                                            ((RESP) == SDIO_RESP2) ||      \
                                            ((RESP) == SDIO_RESP3) ||      \
                                            ((RESP) == SDIO_RESP4))

#define SDIO_POLLING_MODE    	0
#define SDIO_DMA_MODE    		1

#define SDIO_CMD_TYPE_COMMON   0
#define SDIO_CMD_TYPE_DATA    	1

#define SDIO_SDR_MODE 0
#define SDIO_DDR_MODE 1

#define SDIO_FLAG_DTIMEOUT                  ((uint32_t)0x00000008)

#define SDIO_CMDTIMEOUT                   ((uint32_t)5000U)        /* Command send and response timeout     */
#define SDIO_MAXERASETIMEOUT              ((uint32_t)63000U)       /* Max erase Timeout 63 s                */
#define SDIO_STOPTRANSFERTIMEOUT          ((uint32_t)100000000U)   /* Timeout for STOP TRANSMISSION command */

/** 
  * @brief  SDIO Static flags, TimeOut, FIFO Address
  */
#define SDIO_STATIC_FLAGS               ((uint32_t)(SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_TXUDR_ERR |SDIO_STATUS_TXOVF_ERR |                                                    SDIO_STATUS_RXUDR_ERR | SDIO_STATUS_RXOVF_ERR | SDIO_STATUS_BUSY | SDIO_STATUS_CLEAR_FIFO_STAR |                                                    SDIO_STATUS_IRQ_CHECK | SDIO_STATUS_BUSY0_END | SDIO_STATUS_VOLTAGE_SWITCH_PASS | SDIO_STATUS_VOLTAGE_SWITCH_FAIL |                                                    SDIO_STATUS_BOOT_ACK_PASS | SDIO_STATUS_BOOT_ACK_FAIL | SDIO_STATUS_CLK_STOP | SDIO_STATUS_VOLTAGE_SWITCH_END |                                                    SDIO_STATUS_DATA_RUNING | SDIO_STATUS_SLAVE_READY))
#define SDIO_STATIC_CMD_FLAGS           ((uint32_t)(SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_CMD_ERR))
#define SDIO_STATIC_DATA_FLAGS           ((uint32_t)SDIO_STATUS_EOT | SDIO_STATUS_ERR | SDIO_STATUS_DATA_ERR)

#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00010000)
#define SDIO_CMDWrongDirection          BIT(17)
#define SDIO_CMDBusyTimeout             BIT(18)
#define SDIO_DATRunning                 BITS(24,29)
#define SDIO_DATATIMEOUT                BIT(24)
#define SDIO_FIFO_Address               ((uint32_t)0x40018080)

#define SDIO_ERROR_NONE                     ((uint32_t)0x00000000U)   /*!< No error                                                      */
#define SDIO_ERROR_CMD_CRC_FAIL             ((uint32_t)0x00000001U)   /*!< Command response received (but CRC check failed)              */
#define SDIO_ERROR_DATA_CRC_FAIL            ((uint32_t)0x00000002U)   /*!< Data block sent/received (CRC check failed)                   */
#define SDIO_ERROR_CMD_RSP_TIMEOUT          ((uint32_t)0x00000004U)   /*!< Command response timeout                                      */
#define SDIO_ERROR_DATA_TIMEOUT             ((uint32_t)0x00000008U)   /*!< Data timeout                                                  */
#define SDIO_ERROR_TX_UNDERRUN              ((uint32_t)0x00000010U)   /*!< Transmit FIFO underrun                                        */
#define SDIO_ERROR_RX_OVERRUN               ((uint32_t)0x00000020U)   /*!< Receive FIFO overrun                                          */
#define SDIO_ERROR_ADDR_MISALIGNED          ((uint32_t)0x00000040U)   /*!< Misaligned address                                            */
#define SDIO_ERROR_BLOCK_LEN_ERR            ((uint32_t)0x00000080U)   /*!< Transferred block length is not allowed for the card or the
                                                                            number of transferred bytes does not match the block length   */
#define SDIO_ERROR_ERASE_SEQ_ERR            ((uint32_t)0x00000100U)   /*!< An error in the sequence of erase command occurs              */
#define SDIO_ERROR_BAD_ERASE_PARAM          ((uint32_t)0x00000200U)   /*!< An invalid selection for erase groups                         */
#define SDIO_ERROR_WRITE_PROT_VIOLATION     ((uint32_t)0x00000400U)   /*!< Attempt to program a write protect block                      */
#define SDIO_ERROR_LOCK_UNLOCK_FAILED       ((uint32_t)0x00000800U)   /*!< Sequence or password error has been detected in unlock
                                                                            command or if there was an attempt to access a locked card    */
#define SDIO_ERROR_COM_CRC_FAILED           ((uint32_t)0x00001000U)   /*!< CRC check of the previous command failed                      */
#define SDIO_ERROR_ILLEGAL_CMD              ((uint32_t)0x00002000U)   /*!< Command is not legal for the card state                       */
#define SDIO_ERROR_CARD_ECC_FAILED          ((uint32_t)0x00004000U)   /*!< Card internal ECC was applied but failed to correct the data  */
#define SDIO_ERROR_CC_ERR                   ((uint32_t)0x00008000U)   /*!< Internal card controller error                                */
#define SDIO_ERROR_GENERAL_UNKNOWN_ERR      ((uint32_t)0x00010000U)   /*!< General or unknown error                                      */
#define SDIO_ERROR_STREAM_READ_UNDERRUN     ((uint32_t)0x00020000U)   /*!< The card could not sustain data reading in stream rmode       */
#define SDIO_ERROR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00040000U)   /*!< The card could not sustain data programming in stream mode    */
#define SDIO_ERROR_CID_CSD_OVERWRITE        ((uint32_t)0x00080000U)   /*!< CID/CSD overwrite error                                       */
#define SDIO_ERROR_WP_ERASE_SKIP            ((uint32_t)0x00100000U)   /*!< Only partial address space was erased                         */
#define SDIO_ERROR_CARD_ECC_DISABLED        ((uint32_t)0x00200000U)   /*!< Command has been executed without using internal ECC          */
#define SDIO_ERROR_ERASE_RESET              ((uint32_t)0x00400000U)   /*!< Erase sequence was cleared before executing because an out
                                                                            of erase sequence command was received                        */
#define SDIO_ERROR_AKE_SEQ_ERR              ((uint32_t)0x00800000U)   /*!< Error in sequence of authentication                           */
#define SDIO_ERROR_INVALID_VOLTRANGE        ((uint32_t)0x01000000U)   /*!< Error in case of invalid voltage range                        */
#define SDIO_ERROR_ADDR_OUT_OF_RANGE        ((uint32_t)0x02000000U)   /*!< Error when addressed block is out of range                    */
#define SDIO_ERROR_REQUEST_NOT_APPLICABLE   ((uint32_t)0x04000000U)   /*!< Error when command request is not applicable                  */
#define SDIO_ERROR_INVALID_PARAMETER        ((uint32_t)0x08000000U)   /*!< the used parameter is not valid                               */
#define SDIO_ERROR_UNSUPPORTED_FEATURE      ((uint32_t)0x10000000U)   /*!< Error when feature is not insupported                         */
#define SDIO_ERROR_BUSY                     ((uint32_t)0x20000000U)   /*!< Error when transfer process is busy                           */
#define SDIO_ERROR_DMA                      ((uint32_t)0x40000000U)   /*!< Error while DMA transfer                                      */
#define SDIO_ERROR_TIMEOUT                  ((uint32_t)0x80000000U)   /*!< Timeout error       

/**
  * @brief  Masks for R6 Response
  */
#define SDIO_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000U)
#define SDIO_R6_ILLEGAL_CMD               ((uint32_t)0x00004000U)
#define SDIO_R6_COM_CRC_FAILED            ((uint32_t)0x00008000U)

#define SDIO_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000U)
#define SDIO_HIGH_CAPACITY                ((uint32_t)0x40000000U)
#define SDIO_STD_CAPACITY                 ((uint32_t)0x00000000U)
#define SDIO_CHECK_PATTERN                ((uint32_t)0x000001AAU)
#define SD_SWITCH_1_8V_CAPACITY            ((uint32_t)0x01000000U)
#define SDIO_DDR50_SWITCH_PATTERN         ((uint32_t)0x80FFFF04U)
#define SDIO_SDR104_SWITCH_PATTERN        ((uint32_t)0x80FF1F03U)
#define SDIO_SDR50_SWITCH_PATTERN         ((uint32_t)0x80FF1F02U)
#define SDIO_SDR25_SWITCH_PATTERN         ((uint32_t)0x80FFFF01U)

/**
  * @brief  Masks for errors Card Status R1 (OCR Register)
  */
#define SDIO_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000U)
#define SDIO_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000U)
#define SDIO_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000U)
#define SDIO_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000U)
#define SDIO_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000U)
#define SDIO_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000U)
#define SDIO_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000U)
#define SDIO_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000U)
#define SDIO_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000U)
#define SDIO_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000U)
#define SDIO_OCR_CC_ERROR                 ((uint32_t)0x00100000U)
#define SDIO_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000U)
#define SDIO_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000U)
#define SDIO_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000U)
#define SDIO_OCR_CID_CSD_OVERWRITE        ((uint32_t)0x00010000U)
#define SDIO_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000U)
#define SDIO_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000U)
#define SDIO_OCR_ERASE_RESET              ((uint32_t)0x00002000U)
#define SDIO_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008U)
#define SDIO_OCR_ERRORBITS                ((uint32_t)0xFDFFE008U)

/**
  * @brief SDIO Commands Index
  */
#define SDMMC_CMD_GO_IDLE_STATE                       (0) << 8
#define SDMMC_CMD_SEND_OP_COND                        (1) << 8
#define SDMMC_CMD_ALL_SEND_CID                        (2) << 8
#define SDMMC_CMD_SET_REL_ADDR                        (3) << 8     /*!< SDIO_SEND_REL_ADDR for SD Card */
#define SDMMC_CMD_SET_DSR                             (4) << 8
#define SDMMC_CMD_SDIO_SEN_OP_COND                    (5) << 8
#define SDMMC_CMD_HS_SWITCH                           (6) << 8
#define SDMMC_CMD_SEL_DESEL_CARD                      (7) << 8
#define SDMMC_CMD_HS_SEND_EXT_CSD                     (8) << 8
#define SDMMC_CMD_SEND_CSD                            (9) << 8
#define SDMMC_CMD_SEND_CID                            (10) << 8
#define SDMMC_CMD_READ_DAT_UNTIL_STOP                 (11) << 8    /*!< SD Card doesn't support it */
#define SDMMC_CMD_STOP_TRANSMISSION                   (12) << 8
#define SDMMC_CMD_SEND_STATUS                         (13) << 8
#define SDMMC_CMD_HS_BUSTEST_READ                     (14) << 8
#define SDMMC_CMD_GO_INACTIVE_STATE                   (15) << 8
#define SDMMC_CMD_SET_BLOCKLEN                        (16) << 8
#define SDMMC_CMD_READ_SINGLE_BLOCK                   (17) << 8
#define SDMMC_CMD_READ_MULT_BLOCK                     (18) << 8
#define SDMMC_CMD_HS_BUSTEST_WRITE                    (19) << 8
#define SDMMC_CMD_WRITE_DAT_UNTIL_STOP                (20) << 8    /*!< SD Card doesn't support it */
#define SDMMC_CMD_SET_BLOCK_COUNT                     (23) << 8    /*!< SD Card doesn't support it */
#define SDMMC_CMD_WRITE_SINGLE_BLOCK                  (24) << 8
#define SDMMC_CMD_WRITE_MULT_BLOCK                    (25) << 8
#define SDMMC_CMD_PROG_CID                            (26) << 8
#define SDMMC_CMD_PROG_CSD                            (27) << 8
#define SDMMC_CMD_SET_WRITE_PROT                      (28) << 8
#define SDMMC_CMD_CLR_WRITE_PROT                      (29) << 8
#define SDMMC_CMD_SEND_WRITE_PROT                     (30) << 8
#define SDMMC_CMD_SD_ERASE_GRP_START                  (32) << 8    /*!< To set the address of the first write                                                                     block to be erased. (For SD card only) */
#define SDMMC_CMD_SD_ERASE_GRP_END                    (33) << 8    /*!< To set the address of the last write block of the                                                                     continuous range to be erased. (For SD card only) */
#define SDMMC_CMD_ERASE_GRP_START                     (35) << 8    /*!< To set the address of the first write block to be erased.                                                                    (For MMC card only spec 3.31) */
#define SDMMC_CMD_ERASE_GRP_END                       (36) << 8    /*!< To set the address of the last write block of the                                                                     continuous range to be erased. (For MMC card only spec 3.31) */

#define SDMMC_CMD_ERASE                               (38) << 8
#define SDMMC_CMD_FAST_IO                             (39) << 8    /*!< SD Card doesn't support it */
#define SDMMC_CMD_GO_IRQ_STATE                        (40) << 8    /*!< SD Card doesn't support it */
#define SDMMC_CMD_LOCK_UNLOCK                         (42) << 8
#define SDMMC_CMD_APP_CMD                             (55) << 8
#define SDMMC_CMD_GEN_CMD                             (56) << 8
#define SDMMC_CMD_NO_CMD                              (64) << 8

/**
  * @brief Following commands are SD Card Specific commands.
  *        SDIO_APP_CMD should be sent before sending these commands.
  */
#define SDMMC_CMD_APP_SD_SET_BUSWIDTH                 (6) << 8     /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_STATUS                       (13) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        (22) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_OP_COND                      (41) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SET_CLR_CARD_DETECT          (42) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SEND_SCR                     (51) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SDIO_RW_DIRECT                      (52) << 8    /*!< For SD I/O Card only */
#define SDMMC_CMD_SDIO_RW_EXTENDED                    (53) << 8    /*!< For SD I/O Card only */

/**
  * @brief Following commands are SD Card Specific security commands.
  *        SDIO_APP_CMD should be sent before sending these commands.
  */
#define SDMMC_CMD_SD_APP_GET_MKB                      (43) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_GET_MID                      (44) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SET_CER_RN1                  (45) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_GET_CER_RN2                  (46) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SET_CER_RES2                 (47) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_GET_CER_RES1                 (48) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   (18) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  (25) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SECURE_ERASE                 (38) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_CHANGE_SECURE_AREA           (49) << 8    /*!< For SD Card only */
#define SDMMC_CMD_SD_APP_SECURE_WRITE_MKB             (48) << 8    /*!< For SD Card only */

/**
  * @brief  SDIO proto mode
  */
typedef enum
{
    SDIO_ARG_SINGLE_LINE = 0,
    SDIO_ARG_QUAD_LINE = 2,
    SDIO_ARG_OCTOL_LINE = 3,
} SDIO_ArgProtoMode;

#define SDIO_SEND_IF_COND               ((uint32_t)0x00000008)

#define SDIO_SDR_MODE 0
#define SDIO_DDR_MODE 1

#define SDIO_GET_STATUS(__SDIO__)   __SDIO__->STATUS
#define SDIO_GET_STATU_FLAG(__SDIO__,__FLAG__) (((__SDIO__)->STATUS & (__FLAG__)) == (__FLAG__))
#define SDIO_GET_IP_FLAG(__SDIO__,__IP__)  (((__SDIO__)->IP & (__IP__)) == (__IP__))

#define DMA_LIST_INIT(dma_cfg_list, p_addr, blk_size, dma_cfg, send_channel) do {  \
    (dma_cfg_list).addr = (p_addr);                                                 \
    (dma_cfg_list).block_size = (blk_size);                                         \
    (dma_cfg_list).dma_config = (dma_cfg);                                          \
    (dma_cfg_list).channel = (send_channel);                                        \
    (dma_cfg_list).link_list = NULL;                                                \
} while(0)

#define DMA_LINT_NODE_INIT(node, p_addr, blk_size, dat_size, cfg_addr) do { \
    (node).addr = (p_addr);                                                 \
    (node).block_size = (blk_size);                                         \
    (node).data_size = (dat_size);                                          \
    (node).config_addr = (cfg_addr);                                        \
    (node).next = NULL;                                                     \
} while (0)

#define DMA_LIST_INSERT_NODE(dma_cfg_list, node) do {                              \
    if ((dma_cfg_list).link_list == NULL) {                                       \
        (dma_cfg_list).link_list = &(node);                                       \
    } else {                                                                      \
        SDIO_DmaLinkListNodeTypeDef *current = (dma_cfg_list).link_list;          \
        while (current->next != NULL) {                                           \
            current = current->next;                                              \
        }                                                                         \
        current->next = &(node);                                                  \
    }                                                                             \
} while (0)

#define SDIO_RX_SADDR_OFFSET                                           0x0 /*!< uDMA RX SDIO buffer base address */
#define SDIO_RX_SIZE_OFFSET                                            0x4 /*!< uDMA RX SDIO buffer size configuration register */
#define SDIO_RX_CFG_OFFSET                                             0x8 /*!< uDMA RX SDIO stream configuration register */
#define SDIO_CR_OFFSET                                                 0xc /*!< SDIO control register */
#define SDIO_TX_SADDR_OFFSET                                           0x10 /*!< uDMA TX SDIO buffer base address configuration register */
#define SDIO_TX_SIZE_OFFSET                                            0x14 /*!< uDMA TX SDIO buffer size configuration register */
#define SDIO_TX_CFG_OFFSET                                             0x18 /*!< uDMA TX SDIO stream configuration register */
#define SDIO_VERSION_OFFSET                                            0x1c /*!< SDIO version 1.2.0 */
#define SDIO_CMD_OP_OFFSET                                             0x20 /*!< SDIO command */
#define SDIO_CMD_ARG_OFFSET                                            0x24 /*!< SDIO argument */
#define SDIO_DATA_SETUP_OFFSET                                         0x28 /*!< SDIO Data transfer setup */
#define SDIO_START_OFFSET                                              0x2c /*!< SDIO Start */
#define SDIO_RSP0_OFFSET                                               0x30 /*!< SDIO Response 0 */
#define SDIO_RSP1_OFFSET                                               0x34 /*!< SDIO Response 1 */
#define SDIO_RSP2_OFFSET                                               0x38 /*!< SDIO Response 2 */
#define SDIO_RSP3_OFFSET                                               0x3c /*!< SDIO Response 3 */
#define SDIO_CLK_DIV_OFFSET                                            0x40 /*!< SDIO Clock Divider */
#define SDIO_STATUS_OFFSET                                             0x44 /*!< SDIO STATUS register */
#define SDIO_STOP_CMD_OP_OFFSET                                        0x48 /*!< SDIO STOP command op */
#define SDIO_STOP_CMD_ARG_OFFSET                                       0x4c /*!< SDIO STOP command arg */
#define SDIO_DATA_TIMEOUT_OFFSET                                       0x50 /*!< SDIO data timeout delay counter register */
#define SDIO_CMD_POWERUP_OFFSET                                        0x54 /*!< SDIO command power up counter config */
#define SDIO_CMD_WAIT_RSP_OFFSET                                       0x58 /*!< SDIO wait respone counter congfig */
#define SDIO_CMD_WAIT_EOT_OFFSET                                       0x5c /*!< SDIO wait eot counter congfig */
#define SDIO_TX_DATA_OFFSET                                            0x60 /*!< SDIO send data */
#define SDIO_RX_DATA_OFFSET                                            0x64 /*!< SDIO receive data */
#define SDIO_TX_MARK_OFFSET                                            0x68 /*!< SDIO Tx FIFO watermark */
#define SDIO_RX_MARK_OFFSET                                            0x6c /*!< SDIO Rx FIFO watermark */
#define SDIO_IP_OFFSET                                                 0x70 /*!< SDIO interrupt pending */
#define SDIO_IE_OFFSET                                                 0x74 /*!< SDIO interrupt enable */
#define SDIO_SAMPLE_DDR_OFFSET                                         0x78 /*!< SDIO ddr sample select */
#define SDIO_RX_SADDR_HI_OFFSET                                        0x7c /*!< uDMA RX SDIO buffer base high address */
#define SDIO_TX_SADDR_HI_OFFSET                                        0x80 /*!< uDMA TX SDIO buffer base high address */
#define SDIO_DATA_TX_DELAY_OFFSET                                      0x84 /*!< SDIO data tx delay counter register */
#define SDIO_DATA_CRC_TOKEN_OFFSET                                     0x88 /*!< SDIO crc token timeout delay counter register */
#define SDIO_CRC_VALUE_OFFSET                                          0x8c /*!< SDIO cmd crc value  */
#define SDIO_STATUS1_OFFSET                                            0x90 /*!< SDIO STATUS1 register */
#define SDIO_STOP_OFFSET                                               0x94 /*!< SDIO Start */
#define SDIO_PAD_CTRL_OFFSET                                           0x98 /*!< SDIO pad pue control */
#define SDIO_STATUS2_OFFSET                                            0x9c /*!< SDIO STATUS2 register */
#define SDIO_STOP_RSP0_OFFSET                                          0xa0 /*!< SDIO Stop Response 0 */
#define SDIO_STOP_RSP1_OFFSET                                          0xa4 /*!< SDIO Stop Response 1 */
#define SDIO_STOP_RSP2_OFFSET                                          0xa8 /*!< SDIO Stop Response 2 */
#define SDIO_STOP_RSP3_OFFSET                                          0xac /*!< SDIO Stop Response 3 */
#define SDIO_RCA_OFFSET                                                0xb0 /*!< SDIO RCA */
#define SDIO_STATUS3_OFFSET                                            0xb4 /*!< SDIO STATUS3 register */
#define SDIO_SLAVE_DATA_SETUP_OFFSET                                   0xb8 /*!< SDIO_SLAVE_DATA_SETUP register */

#define SDIO_RX_SADDR_OFFSET                                           0x0 /*!< uDMA RX SDIO buffer base address Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address. */
#define SDIO_RX_SIZE_OFFSET                                            0x4 /*!< uDMA RX SDIO buffer size configuration register. Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address.  */
#define SDIO_RX_CFG_OFFSET                                             0x8 /*!< uDMA RX SDIO stream configuration register. Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address. Linked list DMA does not support continuous mode.  */
#define SDIO_CR_OFFSET                                                 0xc /*!< SDIO control register */
#define SDIO_TX_SADDR_OFFSET                                           0x10 /*!< uDMA TX SDIO buffer base address configuration register. Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address.  */
#define SDIO_TX_SIZE_OFFSET                                            0x14 /*!< uDMA TX SDIO buffer size configuration register. Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address.  */
#define SDIO_TX_CFG_OFFSET                                             0x18 /*!< uDMA TX SDIO stream configuration register. Note: that if DMA is in linked list mode. this register is configured for the first time in the linked list. Afterward the DMA will read the information from its own linked list address . Linked list DMA does not support continuous mode. */
#define SDIO_VERSION_OFFSET                                            0x1c /*!< SDIO version 1.5.0 */
#define SDIO_CMD_OP_OFFSET                                             0x20 /*!< SDIO command */
#define SDIO_CMD_ARG_OFFSET                                            0x24 /*!< SDIO argument */
#define SDIO_DATA_SETUP_OFFSET                                         0x28 /*!< SDIO Data transfer setup */
#define SDIO_START_OFFSET                                              0x2c /*!< SDIO Start */
#define SDIO_RSP0_OFFSET                                               0x30 /*!< SDIO Response 0 */
#define SDIO_RSP1_OFFSET                                               0x34 /*!< SDIO Response 1 */
#define SDIO_RSP2_OFFSET                                               0x38 /*!< SDIO Response 2 */
#define SDIO_RSP3_OFFSET                                               0x3c /*!< SDIO Response 3 */
#define SDIO_CLK_DIV_OFFSET                                            0x40 /*!< SDIO Clock Divider */
#define SDIO_STATUS_OFFSET                                             0x44 /*!< SDIO STATUS register */
#define SDIO_STOP_CMD_OP_OFFSET                                        0x48 /*!< SDIO STOP command op */
#define SDIO_STOP_CMD_ARG_OFFSET                                       0x4c /*!< SDIO STOP command arg */
#define SDIO_DATA_TIMEOUT_OFFSET                                       0x50 /*!< SDIO data timeout delay counter register */
#define SDIO_CMD_POWERUP_OFFSET                                        0x54 /*!< SDIO command power up counter config */
#define SDIO_CMD_WAIT_RSP_OFFSET                                       0x58 /*!< SDIO wait respone counter congfig */
#define SDIO_CMD_WAIT_EOT_OFFSET                                       0x5c /*!< SDIO wait eot counter congfig */
#define SDIO_TX_DATA_OFFSET                                            0x60 /*!< SDIO send data */
#define SDIO_RX_DATA_OFFSET                                            0x64 /*!< SDIO receive data */
#define SDIO_TX_MARK_OFFSET                                            0x68 /*!< SDIO Tx FIFO watermark */
#define SDIO_RX_MARK_OFFSET                                            0x6c /*!< SDIO Rx FIFO watermark */
#define SDIO_IP_OFFSET                                                 0x70 /*!< SDIO interrupt pending */
#define SDIO_IE_OFFSET                                                 0x74 /*!< SDIO interrupt enable */
#define SDIO_SAMPLE_DDR_OFFSET                                         0x78 /*!< SDIO ddr sample select */
#define SDIO_RX_SADDR_HI_OFFSET                                        0x7c /*!< uDMA RX SDIO buffer base high address */
#define SDIO_TX_SADDR_HI_OFFSET                                        0x80 /*!< uDMA TX SDIO buffer base high address */
#define SDIO_DATA_TX_DELAY_OFFSET                                      0x84 /*!< SDIO data tx delay counter register */
#define SDIO_DATA_CRC_TOKEN_OFFSET                                     0x88 /*!< SDIO crc token timeout delay counter register */
#define SDIO_CRC_VALUE_OFFSET                                          0x8c /*!< SDIO cmd crc value  */
#define SDIO_STATUS1_OFFSET                                            0x90 /*!< SDIO STATUS1 register */
#define SDIO_STOP_OFFSET                                               0x94 /*!< SDIO Start */
#define SDIO_PAD_CTRL_OFFSET                                           0x98 /*!< SDIO pad pue control */
#define SDIO_STOP_RSP0_OFFSET                                          0xa0 /*!< SDIO Stop Response 0 */
#define SDIO_STOP_RSP1_OFFSET                                          0xa4 /*!< SDIO Stop Response 1 */
#define SDIO_STOP_RSP2_OFFSET                                          0xa8 /*!< SDIO Stop Response 2 */
#define SDIO_STOP_RSP3_OFFSET                                          0xac /*!< SDIO Stop Response 3 */

 /* ===== SDIO RX_SADDR Register definition ===== */
#define SDIO_RX_SADDR_ADDRESS_MASK                 BITS(0,31)                                   /*!< SDIO RX SADDR: ADDRESS Bit Mask */  
#define SDIO_RX_SADDR_ADDRESS_OFS                  0U                                          /*!< SDIO RX SADDR: ADDRESS Bit Offset */
#define SDIO_RX_SADDR_ADDRESS(regval)              (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO RX SADDR: ADDRESS Bit Value */  
 
 /* ===== SDIO RX_SIZE Register definition ===== */
#define SDIO_RX_SIZE_NUM_MASK                     BITS(0,24)                                   /*!< SDIO RX SIZE: NUM Bit Mask */  
#define SDIO_RX_SIZE_NUM_OFS                      0U                                          /*!< SDIO RX SIZE: NUM Bit Offset */
#define SDIO_RX_SIZE_NUM(regval)                  (BITS(0,24) & ((uint32_t)(regval) << 0))        /*!< SDIO RX SIZE: NUM Bit Value */  
 
 /* ===== SDIO RX_CFG Register definition ===== */
#define SDIO_RX_CFG_CONTINUOUS                   BIT(0)                                      /*!< RX channel continuous mode bitfield:  -1'b0: disable -1'b1: enable */
#define SDIO_RX_CFG_CONTINUOUS_OFS               0U                                          /*!< SDIO RX CFG: CONTINUOUS Bit Offset */
#define SDIO_RX_CFG_CONTINUOUS_VAL(regval)           (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO RX CFG: CONTINUOUS Bit Value */  
#define SDIO_RX_CFG_CONTINUOUS_DISABLE                0x0UL                                              /*!< DISABLE */
#define SDIO_RX_CFG_CONTINUOUS_ENABLE                 BIT(0)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO rx_cfg continuous bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid continuous bit.
  * \retval 1 This is a valid continuous bit.
  */
#define IS_SDIO_RX_CFG_CONTINUOUS(regval)             (\
                                       ((regval) == SDIO_RX_CFG_CONTINUOUS_DISABLE             ) || \
                                       ((regval) == SDIO_RX_CFG_CONTINUOUS_ENABLE              )  \
                                                 )

#define SDIO_RX_CFG_DATASIZE_MASK                BITS(1,2)                                   /*!< SDIO RX CFG: DATASIZE Bit Mask */  
#define SDIO_RX_CFG_DATASIZE_OFS                 1U                                          /*!< SDIO RX CFG: DATASIZE Bit Offset */
#define SDIO_RX_CFG_DATASIZE(regval)             (BITS(1,2) & ((uint32_t)(regval) << 1))        /*!< SDIO RX CFG: DATASIZE Bit Value */  
#define SDIO_RX_CFG_DATASIZE_BYTE                     SDIO_RX_CFG_DATASIZE(0)                                                /*!< BYTE */
#define SDIO_RX_CFG_DATASIZE_HALFWORD                 SDIO_RX_CFG_DATASIZE(1)                                                /*!< HALFWORD */
#define SDIO_RX_CFG_DATASIZE_WORD                     SDIO_RX_CFG_DATASIZE(2)                                                /*!< WORD */
#define SDIO_RX_CFG_DATASIZE_RESERVED                 SDIO_RX_CFG_DATASIZE(3)                                                /*!< RESERVED */

/**
  * \brief Check the SDIO rx_cfg datasize bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid datasize bits.
  * \retval 1 This is a valid datasize bits.
  */
#define IS_SDIO_RX_CFG_DATASIZE(regval)               (\
                                         ((regval) == SDIO_RX_CFG_DATASIZE_BYTE                ) || \
                                         ((regval) == SDIO_RX_CFG_DATASIZE_HALFWORD            ) || \
                                         ((regval) == SDIO_RX_CFG_DATASIZE_WORD                ) || \
                                         ((regval) == SDIO_RX_CFG_DATASIZE_RESERVED            )  \
                                                 )

#define SDIO_RX_CFG_EN                           BIT(4)                                      /*!< RX channel enable and start transfer bitfield: -1'b0: disable -1'b1: enable and start the transfer */
#define SDIO_RX_CFG_EN_OFS                       4U                                          /*!< SDIO RX CFG: EN Bit Offset */
#define SDIO_RX_CFG_EN_VAL(regval)                   (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO RX CFG: EN Bit Value */  
#define SDIO_RX_CFG_EN_DISABLE                        0x0UL                                              /*!< DISABLE */
#define SDIO_RX_CFG_EN_ENABLE                         BIT(4)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO rx_cfg en bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid en bit.
  * \retval 1 This is a valid en bit.
  */
#define IS_SDIO_RX_CFG_EN(regval)                     (\
                                               ((regval) == SDIO_RX_CFG_EN_DISABLE             ) || \
                                               ((regval) == SDIO_RX_CFG_EN_ENABLE              )  \
                                                 )

#define SDIO_RX_CFG_PENDING                      BIT(5)                                      /*!< RX transfer pending in queue status flag: -1'b0: disable: no pending - no pending transfer in the queue -1'b1: enable: pending - pending transfer in the queue */
#define SDIO_RX_CFG_PENDING_DISABLE                   ((uint32_t)(0) << 5)                                                /*!< DISABLE */
#define SDIO_RX_CFG_PENDING_ENABLE                    ((uint32_t)(1) << 5)                                                /*!< ENABLE */

/**
  * \brief Check the SDIO rx_cfg pending bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid pending bit.
  * \retval 1 This is a valid pending bit.
  */
#define IS_SDIO_RX_CFG_PENDING(regval)                (\
                                          ((regval) == SDIO_RX_CFG_PENDING_DISABLE             ) || \
                                          ((regval) == SDIO_RX_CFG_PENDING_ENABLE              )  \
                                                 )

#define SDIO_RX_CFG_CLR                          BIT(6)                                      /*!< RX channel clear and stop transfer bitfield: -1'b0: disable -1'b1: enable: stop and clear - stop and clear the on-going transfer Note:The clear operation is not performed immediately.  You need to wait until TXEN is pulled down to clear it */
#define SDIO_RX_CFG_CLR_OFS                      6U                                          /*!< SDIO RX CFG: CLR Bit Offset */
#define SDIO_RX_CFG_CLR_VAL(regval)                  (BIT(6) & ((uint32_t)(regval) << 6))        /*!< SDIO RX CFG: CLR Bit Value */  

/**
  * \brief Check the SDIO rx_cfg clr bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid clr bit.
  * \retval 1 This is a valid clr bit.
  */
#define IS_SDIO_RX_CFG_CLR(regval)                    (\
                                                 )

 /* ===== SDIO CR Register definition ===== */
#define SDIO_CR_DMA                          BIT(0)                                      /*!< uDMA mode enable -1'b0: disable -1'b1: enable */
#define SDIO_CR_DMA_OFS                      0U                                          /*!< SDIO CR: DMA Bit Offset */
#define SDIO_CR_DMA_VAL(regval)                  (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO CR: DMA Bit Value */  
#define SDIO_CR_DMA_DISABLE                       0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_DMA_ENABLE                        BIT(0)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr dma bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid dma bit.
  * \retval 1 This is a valid dma bit.
  */
#define IS_SDIO_CR_DMA(regval)                    (\
                                              ((regval) == SDIO_CR_DMA_DISABLE             ) || \
                                              ((regval) == SDIO_CR_DMA_ENABLE              )  \
                                                 )

#define SDIO_CR_DDR                          BIT(1)                                      /*!< Double Data Rate (DDR) mode enable -1'b0: disable -1'b1: enable */
#define SDIO_CR_DDR_OFS                      1U                                          /*!< SDIO CR: DDR Bit Offset */
#define SDIO_CR_DDR_VAL(regval)                  (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO CR: DDR Bit Value */  
#define SDIO_CR_DDR_DISABLE                       0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_DDR_ENABLE                        BIT(1)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr ddr bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid ddr bit.
  * \retval 1 This is a valid ddr bit.
  */
#define IS_SDIO_CR_DDR(regval)                    (\
                                              ((regval) == SDIO_CR_DDR_DISABLE             ) || \
                                              ((regval) == SDIO_CR_DDR_ENABLE              )  \
                                                 )

#define SDIO_CR_AUTO_CMD12                   BIT(2)                                      /*!< This bit field is used to enable automatic transmission of cmd12 mode -1'b0: disable: turn off auto send cmd12 mode -1'b1: enable: Turn on auto send cmd12 mode */
#define SDIO_CR_AUTO_CMD12_OFS               2U                                          /*!< SDIO CR: AUTO_CMD12 Bit Offset */
#define SDIO_CR_AUTO_CMD12_VAL(regval)           (BIT(2) & ((uint32_t)(regval) << 2))        /*!< SDIO CR: AUTO_CMD12 Bit Value */  
#define SDIO_CR_AUTO_CMD12_DISABLE                0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_AUTO_CMD12_ENABLE                 BIT(2)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr auto_cmd12 bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid auto_cmd12 bit.
  * \retval 1 This is a valid auto_cmd12 bit.
  */
#define IS_SDIO_CR_AUTO_CMD12(regval)             (\
                                       ((regval) == SDIO_CR_AUTO_CMD12_DISABLE             ) || \
                                       ((regval) == SDIO_CR_AUTO_CMD12_ENABLE              )  \
                                                 )

#define SDIO_CR_BLOCK                        BIT(3)                                      /*!< This bit field is used to enable block_num mode; -1'b0: disable: ignore block_num configuration, sending and receiving data enters the unlimited sending and receiving state, and can't stop until sending cmd12. -1'b1: enable: The number of blocks sent by SDIO controller is determined by block_num decision */
#define SDIO_CR_BLOCK_OFS                    3U                                          /*!< SDIO CR: BLOCK Bit Offset */
#define SDIO_CR_BLOCK_VAL(regval)                (BIT(3) & ((uint32_t)(regval) << 3))        /*!< SDIO CR: BLOCK Bit Value */  
#define SDIO_CR_BLOCK_DISABLE                     0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_BLOCK_ENABLE                      BIT(3)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr block bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid block bit.
  * \retval 1 This is a valid block bit.
  */
#define IS_SDIO_CR_BLOCK(regval)                  (\
                                            ((regval) == SDIO_CR_BLOCK_DISABLE             ) || \
                                            ((regval) == SDIO_CR_BLOCK_ENABLE              )  \
                                                 )

#define SDIO_CR_CLK_STOP                     BIT(5)                                      /*!< If this bit domain is turned on, the SD card clock will be stopped each time the transfer is completed : -1'b0: disable: always open sdclk -1'b1: enable: stop sdclk */
#define SDIO_CR_CLK_STOP_OFS                 5U                                          /*!< SDIO CR: CLK_STOP Bit Offset */
#define SDIO_CR_CLK_STOP_VAL(regval)             (BIT(5) & ((uint32_t)(regval) << 5))        /*!< SDIO CR: CLK_STOP Bit Value */  
#define SDIO_CR_CLK_STOP_DISABLE                  0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_CLK_STOP_ENABLE                   BIT(5)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr clk_stop bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid clk_stop bit.
  * \retval 1 This is a valid clk_stop bit.
  */
#define IS_SDIO_CR_CLK_STOP(regval)               (\
                                         ((regval) == SDIO_CR_CLK_STOP_DISABLE             ) || \
                                         ((regval) == SDIO_CR_CLK_STOP_ENABLE              )  \
                                                 )

#define SDIO_CR_HIGH_WIDTH_MODE              BIT(6)                                      /*!< High bandwidth mode is turned on, the data will be four bytes bandwidth throughput data, when there is no 4 bytes of aligned data, the data will only send and receive the number of valid bytes -1'b0: disable -1'b1: enable */
#define SDIO_CR_HIGH_WIDTH_MODE_OFS          6U                                          /*!< SDIO CR: HIGH_WIDTH_MODE Bit Offset */
#define SDIO_CR_HIGH_WIDTH_MODE_VAL(regval)      (BIT(6) & ((uint32_t)(regval) << 6))        /*!< SDIO CR: HIGH_WIDTH_MODE Bit Value */  
#define SDIO_CR_HIGH_WIDTH_MODE_DISABLE             0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_HIGH_WIDTH_MODE_ENABLE             BIT(6)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr high_width_mode bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid high_width_mode bit.
  * \retval 1 This is a valid high_width_mode bit.
  */
#define IS_SDIO_CR_HIGH_WIDTH_MODE(regval)        (\
                                  ((regval) == SDIO_CR_HIGH_WIDTH_MODE_DISABLE             ) || \
                                  ((regval) == SDIO_CR_HIGH_WIDTH_MODE_ENABLE              )  \
                                                 )

#define SDIO_CR_DATA_CRC_CHECK               BIT(7)                                      /*!< data crc check enable -1'b0: disable -1'b1: enable */
#define SDIO_CR_DATA_CRC_CHECK_OFS           7U                                          /*!< SDIO CR: DATA_CRC_CHECK Bit Offset */
#define SDIO_CR_DATA_CRC_CHECK_VAL(regval)       (BIT(7) & ((uint32_t)(regval) << 7))        /*!< SDIO CR: DATA_CRC_CHECK Bit Value */  
#define SDIO_CR_DATA_CRC_CHECK_DISABLE             0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_DATA_CRC_CHECK_ENABLE             BIT(7)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr data_crc_check bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid data_crc_check bit.
  * \retval 1 This is a valid data_crc_check bit.
  */
#define IS_SDIO_CR_DATA_CRC_CHECK(regval)         (\
                                   ((regval) == SDIO_CR_DATA_CRC_CHECK_DISABLE             ) || \
                                   ((regval) == SDIO_CR_DATA_CRC_CHECK_ENABLE              )  \
                                                 )

#define SDIO_CR_DATA_BUS_SPEED               BIT(8)                                      /*!< data bus speed select -1'b0: disable -1'b1: enable */
#define SDIO_CR_DATA_BUS_SPEED_OFS           8U                                          /*!< SDIO CR: DATA_BUS_SPEED Bit Offset */
#define SDIO_CR_DATA_BUS_SPEED_VAL(regval)       (BIT(8) & ((uint32_t)(regval) << 8))        /*!< SDIO CR: DATA_BUS_SPEED Bit Value */  
#define SDIO_CR_DATA_BUS_SPEED_DISABLE             0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_DATA_BUS_SPEED_ENABLE             BIT(8)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr data_bus_speed bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid data_bus_speed bit.
  * \retval 1 This is a valid data_bus_speed bit.
  */
#define IS_SDIO_CR_DATA_BUS_SPEED(regval)         (\
                                   ((regval) == SDIO_CR_DATA_BUS_SPEED_DISABLE             ) || \
                                   ((regval) == SDIO_CR_DATA_BUS_SPEED_ENABLE              )  \
                                                 )

#define SDIO_CR_IRQ_PERIOD_CHECK             BIT(9)                                      /*!< interrupt period check enable -1'b0: disable -1'b1: enable */
#define SDIO_CR_IRQ_PERIOD_CHECK_OFS         9U                                          /*!< SDIO CR: IRQ_PERIOD_CHECK Bit Offset */
#define SDIO_CR_IRQ_PERIOD_CHECK_VAL(regval)     (BIT(9) & ((uint32_t)(regval) << 9))        /*!< SDIO CR: IRQ_PERIOD_CHECK Bit Value */  
#define SDIO_CR_IRQ_PERIOD_CHECK_DISABLE             0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_IRQ_PERIOD_CHECK_ENABLE             BIT(9)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cr irq_period_check bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid irq_period_check bit.
  * \retval 1 This is a valid irq_period_check bit.
  */
#define IS_SDIO_CR_IRQ_PERIOD_CHECK(regval)       (\
                                 ((regval) == SDIO_CR_IRQ_PERIOD_CHECK_DISABLE             ) || \
                                 ((regval) == SDIO_CR_IRQ_PERIOD_CHECK_ENABLE              )  \
                                                 )

#define SDIO_CR_STREAM_MODE                  BIT(10)                                      /*!< EMMC Stream mode  -1'b0: disable -1'b1: enable */
#define SDIO_CR_STREAM_MODE_OFS              10U                                          /*!< SDIO CR: STREAM_MODE Bit Offset */
#define SDIO_CR_STREAM_MODE_VAL(regval)          (BIT(10) & ((uint32_t)(regval) << 10))        /*!< SDIO CR: STREAM_MODE Bit Value */  
#define SDIO_CR_STREAM_MODE_DISABLE               0x0UL                                                  /*!< DISABLE */
#define SDIO_CR_STREAM_MODE_ENABLE                BIT(10)                                                  /*!< ENABLE */

/**
  * \brief Check the SDIO cr stream_mode bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid stream_mode bit.
  * \retval 1 This is a valid stream_mode bit.
  */
#define IS_SDIO_CR_STREAM_MODE(regval)            (\
                                      ((regval) == SDIO_CR_STREAM_MODE_DISABLE             ) || \
                                      ((regval) == SDIO_CR_STREAM_MODE_ENABLE              )  \
                                                 )

#define SDIO_CR_SLEEP_MODE                   BIT(11)                                      /*!< Sleep mode */
#define SDIO_CR_SLEEP_MODE_OFS               11U                                          /*!< SDIO CR: SLEEP_MODE Bit Offset */
#define SDIO_CR_SLEEP_MODE_VAL(regval)           (BIT(11) & ((uint32_t)(regval) << 11))        /*!< SDIO CR: SLEEP_MODE Bit Value */  
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE_MASK    BITS(18,19)                                   /*!< SDIO CR: NORMAL_BOOT_MODE_ENABLE Bit Mask */  
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE_OFS     18U                                          /*!< SDIO CR: NORMAL_BOOT_MODE_ENABLE Bit Offset */
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE(regval) (BITS(18,19) & ((uint32_t)(regval) << 18))        /*!< SDIO CR: NORMAL_BOOT_MODE_ENABLE Bit Value */  
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE_NON             SDIO_CR_NORMAL_BOOT_MODE_ENABLE(0)                                                    /*!< NON */
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE_NORMAL             SDIO_CR_NORMAL_BOOT_MODE_ENABLE(1)                                                    /*!< NORMAL */
#define SDIO_CR_NORMAL_BOOT_MODE_ENABLE_STANDBY             SDIO_CR_NORMAL_BOOT_MODE_ENABLE(2)                                                    /*!< STANDBY */

/**
  * \brief Check the SDIO cr normal_boot_mode_enable bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid normal_boot_mode_enable bits.
  * \retval 1 This is a valid normal_boot_mode_enable bits.
  */
#define IS_SDIO_CR_NORMAL_BOOT_MODE_ENABLE(regval)   (\
                             ((regval) == SDIO_CR_NORMAL_BOOT_MODE_ENABLE_NON                 ) || \
                             ((regval) == SDIO_CR_NORMAL_BOOT_MODE_ENABLE_NORMAL              ) || \
                             ((regval) == SDIO_CR_NORMAL_BOOT_MODE_ENABLE_STANDBY             )  \
                                                 )

#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE         BIT(20)                                      /*!< sdio is voltage switch mode enable -1'b0: non voltage switch mode -1'b1: voltage switch mode */
#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_OFS     20U                                          /*!< SDIO CR: VOLTAGE_SWITCH_MODE_ENABLE Bit Offset */
#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_VAL(regval) (BIT(20) & ((uint32_t)(regval) << 20))        /*!< SDIO CR: VOLTAGE_SWITCH_MODE_ENABLE Bit Value */  
#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_NON             0x0UL                                                  /*!< NON */
#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_VOLTAGE             BIT(20)                                                  /*!< VOLTAGE */

/**
  * \brief Check the SDIO cr voltage_switch_mode_enable bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid voltage_switch_mode_enable bit.
  * \retval 1 This is a valid voltage_switch_mode_enable bit.
  */
#define IS_SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE(regval)   (\
                             ((regval) == SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_NON                 ) || \
                             ((regval) == SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE_VOLTAGE             )  \
                                                 )

#define SDIO_CR_POWER_ON_MODE_ENABLE_MASK    BITS(21,22)                                   /*!< SDIO CR: POWER_ON_MODE_ENABLE Bit Mask */  
#define SDIO_CR_POWER_ON_MODE_ENABLE_OFS     21U                                          /*!< SDIO CR: POWER_ON_MODE_ENABLE Bit Offset */
#define SDIO_CR_POWER_ON_MODE_ENABLE(regval) (BITS(21,22) & ((uint32_t)(regval) << 21))        /*!< SDIO CR: POWER_ON_MODE_ENABLE Bit Value */  
#define SDIO_CR_VOLTAGE_SWITCH_START         BIT(23)                                      /*!< voltage switch start flag */
#define SDIO_CR_VOLTAGE_SWITCH_START_OFS     23U                                          /*!< SDIO CR: VOLTAGE_SWITCH_START Bit Offset */
#define SDIO_CR_VOLTAGE_SWITCH_START_VAL(regval) (BIT(23) & ((uint32_t)(regval) << 23))        /*!< SDIO CR: VOLTAGE_SWITCH_START Bit Value */  
#define SDIO_CR_BOOT_ACK_REQ                 BIT(24)                                      /*!< boot ack request */
#define SDIO_CR_BOOT_ACK_REQ_OFS             24U                                          /*!< SDIO CR: BOOT_ACK_REQ Bit Offset */
#define SDIO_CR_BOOT_ACK_REQ_VAL(regval)         (BIT(24) & ((uint32_t)(regval) << 24))        /*!< SDIO CR: BOOT_ACK_REQ Bit Value */  
#define SDIO_CR_READ_WAIT_REQ                BIT(25)                                      /*!< read wait request */
#define SDIO_CR_READ_WAIT_REQ_OFS            25U                                          /*!< SDIO CR: READ_WAIT_REQ Bit Offset */
#define SDIO_CR_READ_WAIT_REQ_VAL(regval)        (BIT(25) & ((uint32_t)(regval) << 25))        /*!< SDIO CR: READ_WAIT_REQ Bit Value */  
#define SDIO_CR_BOOT_DATA_REQ                BIT(26)                                      /*!< boot data request */
#define SDIO_CR_BOOT_DATA_REQ_OFS            26U                                          /*!< SDIO CR: BOOT_DATA_REQ Bit Offset */
#define SDIO_CR_BOOT_DATA_REQ_VAL(regval)        (BIT(26) & ((uint32_t)(regval) << 26))        /*!< SDIO CR: BOOT_DATA_REQ Bit Value */  
#define SDIO_CR_VOLTAGE_SDCLK_RESUME_CNT_MASK    BITS(28,31)                                   /*!< SDIO CR: VOLTAGE_SDCLK_RESUME_CNT Bit Mask */  
#define SDIO_CR_VOLTAGE_SDCLK_RESUME_CNT_OFS     28U                                          /*!< SDIO CR: VOLTAGE_SDCLK_RESUME_CNT Bit Offset */
#define SDIO_CR_VOLTAGE_SDCLK_RESUME_CNT(regval) (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< SDIO CR: VOLTAGE_SDCLK_RESUME_CNT Bit Value */  
 
 /* ===== SDIO TX_SADDR Register definition ===== */
#define SDIO_TX_SADDR_ADDRESS_MASK                 BITS(0,31)                                   /*!< SDIO TX SADDR: ADDRESS Bit Mask */  
#define SDIO_TX_SADDR_ADDRESS_OFS                  0U                                          /*!< SDIO TX SADDR: ADDRESS Bit Offset */
#define SDIO_TX_SADDR_ADDRESS(regval)              (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO TX SADDR: ADDRESS Bit Value */  
 
 /* ===== SDIO TX_SIZE Register definition ===== */
#define SDIO_TX_SIZE_TX_SIZE_MASK                 BITS(0,24)                                   /*!< SDIO TX SIZE: TX_SIZE Bit Mask */  
#define SDIO_TX_SIZE_TX_SIZE_OFS                  0U                                          /*!< SDIO TX SIZE: TX_SIZE Bit Offset */
#define SDIO_TX_SIZE_TX_SIZE(regval)              (BITS(0,24) & ((uint32_t)(regval) << 0))        /*!< SDIO TX SIZE: TX_SIZE Bit Value */  
 
 /* ===== SDIO TX_CFG Register definition ===== */
#define SDIO_TX_CFG_CONTINUOUS                   BIT(0)                                      /*!< TX channel continuous mode bitfield:  -1'b0: disable -1'b1: enable */
#define SDIO_TX_CFG_CONTINUOUS_OFS               0U                                          /*!< SDIO TX CFG: CONTINUOUS Bit Offset */
#define SDIO_TX_CFG_CONTINUOUS_VAL(regval)           (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO TX CFG: CONTINUOUS Bit Value */  
#define SDIO_TX_CFG_CONTINUOUS_DISABLE                0x0UL                                              /*!< DISABLE */
#define SDIO_TX_CFG_CONTINUOUS_ENABLE                 BIT(0)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO tx_cfg continuous bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid continuous bit.
  * \retval 1 This is a valid continuous bit.
  */
#define IS_SDIO_TX_CFG_CONTINUOUS(regval)             (\
                                       ((regval) == SDIO_TX_CFG_CONTINUOUS_DISABLE             ) || \
                                       ((regval) == SDIO_TX_CFG_CONTINUOUS_ENABLE              )  \
                                                 )

#define SDIO_TX_CFG_DATASIZE_MASK                BITS(1,2)                                   /*!< SDIO TX CFG: DATASIZE Bit Mask */  
#define SDIO_TX_CFG_DATASIZE_OFS                 1U                                          /*!< SDIO TX CFG: DATASIZE Bit Offset */
#define SDIO_TX_CFG_DATASIZE(regval)             (BITS(1,2) & ((uint32_t)(regval) << 1))        /*!< SDIO TX CFG: DATASIZE Bit Value */  
#define SDIO_TX_CFG_DATASIZE_BYTE                     SDIO_TX_CFG_DATASIZE(0)                                                /*!< BYTE */
#define SDIO_TX_CFG_DATASIZE_HALFWORD                 SDIO_TX_CFG_DATASIZE(1)                                                /*!< HALFWORD */
#define SDIO_TX_CFG_DATASIZE_WORD                     SDIO_TX_CFG_DATASIZE(2)                                                /*!< WORD */
#define SDIO_TX_CFG_DATASIZE_RESERVED                 SDIO_TX_CFG_DATASIZE(3)                                                /*!< RESERVED */

/**
  * \brief Check the SDIO tx_cfg datasize bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid datasize bits.
  * \retval 1 This is a valid datasize bits.
  */
#define IS_SDIO_TX_CFG_DATASIZE(regval)               (\
                                         ((regval) == SDIO_TX_CFG_DATASIZE_BYTE                ) || \
                                         ((regval) == SDIO_TX_CFG_DATASIZE_HALFWORD            ) || \
                                         ((regval) == SDIO_TX_CFG_DATASIZE_WORD                ) || \
                                         ((regval) == SDIO_TX_CFG_DATASIZE_RESERVED            )  \
                                                 )

#define SDIO_TX_CFG_EN                           BIT(4)                                      /*!< TX channel enable and start transfer bitfield: -1'b0: disable -1'b1: enable and start the transfer */
#define SDIO_TX_CFG_EN_OFS                       4U                                          /*!< SDIO TX CFG: EN Bit Offset */
#define SDIO_TX_CFG_EN_VAL(regval)                   (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO TX CFG: EN Bit Value */  
#define SDIO_TX_CFG_EN_DISABLE                        0x0UL                                              /*!< DISABLE */
#define SDIO_TX_CFG_EN_ENABLE                         BIT(4)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO tx_cfg en bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid en bit.
  * \retval 1 This is a valid en bit.
  */
#define IS_SDIO_TX_CFG_EN(regval)                     (\
                                               ((regval) == SDIO_TX_CFG_EN_DISABLE             ) || \
                                               ((regval) == SDIO_TX_CFG_EN_ENABLE              )  \
                                                 )

#define SDIO_TX_CFG_PENDING                      BIT(5)                                      /*!< TX transfer pending in queue status flag: -1'b0:  disable: no pending - no pending transfer in the queue -1'b1:  enable: pending - pending transfer in the queue */
#define SDIO_TX_CFG_PENDING_DISABLE                   ((uint32_t)(0) << 5)                                                /*!< DISABLE */
#define SDIO_TX_CFG_PENDING_ENABLE                    ((uint32_t)(1) << 5)                                                /*!< ENABLE */

/**
  * \brief Check the SDIO tx_cfg pending bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid pending bit.
  * \retval 1 This is a valid pending bit.
  */
#define IS_SDIO_TX_CFG_PENDING(regval)                (\
                                          ((regval) == SDIO_TX_CFG_PENDING_DISABLE             ) || \
                                          ((regval) == SDIO_TX_CFG_PENDING_ENABLE              )  \
                                                 )

#define SDIO_TX_CFG_CLR                          BIT(6)                                      /*!< TX channel clear and stop transfer bitfield: -1'b0: disable -1'b1:  enable: stop and clear - stop and clear the on-going transfer Note:The clear operation is not performed immediately.  You need to wait until TXEN is pulled down to clear it */
#define SDIO_TX_CFG_CLR_OFS                      6U                                          /*!< SDIO TX CFG: CLR Bit Offset */
#define SDIO_TX_CFG_CLR_VAL(regval)                  (BIT(6) & ((uint32_t)(regval) << 6))        /*!< SDIO TX CFG: CLR Bit Value */  

/**
  * \brief Check the SDIO tx_cfg clr bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid clr bit.
  * \retval 1 This is a valid clr bit.
  */
#define IS_SDIO_TX_CFG_CLR(regval)                    (\
                                                 )

 /* ===== SDIO VERSION Register definition ===== */
#define SDIO_VERSION_NUM                          BITS(0,31)                
 
 /* ===== SDIO CMD_OP Register definition ===== */
#define SDIO_CMD_OP_RSP                          BIT(0)                                      /*!< This bit field indicates whether to receive a response -1'b0: no response received -1'b1: yes: Receive 48bit response (default) */
#define SDIO_CMD_OP_RSP_OFS                      0U                                          /*!< SDIO CMD OP: RSP Bit Offset */
#define SDIO_CMD_OP_RSP_VAL(regval)                  (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO CMD OP: RSP Bit Value */  
#define SDIO_CMD_OP_RSP_NO                            0x0UL                                              /*!< NO */
#define SDIO_CMD_OP_RSP_YES                           BIT(0)                                                   /*!< YES */

/**
  * \brief Check the SDIO cmd_op rsp bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid rsp bit.
  * \retval 1 This is a valid rsp bit.
  */
#define IS_SDIO_CMD_OP_RSP(regval)                    (\
                                              ((regval) == SDIO_CMD_OP_RSP_NO                  ) || \
                                              ((regval) == SDIO_CMD_OP_RSP_YES                 )  \
                                                 )

#define SDIO_CMD_OP_RSP_LEN                      BIT(1)                                      /*!< This bit field represents the received response length selection -1'b0: 48:  receive 48bit response -1'b1: 136: Receive 136bit response */
#define SDIO_CMD_OP_RSP_LEN_OFS                  1U                                          /*!< SDIO CMD OP: RSP_LEN Bit Offset */
#define SDIO_CMD_OP_RSP_LEN_VAL(regval)              (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO CMD OP: RSP_LEN Bit Value */  
#define SDIO_CMD_OP_RSP_LEN_48                        0x0UL                                              /*!< 48 */
#define SDIO_CMD_OP_RSP_LEN_136                       BIT(1)                                                   /*!< 136 */

/**
  * \brief Check the SDIO cmd_op rsp_len bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid rsp_len bit.
  * \retval 1 This is a valid rsp_len bit.
  */
#define IS_SDIO_CMD_OP_RSP_LEN(regval)                (\
                                          ((regval) == SDIO_CMD_OP_RSP_LEN_48                  ) || \
                                          ((regval) == SDIO_CMD_OP_RSP_LEN_136                 )  \
                                                 )

#define SDIO_CMD_OP_CRC                          BIT(2)                                      /*!< This bit field indicates whether to enable CRC checking -1'b0: disable:  close CRC check -1'b1: enable:  Turn on CRC check */
#define SDIO_CMD_OP_CRC_OFS                      2U                                          /*!< SDIO CMD OP: CRC Bit Offset */
#define SDIO_CMD_OP_CRC_VAL(regval)                  (BIT(2) & ((uint32_t)(regval) << 2))        /*!< SDIO CMD OP: CRC Bit Value */  
#define SDIO_CMD_OP_CRC_DISABLE                       0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_CRC_ENABLE                        BIT(2)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op crc bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid crc bit.
  * \retval 1 This is a valid crc bit.
  */
#define IS_SDIO_CMD_OP_CRC(regval)                    (\
                                              ((regval) == SDIO_CMD_OP_CRC_DISABLE             ) || \
                                              ((regval) == SDIO_CMD_OP_CRC_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_BUSY                         BIT(3)                                      /*!< This bit field indicates whether to enable busy check for r1b command type. -1'b0: disable: off -1'b1: enable:  open */
#define SDIO_CMD_OP_BUSY_OFS                     3U                                          /*!< SDIO CMD OP: BUSY Bit Offset */
#define SDIO_CMD_OP_BUSY_VAL(regval)                 (BIT(3) & ((uint32_t)(regval) << 3))        /*!< SDIO CMD OP: BUSY Bit Value */  
#define SDIO_CMD_OP_BUSY_DISABLE                      0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_BUSY_ENABLE                       BIT(3)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op busy bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid busy bit.
  * \retval 1 This is a valid busy bit.
  */
#define IS_SDIO_CMD_OP_BUSY(regval)                   (\
                                             ((regval) == SDIO_CMD_OP_BUSY_DISABLE             ) || \
                                             ((regval) == SDIO_CMD_OP_BUSY_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_POWER_UP                     BIT(4)                                      /*!< This bit field indicates whether to enable power on initialization -1'b0: disable: off -1'b1: enable:  open */
#define SDIO_CMD_OP_POWER_UP_OFS                 4U                                          /*!< SDIO CMD OP: POWER_UP Bit Offset */
#define SDIO_CMD_OP_POWER_UP_VAL(regval)             (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO CMD OP: POWER_UP Bit Value */  
#define SDIO_CMD_OP_POWER_UP_DISABLE                  0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_POWER_UP_ENABLE                   BIT(4)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op power_up bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid power_up bit.
  * \retval 1 This is a valid power_up bit.
  */
#define IS_SDIO_CMD_OP_POWER_UP(regval)               (\
                                         ((regval) == SDIO_CMD_OP_POWER_UP_DISABLE             ) || \
                                         ((regval) == SDIO_CMD_OP_POWER_UP_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_CRC_CHECK                    BIT(5)                                      /*!< This bit field indicates whether to enable crc error check -1'b0: disable: off -1'b1: enable:  open */
#define SDIO_CMD_OP_CRC_CHECK_OFS                5U                                          /*!< SDIO CMD OP: CRC_CHECK Bit Offset */
#define SDIO_CMD_OP_CRC_CHECK_VAL(regval)            (BIT(5) & ((uint32_t)(regval) << 5))        /*!< SDIO CMD OP: CRC_CHECK Bit Value */  
#define SDIO_CMD_OP_CRC_CHECK_DISABLE                 0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_CRC_CHECK_ENABLE                  BIT(5)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op crc_check bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid crc_check bit.
  * \retval 1 This is a valid crc_check bit.
  */
#define IS_SDIO_CMD_OP_CRC_CHECK(regval)              (\
                                        ((regval) == SDIO_CMD_OP_CRC_CHECK_DISABLE             ) || \
                                        ((regval) == SDIO_CMD_OP_CRC_CHECK_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_STOP_CMD                     BIT(6)                                      /*!< This bit field indicates whether cmd whether abort command -1'b0: disable: no stop command -1'b1: enable: stop command */
#define SDIO_CMD_OP_STOP_CMD_OFS                 6U                                          /*!< SDIO CMD OP: STOP_CMD Bit Offset */
#define SDIO_CMD_OP_STOP_CMD_VAL(regval)             (BIT(6) & ((uint32_t)(regval) << 6))        /*!< SDIO CMD OP: STOP_CMD Bit Value */  
#define SDIO_CMD_OP_STOP_CMD_DISABLE                  0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_STOP_CMD_ENABLE                   BIT(6)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op stop_cmd bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid stop_cmd bit.
  * \retval 1 This is a valid stop_cmd bit.
  */
#define IS_SDIO_CMD_OP_STOP_CMD(regval)               (\
                                         ((regval) == SDIO_CMD_OP_STOP_CMD_DISABLE             ) || \
                                         ((regval) == SDIO_CMD_OP_STOP_CMD_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_CMD_ENABLE                   BIT(7)                                      /*!< This bit field indicates whether send command -1'b0: disable: not send command -1'b1: enable: send command */
#define SDIO_CMD_OP_CMD_ENABLE_OFS               7U                                          /*!< SDIO CMD OP: CMD_ENABLE Bit Offset */
#define SDIO_CMD_OP_CMD_ENABLE_VAL(regval)           (BIT(7) & ((uint32_t)(regval) << 7))        /*!< SDIO CMD OP: CMD_ENABLE Bit Value */  
#define SDIO_CMD_OP_CMD_ENABLE_DISABLE                0x0UL                                              /*!< DISABLE */
#define SDIO_CMD_OP_CMD_ENABLE_ENABLE                 BIT(7)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO cmd_op cmd_enable bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid cmd_enable bit.
  * \retval 1 This is a valid cmd_enable bit.
  */
#define IS_SDIO_CMD_OP_CMD_ENABLE(regval)             (\
                                       ((regval) == SDIO_CMD_OP_CMD_ENABLE_DISABLE             ) || \
                                       ((regval) == SDIO_CMD_OP_CMD_ENABLE_ENABLE              )  \
                                                 )

#define SDIO_CMD_OP_INDEX_MASK                   BITS(8,13)                                   /*!< SDIO CMD OP: INDEX Bit Mask */  
#define SDIO_CMD_OP_INDEX_OFS                    8U                                          /*!< SDIO CMD OP: INDEX Bit Offset */
#define SDIO_CMD_OP_INDEX(regval)                (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< SDIO CMD OP: INDEX Bit Value */  
 
 /* ===== SDIO CMD_ARG Register definition ===== */
#define SDIO_CMD_ARG_ARG_MASK                     BITS(0,31)                                   /*!< SDIO CMD ARG: ARG Bit Mask */  
#define SDIO_CMD_ARG_ARG_OFS                      0U                                          /*!< SDIO CMD ARG: ARG Bit Offset */
#define SDIO_CMD_ARG_ARG(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO CMD ARG: ARG Bit Value */  
 
 /* ===== SDIO DATA_SETUP Register definition ===== */
#define SDIO_DATA_SETUP_CHANNEL                      BIT(0)                                      /*!< SDIO data channel enable -1'b0: disable: off -1'b1: enable: open */
#define SDIO_DATA_SETUP_CHANNEL_OFS                  0U                                          /*!< SDIO DATA SETUP: CHANNEL Bit Offset */
#define SDIO_DATA_SETUP_CHANNEL_VAL(regval)              (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO DATA SETUP: CHANNEL Bit Value */  
#define SDIO_DATA_SETUP_CHANNEL_DISABLE                   0x0UL                                          /*!< DISABLE */
#define SDIO_DATA_SETUP_CHANNEL_ENABLE                    BIT(0)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO data_setup channel bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid channel bit.
  * \retval 1 This is a valid channel bit.
  */
#define IS_SDIO_DATA_SETUP_CHANNEL(regval)                (\
                                          ((regval) == SDIO_DATA_SETUP_CHANNEL_DISABLE             ) || \
                                          ((regval) == SDIO_DATA_SETUP_CHANNEL_ENABLE              )  \
                                                 )

#define SDIO_DATA_SETUP_RWN                          BIT(1)                                      /*!< SDIO read / write enable -1'b0: write enable -1'b1: read enable */
#define SDIO_DATA_SETUP_RWN_OFS                      1U                                          /*!< SDIO DATA SETUP: RWN Bit Offset */
#define SDIO_DATA_SETUP_RWN_VAL(regval)                  (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO DATA SETUP: RWN Bit Value */  
#define SDIO_DATA_SETUP_RWN_WRITE                         0x0UL                                          /*!< WRITE */
#define SDIO_DATA_SETUP_RWN_READ                          BIT(1)                                                   /*!< READ */

/**
  * \brief Check the SDIO data_setup rwn bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid rwn bit.
  * \retval 1 This is a valid rwn bit.
  */
#define IS_SDIO_DATA_SETUP_RWN(regval)                    (\
                                              ((regval) == SDIO_DATA_SETUP_RWN_WRITE               ) || \
                                              ((regval) == SDIO_DATA_SETUP_RWN_READ                )  \
                                                 )

#define SDIO_DATA_SETUP_MODE_MASK                    BITS(2,3)                                   /*!< SDIO DATA SETUP: MODE Bit Mask */  
#define SDIO_DATA_SETUP_MODE_OFS                     2U                                          /*!< SDIO DATA SETUP: MODE Bit Offset */
#define SDIO_DATA_SETUP_MODE(regval)                 (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< SDIO DATA SETUP: MODE Bit Value */  
#define SDIO_DATA_SETUP_MODE_SINGLE                       SDIO_DATA_SETUP_MODE(0)                                            /*!< SINGLE */
#define SDIO_DATA_SETUP_MODE_QUAD                         SDIO_DATA_SETUP_MODE(1)                                            /*!< QUAD */
#define SDIO_DATA_SETUP_MODE_OCTOL                        SDIO_DATA_SETUP_MODE(2)                                            /*!< OCTOL */

/**
  * \brief Check the SDIO data_setup mode bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid mode bits.
  * \retval 1 This is a valid mode bits.
  */
#define IS_SDIO_DATA_SETUP_MODE(regval)                   (\
                                             ((regval) == SDIO_DATA_SETUP_MODE_SINGLE              ) || \
                                             ((regval) == SDIO_DATA_SETUP_MODE_QUAD                ) || \
                                             ((regval) == SDIO_DATA_SETUP_MODE_OCTOL               )  \
                                                 )

#define SDIO_DATA_SETUP_BLOCK_NUM_MASK               BITS(4,19)                                   /*!< SDIO DATA SETUP: BLOCK_NUM Bit Mask */  
#define SDIO_DATA_SETUP_BLOCK_NUM_OFS                4U                                          /*!< SDIO DATA SETUP: BLOCK_NUM Bit Offset */
#define SDIO_DATA_SETUP_BLOCK_NUM(regval)            (BITS(4,19) & ((uint32_t)(regval) << 4))        /*!< SDIO DATA SETUP: BLOCK_NUM Bit Value */  
#define SDIO_DATA_SETUP_BLOCK_SIZE_MASK              BITS(20,31)                                   /*!< SDIO DATA SETUP: BLOCK_SIZE Bit Mask */  
#define SDIO_DATA_SETUP_BLOCK_SIZE_OFS               20U                                          /*!< SDIO DATA SETUP: BLOCK_SIZE Bit Offset */
#define SDIO_DATA_SETUP_BLOCK_SIZE(regval)           (BITS(20,31) & ((uint32_t)(regval) << 20))        /*!< SDIO DATA SETUP: BLOCK_SIZE Bit Value */  
 
 /* ===== SDIO START Register definition ===== */
#define SDIO_START_ENABLE                       BIT(0)                                      /*!< Start transmission, write 1 to generate pulse and reset automatically */
#define SDIO_START_ENABLE_OFS                   0U                                          /*!< SDIO START: ENABLE Bit Offset */
#define SDIO_START_ENABLE_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO START: ENABLE Bit Value */  
 
 /* ===== SDIO RSP0 Register definition ===== */
#define SDIO_RSP0_NUM                          BITS(0,31)                
 
 /* ===== SDIO RSP1 Register definition ===== */
#define SDIO_RSP1_NUM                          BITS(0,31)                
 
 /* ===== SDIO RSP2 Register definition ===== */
#define SDIO_RSP2_NUM                          BITS(0,31)                
 
 /* ===== SDIO RSP3 Register definition ===== */
#define SDIO_RSP3_NUM                          BITS(0,31)                
 
 /* ===== SDIO CLK_DIV Register definition ===== */
#define SDIO_CLK_DIV_NUM_MASK                     BITS(0,31)                                   /*!< SDIO CLK DIV: NUM Bit Mask */  
#define SDIO_CLK_DIV_NUM_OFS                      0U                                          /*!< SDIO CLK DIV: NUM Bit Offset */
#define SDIO_CLK_DIV_NUM(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO CLK DIV: NUM Bit Value */  
 
 /* ===== SDIO STATUS Register definition ===== */
#define SDIO_STATUS_EOT                          BIT(0)                                      /*!< Transmission end flag, write 1 clear 0 */
#define SDIO_STATUS_EOT_OFS                      0U                                          /*!< SDIO STATUS: EOT Bit Offset */
#define SDIO_STATUS_EOT_VAL(regval)                  (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO STATUS: EOT Bit Value */  
#define SDIO_STATUS_ERR                          BIT(1)                                      /*!< Transmission error flag, write 1 clear 0 */
#define SDIO_STATUS_ERR_OFS                      1U                                          /*!< SDIO STATUS: ERR Bit Offset */
#define SDIO_STATUS_ERR_VAL(regval)                  (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO STATUS: ERR Bit Value */  
#define SDIO_STATUS_TXUDR_ERR                    BIT(2)                                      /*!< tx udr error flag , write 1 clear 0 */
#define SDIO_STATUS_TXUDR_ERR_OFS                2U                                          /*!< SDIO STATUS: TXUDR_ERR Bit Offset */
#define SDIO_STATUS_TXUDR_ERR_VAL(regval)            (BIT(2) & ((uint32_t)(regval) << 2))        /*!< SDIO STATUS: TXUDR_ERR Bit Value */  
#define SDIO_STATUS_TXOVF_ERR                    BIT(3)                                      /*!< tx ovf error flag , write 1 clear 0 */
#define SDIO_STATUS_TXOVF_ERR_OFS                3U                                          /*!< SDIO STATUS: TXOVF_ERR Bit Offset */
#define SDIO_STATUS_TXOVF_ERR_VAL(regval)            (BIT(3) & ((uint32_t)(regval) << 3))        /*!< SDIO STATUS: TXOVF_ERR Bit Value */  
#define SDIO_STATUS_RXUDR_ERR                    BIT(4)                                      /*!< rx udr error flag , write 1 clear 0 */
#define SDIO_STATUS_RXUDR_ERR_OFS                4U                                          /*!< SDIO STATUS: RXUDR_ERR Bit Offset */
#define SDIO_STATUS_RXUDR_ERR_VAL(regval)            (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO STATUS: RXUDR_ERR Bit Value */  
#define SDIO_STATUS_RXOVF_ERR                    BIT(5)                                      /*!< rx ovf error flag , write 1 clear 0 */
#define SDIO_STATUS_RXOVF_ERR_OFS                5U                                          /*!< SDIO STATUS: RXOVF_ERR Bit Offset */
#define SDIO_STATUS_RXOVF_ERR_VAL(regval)            (BIT(5) & ((uint32_t)(regval) << 5))        /*!< SDIO STATUS: RXOVF_ERR Bit Value */  
#define SDIO_STATUS_BUSY                         BIT(6)                                      /*!< sdio busy flag  */
#define SDIO_STATUS_CLEAR_FIFO_STAR              BIT(7)                                      /*!< sdio clear fifo  */
#define SDIO_STATUS_CLEAR_FIFO_STAR_OFS          7U                                          /*!< SDIO STATUS: CLEAR_FIFO_STAR Bit Offset */
#define SDIO_STATUS_CLEAR_FIFO_STAR_VAL(regval)      (BIT(7) & ((uint32_t)(regval) << 7))        /*!< SDIO STATUS: CLEAR_FIFO_STAR Bit Value */  
#define SDIO_STATUS_IRQ_CHECK                    BIT(8)                                      /*!< sdio interrupt check  */
#define SDIO_STATUS_IRQ_CHECK_OFS                8U                                          /*!< SDIO STATUS: IRQ_CHECK Bit Offset */
#define SDIO_STATUS_IRQ_CHECK_VAL(regval)            (BIT(8) & ((uint32_t)(regval) << 8))        /*!< SDIO STATUS: IRQ_CHECK Bit Value */  
#define SDIO_STATUS_BLOCK_DONE                   BIT(9)                                      /*!< sdio block done */
#define SDIO_STATUS_BLOCK_DONE_OFS               9U                                          /*!< SDIO STATUS: BLOCK_DONE Bit Offset */
#define SDIO_STATUS_BLOCK_DONE_VAL(regval)           (BIT(9) & ((uint32_t)(regval) << 9))        /*!< SDIO STATUS: BLOCK_DONE Bit Value */  
#define SDIO_STATUS_BUSY0_END                    BIT(10)                                      /*!< sdio sleep done */
#define SDIO_STATUS_BUSY0_END_OFS                10U                                          /*!< SDIO STATUS: BUSY0_END Bit Offset */
#define SDIO_STATUS_BUSY0_END_VAL(regval)            (BIT(10) & ((uint32_t)(regval) << 10))        /*!< SDIO STATUS: BUSY0_END Bit Value */  
#define SDIO_STATUS_VOLTAGE_SWITCH_PASS          BIT(11)                                      /*!< sdio voltage switch pass */
#define SDIO_STATUS_VOLTAGE_SWITCH_PASS_OFS      11U                                          /*!< SDIO STATUS: VOLTAGE_SWITCH_PASS Bit Offset */
#define SDIO_STATUS_VOLTAGE_SWITCH_PASS_VAL(regval)  (BIT(11) & ((uint32_t)(regval) << 11))        /*!< SDIO STATUS: VOLTAGE_SWITCH_PASS Bit Value */  
#define SDIO_STATUS_VOLTAGE_SWITCH_FAIL          BIT(12)                                      /*!< sdio voltage switch fail */
#define SDIO_STATUS_VOLTAGE_SWITCH_FAIL_OFS      12U                                          /*!< SDIO STATUS: VOLTAGE_SWITCH_FAIL Bit Offset */
#define SDIO_STATUS_VOLTAGE_SWITCH_FAIL_VAL(regval)  (BIT(12) & ((uint32_t)(regval) << 12))        /*!< SDIO STATUS: VOLTAGE_SWITCH_FAIL Bit Value */  
#define SDIO_STATUS_BOOT_ACK_PASS                BIT(13)                                      /*!< sdio boot ack pass */
#define SDIO_STATUS_BOOT_ACK_PASS_OFS            13U                                          /*!< SDIO STATUS: BOOT_ACK_PASS Bit Offset */
#define SDIO_STATUS_BOOT_ACK_PASS_VAL(regval)        (BIT(13) & ((uint32_t)(regval) << 13))        /*!< SDIO STATUS: BOOT_ACK_PASS Bit Value */  
#define SDIO_STATUS_BOOT_ACK_FAIL                BIT(14)                                      /*!< sdio boot ack fail */
#define SDIO_STATUS_BOOT_ACK_FAIL_OFS            14U                                          /*!< SDIO STATUS: BOOT_ACK_FAIL Bit Offset */
#define SDIO_STATUS_BOOT_ACK_FAIL_VAL(regval)        (BIT(14) & ((uint32_t)(regval) << 14))        /*!< SDIO STATUS: BOOT_ACK_FAIL Bit Value */  
#define SDIO_STATUS_CLK_STOP                     BIT(15)                                      /*!< sdio clk stop flags */
#define SDIO_STATUS_CMD_ERR                      BITS(16,21)                
#define SDIO_STATUS_CMD_ERR_NO                        ((uint32_t)(0) << 16)                                                /*!< NO */
#define SDIO_STATUS_CMD_ERR_TIMEOUT                   ((uint32_t)(1) << 16)                                                /*!< TIMEOUT */
#define SDIO_STATUS_CMD_ERR_DIR                       ((uint32_t)(2) << 16)                                                /*!< DIR */
#define SDIO_STATUS_CMD_ERR_BUSY                      ((uint32_t)(3) << 16)                                                /*!< BUSY */
#define SDIO_STATUS_CMD_ERR_CRCERR                    ((uint32_t)(4) << 16)                                                /*!< CRCERR */

/**
  * \brief Check the SDIO status cmd_err bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid cmd_err bits.
  * \retval 1 This is a valid cmd_err bits.
  */
#define IS_SDIO_STATUS_CMD_ERR(regval)                (\
                                          ((regval) == SDIO_STATUS_CMD_ERR_NO                  ) || \
                                          ((regval) == SDIO_STATUS_CMD_ERR_TIMEOUT             ) || \
                                          ((regval) == SDIO_STATUS_CMD_ERR_DIR                 ) || \
                                          ((regval) == SDIO_STATUS_CMD_ERR_BUSY                ) || \
                                          ((regval) == SDIO_STATUS_CMD_ERR_CRCERR              )  \
                                                 )

#define SDIO_STATUS_VOLTAGE_SWITCH_END           BIT(22)                                      /*!< sdio voltage switch end flags */
#define SDIO_STATUS_VOLTAGE_SWITCH_END_OFS       22U                                          /*!< SDIO STATUS: VOLTAGE_SWITCH_END Bit Offset */
#define SDIO_STATUS_VOLTAGE_SWITCH_END_VAL(regval)   (BIT(22) & ((uint32_t)(regval) << 22))        /*!< SDIO STATUS: VOLTAGE_SWITCH_END Bit Value */  
#define SDIO_STATUS_DATA_ERR                     BITS(24,29)                
#define SDIO_STATUS_DATA_ERR_NO                       ((uint32_t)(0) << 24)                                                /*!< NO */
#define SDIO_STATUS_DATA_ERR_TIMEOUT                  ((uint32_t)(1) << 24)                                                /*!< TIMEOUT */
#define SDIO_STATUS_DATA_ERR_BUSY                     ((uint32_t)(2) << 24)                                                /*!< BUSY */
#define SDIO_STATUS_DATA_ERR_CRCTIMEOUT               ((uint32_t)(3) << 24)                                                /*!< CRCTIMEOUT */
#define SDIO_STATUS_DATA_ERR_CRCERR                   ((uint32_t)(4) << 24)                                                /*!< CRCERR */

/**
  * \brief Check the SDIO status data_err bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid data_err bits.
  * \retval 1 This is a valid data_err bits.
  */
#define IS_SDIO_STATUS_DATA_ERR(regval)               (\
                                         ((regval) == SDIO_STATUS_DATA_ERR_NO                  ) || \
                                         ((regval) == SDIO_STATUS_DATA_ERR_TIMEOUT             ) || \
                                         ((regval) == SDIO_STATUS_DATA_ERR_BUSY                ) || \
                                         ((regval) == SDIO_STATUS_DATA_ERR_CRCTIMEOUT          ) || \
                                         ((regval) == SDIO_STATUS_DATA_ERR_CRCERR              )  \
                                                 )

#define SDIO_STATUS_DATA_RUNING                  BIT(30)                                      /*!< This bit field indicates whether the data channel is working 0: the data channel is not working 1:The data channel is working */
#define SDIO_STATUS_SLAVE_READY                  BIT(31)                                      /*!< sdio slave ready  */
#define SDIO_STATUS_SLAVE_READY_OFS              31U                                          /*!< SDIO STATUS: SLAVE_READY Bit Offset */
#define SDIO_STATUS_SLAVE_READY_VAL(regval)          (BIT(31) & ((uint32_t)(regval) << 31))        /*!< SDIO STATUS: SLAVE_READY Bit Value */  
 
 /* ===== SDIO STOP_CMD_OP Register definition ===== */
#define SDIO_STOP_CMD_OP_TYPE_NO_RSP                  BIT(0)                                      /*!< This bit field indicates whether to receive a response -1'b0: no response received -1'b1: yes: Receive 48bit response (default) */
#define SDIO_STOP_CMD_OP_TYPE_NO_RSP_OFS              0U                                          /*!< SDIO STOP CMD OP: TYPE_NO_RSP Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_NO_RSP_VAL(regval)          (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO STOP CMD OP: TYPE_NO_RSP Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_NO_RSP_NO                    0x0UL                                         /*!< NO */
#define SDIO_STOP_CMD_OP_TYPE_NO_RSP_YES                   BIT(0)                                                   /*!< YES */

/**
  * \brief Check the SDIO stop_cmd_op type_no_rsp bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_no_rsp bit.
  * \retval 1 This is a valid type_no_rsp bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_NO_RSP(regval)            (\
                                      ((regval) == SDIO_STOP_CMD_OP_TYPE_NO_RSP_NO                  ) || \
                                      ((regval) == SDIO_STOP_CMD_OP_TYPE_NO_RSP_YES                 )  \
                                                 )

#define SDIO_STOP_CMD_OP_TYPE_136BIT                  BIT(1)                                      /*!< This bit field represents the received response length selection -1'b0: 48: receive 48bit response -1'b1: 136: Receive 136bit response */
#define SDIO_STOP_CMD_OP_TYPE_136BIT_OFS              1U                                          /*!< SDIO STOP CMD OP: TYPE_136BIT Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_136BIT_VAL(regval)          (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO STOP CMD OP: TYPE_136BIT Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_136BIT_48                    0x0UL                                         /*!< 48 */
#define SDIO_STOP_CMD_OP_TYPE_136BIT_136                   BIT(1)                                                   /*!< 136 */

/**
  * \brief Check the SDIO stop_cmd_op type_136bit bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_136bit bit.
  * \retval 1 This is a valid type_136bit bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_136BIT(regval)            (\
                                      ((regval) == SDIO_STOP_CMD_OP_TYPE_136BIT_48                  ) || \
                                      ((regval) == SDIO_STOP_CMD_OP_TYPE_136BIT_136                 )  \
                                                 )

#define SDIO_STOP_CMD_OP_TYPE_CRC                     BIT(2)                                      /*!< This bit field indicates whether to enable CRC checking -1'b0: disable: close CRC check -1'b1: enable: Turn on CRC check */
#define SDIO_STOP_CMD_OP_TYPE_CRC_OFS                 2U                                          /*!< SDIO STOP CMD OP: TYPE_CRC Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_CRC_VAL(regval)             (BIT(2) & ((uint32_t)(regval) << 2))        /*!< SDIO STOP CMD OP: TYPE_CRC Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_CRC_DISABLE                  0x0UL                                         /*!< DISABLE */
#define SDIO_STOP_CMD_OP_TYPE_CRC_ENABLE                   BIT(2)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO stop_cmd_op type_crc bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_crc bit.
  * \retval 1 This is a valid type_crc bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_CRC(regval)               (\
                                         ((regval) == SDIO_STOP_CMD_OP_TYPE_CRC_DISABLE             ) || \
                                         ((regval) == SDIO_STOP_CMD_OP_TYPE_CRC_ENABLE              )  \
                                                 )

#define SDIO_STOP_CMD_OP_TYPE_BUSY                    BIT(3)                                      /*!< This bit field indicates whether to enable busy check for r1b command type. -1'b0: disable: off -1'b1: enable: open */
#define SDIO_STOP_CMD_OP_TYPE_BUSY_OFS                3U                                          /*!< SDIO STOP CMD OP: TYPE_BUSY Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_BUSY_VAL(regval)            (BIT(3) & ((uint32_t)(regval) << 3))        /*!< SDIO STOP CMD OP: TYPE_BUSY Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_BUSY_DISABLE                 0x0UL                                         /*!< DISABLE */
#define SDIO_STOP_CMD_OP_TYPE_BUSY_ENABLE                  BIT(3)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO stop_cmd_op type_busy bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_busy bit.
  * \retval 1 This is a valid type_busy bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_BUSY(regval)              (\
                                        ((regval) == SDIO_STOP_CMD_OP_TYPE_BUSY_DISABLE             ) || \
                                        ((regval) == SDIO_STOP_CMD_OP_TYPE_BUSY_ENABLE              )  \
                                                 )

#define SDIO_STOP_CMD_OP_TYPE_POWER_UP                BIT(4)                                      /*!< This bit field indicates whether to enable power on initialization -1'b0: disable: off -1'b1: enable: open */
#define SDIO_STOP_CMD_OP_TYPE_POWER_UP_OFS            4U                                          /*!< SDIO STOP CMD OP: TYPE_POWER_UP Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_POWER_UP_VAL(regval)        (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO STOP CMD OP: TYPE_POWER_UP Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_POWER_UP_DISABLE             0x0UL                                         /*!< DISABLE */
#define SDIO_STOP_CMD_OP_TYPE_POWER_UP_ENABLE              BIT(4)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO stop_cmd_op type_power_up bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_power_up bit.
  * \retval 1 This is a valid type_power_up bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_POWER_UP(regval)          (\
                                    ((regval) == SDIO_STOP_CMD_OP_TYPE_POWER_UP_DISABLE             ) || \
                                    ((regval) == SDIO_STOP_CMD_OP_TYPE_POWER_UP_ENABLE              )  \
                                                 )

#define SDIO_STOP_CMD_OP_TYPE_CRC_CHECK               BIT(5)                                      /*!< This bit field indicates whether to enable crc error check -1'b0: disable: off -1'b1: enable: open */
#define SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_OFS           5U                                          /*!< SDIO STOP CMD OP: TYPE_CRC_CHECK Bit Offset */
#define SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_VAL(regval)       (BIT(5) & ((uint32_t)(regval) << 5))        /*!< SDIO STOP CMD OP: TYPE_CRC_CHECK Bit Value */  
#define SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_DISABLE             0x0UL                                         /*!< DISABLE */
#define SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_ENABLE             BIT(5)                                                   /*!< ENABLE */

/**
  * \brief Check the SDIO stop_cmd_op type_crc_check bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid type_crc_check bit.
  * \retval 1 This is a valid type_crc_check bit.
  */
#define IS_SDIO_STOP_CMD_OP_TYPE_CRC_CHECK(regval)         (\
                                   ((regval) == SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_DISABLE             ) || \
                                   ((regval) == SDIO_STOP_CMD_OP_TYPE_CRC_CHECK_ENABLE              )  \
                                                 )

#define SDIO_STOP_CMD_OP_INDEX_MASK                   BITS(8,13)                                   /*!< SDIO STOP CMD OP: INDEX Bit Mask */  
#define SDIO_STOP_CMD_OP_INDEX_OFS                    8U                                          /*!< SDIO STOP CMD OP: INDEX Bit Offset */
#define SDIO_STOP_CMD_OP_INDEX(regval)                (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< SDIO STOP CMD OP: INDEX Bit Value */  
 
 /* ===== SDIO STOP_CMD_ARG Register definition ===== */
#define SDIO_STOP_CMD_ARG_CMD_ARG_MASK                 BITS(0,31)                                   /*!< SDIO STOP CMD ARG: CMD_ARG Bit Mask */  
#define SDIO_STOP_CMD_ARG_CMD_ARG_OFS                  0U                                          /*!< SDIO STOP CMD ARG: CMD_ARG Bit Offset */
#define SDIO_STOP_CMD_ARG_CMD_ARG(regval)              (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO STOP CMD ARG: CMD_ARG Bit Value */  
 
 /* ===== SDIO DATA_TIMEOUT Register definition ===== */
#define SDIO_DATA_TIMEOUT_CNT_MASK                     BITS(0,31)                                   /*!< SDIO DATA TIMEOUT: CNT Bit Mask */  
#define SDIO_DATA_TIMEOUT_CNT_OFS                      0U                                          /*!< SDIO DATA TIMEOUT: CNT Bit Offset */
#define SDIO_DATA_TIMEOUT_CNT(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO DATA TIMEOUT: CNT Bit Value */  
 
 /* ===== SDIO CMD_POWERUP Register definition ===== */
#define SDIO_CMD_POWERUP_CNT_MASK                     BITS(0,31)                                   /*!< SDIO CMD POWERUP: CNT Bit Mask */  
#define SDIO_CMD_POWERUP_CNT_OFS                      0U                                          /*!< SDIO CMD POWERUP: CNT Bit Offset */
#define SDIO_CMD_POWERUP_CNT(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO CMD POWERUP: CNT Bit Value */  
 
 /* ===== SDIO CMD_WAIT_RSP Register definition ===== */
#define SDIO_CMD_WAIT_RSP_CNT_MASK                     BITS(0,31)                                   /*!< SDIO CMD WAIT RSP: CNT Bit Mask */  
#define SDIO_CMD_WAIT_RSP_CNT_OFS                      0U                                          /*!< SDIO CMD WAIT RSP: CNT Bit Offset */
#define SDIO_CMD_WAIT_RSP_CNT(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO CMD WAIT RSP: CNT Bit Value */  
 
 /* ===== SDIO CMD_WAIT_EOT Register definition ===== */
#define SDIO_CMD_WAIT_EOT_CNT_MASK                     BITS(0,31)                                   /*!< SDIO CMD WAIT EOT: CNT Bit Mask */  
#define SDIO_CMD_WAIT_EOT_CNT_OFS                      0U                                          /*!< SDIO CMD WAIT EOT: CNT Bit Offset */
#define SDIO_CMD_WAIT_EOT_CNT(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO CMD WAIT EOT: CNT Bit Value */  
 
 /* ===== SDIO TX_DATA Register definition ===== */
#define SDIO_TX_DATA_NUM_MASK                     BITS(0,31)                                   /*!< SDIO TX DATA: NUM Bit Mask */  
#define SDIO_TX_DATA_NUM_OFS                      0U                                          /*!< SDIO TX DATA: NUM Bit Offset */
#define SDIO_TX_DATA_NUM(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO TX DATA: NUM Bit Value */  
 
 /* ===== SDIO RX_DATA Register definition ===== */
#define SDIO_RX_DATA_NUM                          BITS(0,31)                
 
 /* ===== SDIO TX_MARK Register definition ===== */
#define SDIO_TX_MARK_NUM_MASK                     BITS(0,6)                                   /*!< SDIO TX MARK: NUM Bit Mask */  
#define SDIO_TX_MARK_NUM_OFS                      0U                                          /*!< SDIO TX MARK: NUM Bit Offset */
#define SDIO_TX_MARK_NUM(regval)                  (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< SDIO TX MARK: NUM Bit Value */  
 
 /* ===== SDIO RX_MARK Register definition ===== */
#define SDIO_RX_MARK_NUM_MASK                     BITS(0,6)                                   /*!< SDIO RX MARK: NUM Bit Mask */  
#define SDIO_RX_MARK_NUM_OFS                      0U                                          /*!< SDIO RX MARK: NUM Bit Offset */
#define SDIO_RX_MARK_NUM(regval)                  (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< SDIO RX MARK: NUM Bit Value */  
 
 /* ===== SDIO IP Register definition ===== */
#define SDIO_IP_TX_IRQ                       BIT(0)                                      /*!< Transmit watermark enable */
#define SDIO_IP_RX_IRQ                       BIT(1)                                      /*!< Receive watermark enable */
#define SDIO_IP_RX_EMPTY                     BIT(2)                                      /*!< receive fifo empty */
#define SDIO_IP_TX_FULL                      BIT(3)                                      /*!< send fifo full */
#define SDIO_IP_TX_EMPTY                     BIT(4)                                      /*!< send fifo empty */
#define SDIO_IP_RX_FULL                      BIT(5)                                      /*!< receive fifo full */
 
 /* ===== SDIO IE Register definition ===== */
#define SDIO_IE_TX_IRQ_EN                    BIT(0)                                      /*!< Transmit watermark enable */
#define SDIO_IE_TX_IRQ_EN_OFS                0U                                          /*!< SDIO IE: TX_IRQ_EN Bit Offset */
#define SDIO_IE_TX_IRQ_EN_VAL(regval)            (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO IE: TX_IRQ_EN Bit Value */  
#define SDIO_IE_RX_IRQ_EN                    BIT(1)                                      /*!< Receive watermark enable */
#define SDIO_IE_RX_IRQ_EN_OFS                1U                                          /*!< SDIO IE: RX_IRQ_EN Bit Offset */
#define SDIO_IE_RX_IRQ_EN_VAL(regval)            (BIT(1) & ((uint32_t)(regval) << 1))        /*!< SDIO IE: RX_IRQ_EN Bit Value */  
#define SDIO_IE_EOT_IRQ_EN                   BIT(2)                                      /*!< end interrupt enable */
#define SDIO_IE_EOT_IRQ_EN_OFS               2U                                          /*!< SDIO IE: EOT_IRQ_EN Bit Offset */
#define SDIO_IE_EOT_IRQ_EN_VAL(regval)           (BIT(2) & ((uint32_t)(regval) << 2))        /*!< SDIO IE: EOT_IRQ_EN Bit Value */  
#define SDIO_IE_ERR_IRQ_EN                   BIT(3)                                      /*!< error interrupt enable */
#define SDIO_IE_ERR_IRQ_EN_OFS               3U                                          /*!< SDIO IE: ERR_IRQ_EN Bit Offset */
#define SDIO_IE_ERR_IRQ_EN_VAL(regval)           (BIT(3) & ((uint32_t)(regval) << 3))        /*!< SDIO IE: ERR_IRQ_EN Bit Value */  
#define SDIO_IE_TXUDR_ERR_EN                 BIT(4)                                      /*!< tx udr error interrupt enable */
#define SDIO_IE_TXUDR_ERR_EN_OFS             4U                                          /*!< SDIO IE: TXUDR_ERR_EN Bit Offset */
#define SDIO_IE_TXUDR_ERR_EN_VAL(regval)         (BIT(4) & ((uint32_t)(regval) << 4))        /*!< SDIO IE: TXUDR_ERR_EN Bit Value */  
#define SDIO_IE_TXOVF_ERR_EN                 BIT(5)                                      /*!< tx ovf error interrupt enable */
#define SDIO_IE_TXOVF_ERR_EN_OFS             5U                                          /*!< SDIO IE: TXOVF_ERR_EN Bit Offset */
#define SDIO_IE_TXOVF_ERR_EN_VAL(regval)         (BIT(5) & ((uint32_t)(regval) << 5))        /*!< SDIO IE: TXOVF_ERR_EN Bit Value */  
#define SDIO_IE_RXUDR_ERR_EN                 BIT(6)                                      /*!< rx udr error interrupt enable */
#define SDIO_IE_RXUDR_ERR_EN_OFS             6U                                          /*!< SDIO IE: RXUDR_ERR_EN Bit Offset */
#define SDIO_IE_RXUDR_ERR_EN_VAL(regval)         (BIT(6) & ((uint32_t)(regval) << 6))        /*!< SDIO IE: RXUDR_ERR_EN Bit Value */  
#define SDIO_IE_RXOVF_ERR_EN                 BIT(7)                                      /*!< rx ovf error interrupt enable */
#define SDIO_IE_RXOVF_ERR_EN_OFS             7U                                          /*!< SDIO IE: RXOVF_ERR_EN Bit Offset */
#define SDIO_IE_RXOVF_ERR_EN_VAL(regval)         (BIT(7) & ((uint32_t)(regval) << 7))        /*!< SDIO IE: RXOVF_ERR_EN Bit Value */  
#define SDIO_IE_IRQ_CHECK_EN                 BIT(8)                                      /*!< sdio check interrupt enable */
#define SDIO_IE_IRQ_CHECK_EN_OFS             8U                                          /*!< SDIO IE: IRQ_CHECK_EN Bit Offset */
#define SDIO_IE_IRQ_CHECK_EN_VAL(regval)         (BIT(8) & ((uint32_t)(regval) << 8))        /*!< SDIO IE: IRQ_CHECK_EN Bit Value */  
#define SDIO_IE_BLOCK_DONE_EN                BIT(9)                                      /*!< sdio blcok done */
#define SDIO_IE_BLOCK_DONE_EN_OFS            9U                                          /*!< SDIO IE: BLOCK_DONE_EN Bit Offset */
#define SDIO_IE_BLOCK_DONE_EN_VAL(regval)        (BIT(9) & ((uint32_t)(regval) << 9))        /*!< SDIO IE: BLOCK_DONE_EN Bit Value */  
#define SDIO_IE_BUSY0_END_EN                 BIT(10)                                      /*!< sdio cmd5 done enable */
#define SDIO_IE_BUSY0_END_EN_OFS             10U                                          /*!< SDIO IE: BUSY0_END_EN Bit Offset */
#define SDIO_IE_BUSY0_END_EN_VAL(regval)         (BIT(10) & ((uint32_t)(regval) << 10))        /*!< SDIO IE: BUSY0_END_EN Bit Value */  
#define SDIO_IE_VOLTAGE_SWITCH_PASS_EN         BIT(11)                                      /*!< sdio voltage switch pass enable */
#define SDIO_IE_VOLTAGE_SWITCH_PASS_EN_OFS     11U                                          /*!< SDIO IE: VOLTAGE_SWITCH_PASS_EN Bit Offset */
#define SDIO_IE_VOLTAGE_SWITCH_PASS_EN_VAL(regval) (BIT(11) & ((uint32_t)(regval) << 11))        /*!< SDIO IE: VOLTAGE_SWITCH_PASS_EN Bit Value */  
#define SDIO_IE_VOLTAGE_SWITCH_FAIL_EN         BIT(12)                                      /*!< sdio voltage switch fail enable */
#define SDIO_IE_VOLTAGE_SWITCH_FAIL_EN_OFS     12U                                          /*!< SDIO IE: VOLTAGE_SWITCH_FAIL_EN Bit Offset */
#define SDIO_IE_VOLTAGE_SWITCH_FAIL_EN_VAL(regval) (BIT(12) & ((uint32_t)(regval) << 12))        /*!< SDIO IE: VOLTAGE_SWITCH_FAIL_EN Bit Value */  
#define SDIO_IE_BOOT_ACK_PASS_EN             BIT(13)                                      /*!< sdio boot ack pass enable */
#define SDIO_IE_BOOT_ACK_PASS_EN_OFS         13U                                          /*!< SDIO IE: BOOT_ACK_PASS_EN Bit Offset */
#define SDIO_IE_BOOT_ACK_PASS_EN_VAL(regval)     (BIT(13) & ((uint32_t)(regval) << 13))        /*!< SDIO IE: BOOT_ACK_PASS_EN Bit Value */  
#define SDIO_IE_BOOT_ACK_FAIL_EN             BIT(14)                                      /*!< sdio boot ack fail enable */
#define SDIO_IE_BOOT_ACK_FAIL_EN_OFS         14U                                          /*!< SDIO IE: BOOT_ACK_FAIL_EN Bit Offset */
#define SDIO_IE_BOOT_ACK_FAIL_EN_VAL(regval)     (BIT(14) & ((uint32_t)(regval) << 14))        /*!< SDIO IE: BOOT_ACK_FAIL_EN Bit Value */  
 
 /* ===== SDIO SAMPLE_DDR Register definition ===== */
#define SDIO_SAMPLE_DDR_DELAY_MASK                   BITS(0,31)                                   /*!< SDIO SAMPLE DDR: DELAY Bit Mask */  
#define SDIO_SAMPLE_DDR_DELAY_OFS                    0U                                          /*!< SDIO SAMPLE DDR: DELAY Bit Offset */
#define SDIO_SAMPLE_DDR_DELAY(regval)                (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO SAMPLE DDR: DELAY Bit Value */  
 
 /* ===== SDIO RX_SADDR_HI Register definition ===== */
#define SDIO_RX_SADDR_HI_ADDRESS_MASK                 BITS(0,31)                                   /*!< SDIO RX SADDR HI: ADDRESS Bit Mask */  
#define SDIO_RX_SADDR_HI_ADDRESS_OFS                  0U                                          /*!< SDIO RX SADDR HI: ADDRESS Bit Offset */
#define SDIO_RX_SADDR_HI_ADDRESS(regval)              (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO RX SADDR HI: ADDRESS Bit Value */  
 
 /* ===== SDIO TX_SADDR_HI Register definition ===== */
#define SDIO_TX_SADDR_HI_ADDRESS_MASK                 BITS(0,31)                                   /*!< SDIO TX SADDR HI: ADDRESS Bit Mask */  
#define SDIO_TX_SADDR_HI_ADDRESS_OFS                  0U                                          /*!< SDIO TX SADDR HI: ADDRESS Bit Offset */
#define SDIO_TX_SADDR_HI_ADDRESS(regval)              (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO TX SADDR HI: ADDRESS Bit Value */  
 
 /* ===== SDIO DATA_TX_DELAY Register definition ===== */
#define SDIO_DATA_TX_DELAY_CNT_MASK                     BITS(0,31)                                   /*!< SDIO DATA TX DELAY: CNT Bit Mask */  
#define SDIO_DATA_TX_DELAY_CNT_OFS                      0U                                          /*!< SDIO DATA TX DELAY: CNT Bit Offset */
#define SDIO_DATA_TX_DELAY_CNT(regval)                  (BITS(0,31) & ((uint32_t)(regval) << 0))        /*!< SDIO DATA TX DELAY: CNT Bit Value */  
 
 /* ===== SDIO DATA_CRC_TOKEN Register definition ===== */
#define SDIO_DATA_CRC_TOKEN_CNT_MASK                     BITS(0,15)                                   /*!< SDIO DATA CRC TOKEN: CNT Bit Mask */  
#define SDIO_DATA_CRC_TOKEN_CNT_OFS                      0U                                          /*!< SDIO DATA CRC TOKEN: CNT Bit Offset */
#define SDIO_DATA_CRC_TOKEN_CNT(regval)                  (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< SDIO DATA CRC TOKEN: CNT Bit Value */  
 
 /* ===== SDIO CRC_VALUE Register definition ===== */
#define SDIO_CRC_VALUE_NORMAL_CMD                   BITS(0,6)                
#define SDIO_CRC_VALUE_STOP_CMD                     BITS(8,14)                
 
 /* ===== SDIO STATUS1 Register definition ===== */
#define SDIO_STATUS1_BLOCK_NUM                    BITS(0,15)                
 
 /* ===== SDIO STOP Register definition ===== */
#define SDIO_STOP_ENABLE                       BIT(0)                                      /*!< stop transmission, write 1 to generate pulse and reset automatically */
#define SDIO_STOP_ENABLE_OFS                   0U                                          /*!< SDIO STOP: ENABLE Bit Offset */
#define SDIO_STOP_ENABLE_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< SDIO STOP: ENABLE Bit Value */  
 
 /* ===== SDIO PAD_CTRL Register definition ===== */
#define SDIO_PAD_CTRL_ENABLE_MASK                  BITS(0,9)                                   /*!< SDIO PAD CTRL: ENABLE Bit Mask */  
#define SDIO_PAD_CTRL_ENABLE_OFS                   0U                                          /*!< SDIO PAD CTRL: ENABLE Bit Offset */
#define SDIO_PAD_CTRL_ENABLE(regval)               (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< SDIO PAD CTRL: ENABLE Bit Value */  
 
 /* ===== SDIO STOP_RSP0 Register definition ===== */
#define SDIO_STOP_RSP0_NUM                          BITS(0,31)                
 
 /* ===== SDIO STOP_RSP1 Register definition ===== */
#define SDIO_STOP_RSP1_NUM                          BITS(0,31)                
 
 /* ===== SDIO STOP_RSP2 Register definition ===== */
#define SDIO_STOP_RSP2_NUM                          BITS(0,31)                
 
 /* ===== SDIO STOP_RSP3 Register definition ===== */
#define SDIO_STOP_RSP3_NUM                          BITS(0,31)                

/* ------------------------------------------- Config functions ------------------------------------------- */
void SDIO_StructInit(SDIO_InitTypeDef *sdio_init);
void SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef *sdio_init);

void SDIO_CmdInitStructInit(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_DataSetupStructInit(SDIO_DataSetupTypeDef *SDIO_DataSetupStruct);
void SDIO_DataSetup(SDIO_TypeDef *SDIOx, SDIO_DataSetupTypeDef *SDIO_DataSetupStruct);
void SDIO_ClockDivConfig( SDIO_TypeDef *SDIOx, uint32_t  clkdiv);
void SDIO_ClearDataSetup(SDIO_TypeDef *SDIOx );
void SDIO_SetDateTimeout(SDIO_TypeDef *SDIOx, uint32_t timeout);

uint32_t SDIO_GetFlagStatus(SDIO_TypeDef *SDIOx, uint32_t interrupt);
uint32_t SDIO_GetInterruptEn(SDIO_TypeDef *SDIOx, uint32_t status);
void SDIO_ClearFlag(SDIO_TypeDef *SDIOx, uint32_t status);
uint32_t SDIO_GetIPStatus(SDIO_TypeDef *SDIOx, uint32_t status);
uint32_t SDIO_SetTxMark(SDIO_TypeDef *SDIOx, uint32_t depth);
uint32_t SDIO_SetRxMark(SDIO_TypeDef *SDIOx, uint32_t depth);
void SDIO_SetCmdPowerUpCnt(SDIO_TypeDef *SDIOx, uint32_t count);

void SDIO_TxRxConfig(SDIO_TypeDef *SDIOx, uint32_t dir, uint32_t buswidth, uint32_t blockcnt, uint32_t blocksize, uint32_t mode);
void SDIO_UdmaClearChannel(SDIO_TypeDef* SDIOx, uint32_t channel);
void SDIO_UdmaEnqueueChannel(SDIO_TypeDef* SDIOx, uint32_t* addr, uint32_t size, uint32_t config, uint32_t channel);
void SDIO_UdmaListModeEnqueueChannel(SDIO_TypeDef* SDIOx, SDIO_DmaConfigListTypeDef dma_cfg_list);

/* ------------------------------------------- Control functions ------------------------------------------- */
void SDIO_SendCommand(SDIO_TypeDef *SDIOx, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_StopClkEn(SDIO_TypeDef *SDIOx, ControlStatus control);
void SDIO_ReadWaitEn(SDIO_TypeDef *SDIOx, ControlStatus control);
void SDIO_DmaStart(SDIO_TypeDef *SDIOx, ControlStatus control);
void SDIO_CfgDdrMode(SDIO_TypeDef *SDIOx, ControlStatus control);
uint32_t SDIO_InterruptEn(SDIO_TypeDef *SDIOx, SDIO_IntTypedef interrupt, ControlStatus control);
uint32_t SDIO_DmaInterruptEn(SDIO_TypeDef *SDIOx, SDIO_DmaIntTypedef interrupt, ControlStatus control);
uint32_t SDIO_DmaInterruptClr(SDIO_TypeDef *SDIOx, SDIO_DmaIntClrTypedef interrupt_clr);
uint32_t SDIO_DmaGetIntStat(SDIO_TypeDef *SDIOx, SDIO_DmaIntStatTypedef status);
void SDIO_SendData(SDIO_TypeDef *SDIOx, uint32_t data);
uint32_t SDIO_ReadData(SDIO_TypeDef *SDIOx);

void SDIO_PowerState_ON(SDIO_TypeDef *SDIOx);
void SDIO_PowerState_OFF(SDIO_TypeDef *SDIOx);
uint32_t SDIO_GetPowerState(SDIO_TypeDef *SDIOx);

/* ------------------------------------------- CMD functions ------------------------------------------- */
/* SDIO Commands management functions */
uint32_t SDIO_CmdBlockLength(SDIO_TypeDef *SDIOx, uint32_t BlockSize);
uint32_t SDIO_CmdMultiBlockCount(SDIO_TypeDef *SDIOx, uint32_t BlockCount);
uint32_t SDIO_CmdReadSingleBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDIO_CmdReadMultiBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDIO_CmdWriteSingleBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDIO_CmdWriteMultiBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDIO_CmdEraseStartAdd(SDIO_TypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDIO_CmdSDEraseStartAdd(SDIO_TypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDIO_CmdEraseEndAdd(SDIO_TypeDef *SDIOx, uint32_t EndAdd);
uint32_t SDIO_CmdSDEraseEndAdd(SDIO_TypeDef *SDIOx, uint32_t EndAdd);
uint32_t SDIO_CmdErase(SDIO_TypeDef *SDIOx, uint32_t EraseType);
uint32_t SDIO_CmdStopTransfer(SDIO_TypeDef *SDIOx);
uint32_t SDIO_CmdSelDesel(SDIO_TypeDef *SDIOx, uint64_t Addr);
uint32_t SDIO_CmdGoIdleState(SDIO_TypeDef *SDIOx);
uint32_t SDIO_CmdOperCond(SDIO_TypeDef *SDIOx);
uint32_t SDIO_CmdAppCommand(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDIO_CmdAppOperCommand(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response);
uint32_t SDIO_CmdBusWidth(SDIO_TypeDef *SDIOx, uint32_t BusWidth);
uint32_t SDIO_CmdSendSCR(SDIO_TypeDef *SDIOx);
uint32_t SDIO_CmdSendCID(SDIO_TypeDef *SDIOx, uint32_t *CID);
uint32_t SDIO_CmdSendCSD(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *CSD);
uint32_t SDIO_CmdSetRelAdd(SDIO_TypeDef *SDIOx, uint32_t RCA);
uint32_t SDIO_CmdGetRelAdd(SDIO_TypeDef *SDIOx, uint16_t *pRCA);
uint32_t SDIO_CmdSendStatus(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response);
uint32_t SDIO_CmdStatusRegister(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDIO_CmdVoltageSwitch(SDIO_TypeDef *SDIOx);
uint32_t SDIO_CmdOpCondition(SDIO_TypeDef *SDIOx, uint32_t Argument, uint32_t *Response);
uint32_t SDIO_CmdSwitch(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDIO_CmdSendEXTCSD(SDIO_TypeDef *SDIOx, uint32_t Argument);

void SDIO_SendCommand(SDIO_TypeDef *SDIOx, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
uint32_t SDIO_GetResponse(SDIO_TypeDef *SDIOx, uint32_t Response);
#ifdef __cplusplus
    }
#endif

