
/* Define to prevent recursive inclusion -------------------------------------*/

#include "ns_sdk_hal.h"

#ifndef _NS_SDMMC_H_
#define _NS_SDMMC_H_

/*!
 * @file     ns_sdmmc.h
 * @brief    This file contains all the functions prototypes for the SDMMC firmware
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
    HAL_SDMMC_OK       = 0x00,
    HAL_SDMMC_ERR      = 0x01,
    HAL_SDMMC_BUSY     = 0x02,
    HAL_SDMMC_TIMEOUT  = 0x03
} HAL_SDMMCStatusTypeDef;

/** @defgroup SDMMC_Exported_Constansts_Group1 SDMMC Error status enumeration Structure definition
  * @{
  */
#define HAL_SDMMC_ERROR_NONE                     SDIO_ERROR_NONE                        /*!< No error                                                     */
#define HAL_SDMMC_ERROR_CMD_CRC_FAIL             SDIO_ERROR_CMD_CRC_FAIL                /*!< Command response received (but CRC check failed)             */
#define HAL_SDMMC_ERROR_DATA_CRC_FAIL            SDIO_ERROR_DATA_CRC_FAIL               /*!< Data block sent/received (CRC check failed)                  */
#define HAL_SDMMC_ERROR_CMD_RSP_TIMEOUT          SDIO_ERROR_CMD_RSP_TIMEOUT             /*!< Command response timeout                                     */
#define HAL_SDMMC_ERROR_DATA_TIMEOUT             SDIO_ERROR_DATA_TIMEOUT                /*!< Data timeout                                                 */
#define HAL_SDMMC_ERROR_TX_UNDERRUN              SDIO_ERROR_TX_UNDERRUN                 /*!< Transmit FIFO underrun                                       */
#define HAL_SDMMC_ERROR_RX_OVERRUN               SDIO_ERROR_RX_OVERRUN                  /*!< Receive FIFO overrun                                         */
#define HAL_SDMMC_ERROR_ADDR_MISALIGNED          SDIO_ERROR_ADDR_MISALIGNED             /*!< Misaligned address                                           */
#define HAL_SDMMC_ERROR_BLOCK_LEN_ERR            SDIO_ERROR_BLOCK_LEN_ERR               /*!< Transferred block length is not allowed for the card or the
                                                                                          number of transferred bytes does not match the block length  */
#define HAL_SDMMC_ERROR_ERASE_SEQ_ERR            SDIO_ERROR_ERASE_SEQ_ERR               /*!< An error in the sequence of erase command occurs             */
#define HAL_SDMMC_ERROR_BAD_ERASE_PARAM          SDIO_ERROR_BAD_ERASE_PARAM             /*!< An invalid selection for erase groups                        */
#define HAL_SDMMC_ERROR_WRITE_PROT_VIOLATION     SDIO_ERROR_WRITE_PROT_VIOLATION        /*!< Attempt to program a write protect block                     */
#define HAL_SDMMC_ERROR_LOCK_UNLOCK_FAILED       SDIO_ERROR_LOCK_UNLOCK_FAILED          /*!< Sequence or password error has been detected in unlock
                                                                                           command or if there was an attempt to access a locked card  */
#define HAL_SDMMC_ERROR_COM_CRC_FAILED           SDIO_ERROR_COM_CRC_FAILED              /*!< CRC check of the previous command failed                     */
#define HAL_SDMMC_ERROR_ILLEGAL_CMD              SDIO_ERROR_ILLEGAL_CMD                 /*!< Command is not legal for the card state                      */
#define HAL_SDMMC_ERROR_CARD_ECC_FAILED          SDIO_ERROR_CARD_ECC_FAILED             /*!< Card internal ECC was applied but failed to correct the data */
#define HAL_SDMMC_ERROR_CC_ERR                   SDIO_ERROR_CC_ERR                      /*!< Internal card controller error                               */
#define HAL_SDMMC_ERROR_GENERAL_UNKNOWN_ERR      SDIO_ERROR_GENERAL_UNKNOWN_ERR         /*!< General or unknown error                                     */
#define HAL_SDMMC_ERROR_STREAM_READ_UNDERRUN     SDIO_ERROR_STREAM_READ_UNDERRUN        /*!< The card could not sustain data reading in stream rmode      */
#define HAL_SDMMC_ERROR_STREAM_WRITE_OVERRUN     SDIO_ERROR_STREAM_WRITE_OVERRUN        /*!< The card could not sustain data programming in stream mode   */
#define HAL_SDMMC_ERROR_CID_CSD_OVERWRITE        SDIO_ERROR_CID_CSD_OVERWRITE           /*!< CID/CSD overwrite error                                      */
#define HAL_SDMMC_ERROR_WP_ERASE_SKIP            SDIO_ERROR_WP_ERASE_SKIP               /*!< Only partial address space was erased                        */
#define HAL_SDMMC_ERROR_CARD_ECC_DISABLED        SDIO_ERROR_CARD_ECC_DISABLED           /*!< Command has been executed without using internal ECC         */
#define HAL_SDMMC_ERROR_ERASE_RESET              SDIO_ERROR_ERASE_RESET                 /*!< Erase sequence was cleared before executing because an out
                                                                                           of erase sequence command was received                      */
#define HAL_SDMMC_ERROR_AKE_SEQ_ERR              SDIO_ERROR_AKE_SEQ_ERR                 /*!< Error in sequence of authentication                          */
#define HAL_SDMMC_ERROR_INVALID_VOLTRANGE        SDIO_ERROR_INVALID_VOLTRANGE           /*!< Error in case of invalid voltage range                       */
#define HAL_SDMMC_ERROR_ADDR_OUT_OF_RANGE        SDIO_ERROR_ADDR_OUT_OF_RANGE           /*!< Error when addressed block is out of range                   */
#define HAL_SDMMC_ERROR_REQUEST_NOT_APPLICABLE   SDIO_ERROR_REQUEST_NOT_APPLICABLE      /*!< Error when command request is not applicable                 */
#define HAL_SDMMC_ERROR_PARAM                    SDIO_ERROR_INVALID_PARAMETER           /*!< the used parameter is not valid                              */
#define HAL_SDMMC_ERROR_UNSUPPORTED_FEATURE      SDIO_ERROR_UNSUPPORTED_FEATURE         /*!< Error when feature is not insupported                        */
#define HAL_SDMMC_ERROR_BUSY                     SDIO_ERROR_BUSY                        /*!< Error when transfer process is busy                          */
#define HAL_SDMMC_ERROR_DMA                      SDIO_ERROR_DMA                         /*!< Error while DMA transfer                                     */
#define HAL_SDMMC_ERROR_TIMEOUT                  SDIO_ERROR_TIMEOUT                     /*!< Timeout error                                                */

/** @defgroup SDMMC_Exported_Constansts_Group2 SDMMC context enumeration
  * @{
  */
#define   SDMMC_CONTEXT_NONE                 ((uint32_t)0x00000000U)  /*!< None                             */
#define   SDMMC_CONTEXT_READ_SINGLE_BLOCK    ((uint32_t)0x00000001U)  /*!< Read single block operation      */
#define   SDMMC_CONTEXT_READ_MULTIPLE_BLOCK  ((uint32_t)0x00000002U)  /*!< Read multiple blocks operation   */
#define   SDMMC_CONTEXT_WRITE_SINGLE_BLOCK   ((uint32_t)0x00000010U)  /*!< Write single block operation     */
#define   SDMMC_CONTEXT_WRITE_MULTIPLE_BLOCK ((uint32_t)0x00000020U)  /*!< Write multiple blocks operation  */
#define   SDMMC_CONTEXT_IT                   ((uint32_t)0x00000008U)  /*!< Process in Interrupt mode        */
#define   SDMMC_CONTEXT_DMA                  ((uint32_t)0x00000080U)  /*!< Process in DMA mode              */

/** @defgroup SDMMC_Exported_Types_Group1 SDMMC State enumeration structure
  * @{
  */
typedef enum
{
    HAL_SDMMC_STATE_RESET                  = ((uint32_t)0x00000000U),  /*!< SDMMC not yet initialized or disabled  */
    HAL_SDMMC_STATE_READY                  = ((uint32_t)0x00000001U),  /*!< SDMMC initialized and ready for use    */
    HAL_SDMMC_STATE_TIMEOUT                = ((uint32_t)0x00000002U),  /*!< SDMMC Timeout state                    */
    HAL_SDMMC_STATE_BUSY                   = ((uint32_t)0x00000003U),  /*!< SDMMC process ongoing                  */
    HAL_SDMMC_STATE_PROGRAMMING            = ((uint32_t)0x00000004U),  /*!< SDMMC Programming State                */
    HAL_SDMMC_STATE_RECEIVING              = ((uint32_t)0x00000005U),  /*!< SDMMC Receiving State                  */
    HAL_SDMMC_STATE_TRANSFER               = ((uint32_t)0x00000006U),  /*!< SDMMC Transfert State                  */
    HAL_SDMMC_STATE_ERROR                  = ((uint32_t)0x0000000FU)   /*!< SDMMC is in error state                */
} HAL_SDMMC_StateTypeDef;

/** @defgroup SDMMC_Exported_Types_Group2 SDMMC Card State structure
  * @{
  */

#define HAL_SDMMC_READY          0x00000001U  /*!< Card state is ready                     */
#define HAL_SDMMC_IDENTIFICATION 0x00000002U  /*!< Card is in identification state         */
#define HAL_SDMMC_STANDBY        0x00000003U  /*!< Card is in standby state                */
#define HAL_SDMMC_TRANSFER       0x00000004U  /*!< Card is in transfer state               */
#define HAL_SDMMC_SENDING        0x00000005U  /*!< Card is sending an operation            */
#define HAL_SDMMC_RECEIVING      0x00000006U  /*!< Card is receiving operation information */
#define HAL_SDMMC_PROGRAMMING    0x00000007U  /*!< Card is in programming state            */
#define HAL_SDMMC_DISCONNECTED   0x00000008U  /*!< Card is disconnected                    */
#define HAL_SDMMC_ERROR          0x000000FFU  /*!< Card response Error                     */

/**
  * @brief  SDMMC Card Information Structure definition
  */
typedef struct
{
    uint32_t CardType;                     /*!< Specifies the card Type                         */
    uint32_t CardVersion;                  /*!< Specifies the card version                      */
    uint32_t Class;                        /*!< Specifies the class of the card class           */
    uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */
    uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */
    uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */
    uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */
    uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */
    uint32_t CardSpeed;                    /*!< Specifies the card Speed                        */
} HAL_SDMMC_CardInfoTypeDef;

/**
  * @brief  SDIO Config Structure definition
  */
typedef struct
{
    uint32_t CardType;
    uint32_t DeviceMode;
    uint32_t BusWidth;
    uint32_t BusMode;
} SDIO_CfgTypeDef;

/**
  * @brief  SDMMC handle Structure definition
  */
typedef struct
{
    SDIO_TypeDef                    *Instance;        /*!< SDIO registers base address            */
    SDIO_InitTypeDef                Sdio_init;        /*!< SDIO init required parameters          */
    SDIO_CfgTypeDef                 Sdio_cfg;         /*!< SDIO & SDMMC configuration parameters  */
    volatile HAL_SDMMC_StateTypeDef State;            /*!< SDMMC card State                       */
    volatile uint32_t               ErrorCode;        /*!< SDMMC Card Error codes                 */
    volatile uint32_t               Context;          /*!< SDMMC transfer context                 */
    uint32_t                        CardStatus;       /*!< SDMMC Card status                      */
    HAL_SDMMC_CardInfoTypeDef       CardInfo;         /*!< SDMMC Card information                 */
    uint32_t                        CSD[4];           /*!< SDMMC card specific data table         */
    uint32_t                        CID[4];           /*!< SDMMC card identification number table */
    uint32_t                        MMCExtCSD[128];   /*!< MMC card Extended CSD register table   */
    uint32_t                        SDCardStatus[8];  /*!< SD card application-specific status    */
} SDMMC_HandleTypeDef;

/** @defgroup SDMMC_Exported_Types_Group6 SD Card Status returned by ACMD13
  * @{
  */
typedef struct
{
    __IO uint8_t  DataBusWidth;           /*!< Shows the currently defined data bus width                 */
    __IO uint8_t  SecuredMode;            /*!< Card is in secured mode of operation                       */
    __IO uint16_t CardType;               /*!< Carries information about card type                        */
    __IO uint32_t ProtectedAreaSize;      /*!< Carries information about the capacity of protected area   */
    __IO uint8_t  SpeedClass;             /*!< Carries information about the speed class of the card      */
    __IO uint8_t  PerformanceMove;        /*!< Carries information about the card's performance move      */
    __IO uint8_t  AllocationUnitSize;     /*!< Carries information about the card's allocation unit size  */
    __IO uint16_t EraseSize;              /*!< Determines the number of AUs to be erased in one operation */
    __IO uint8_t  EraseTimeout;           /*!< Determines the timeout for any number of AU erase          */
    __IO uint8_t  EraseOffset;            /*!< Carries information about the erase offset                 */
    __IO uint8_t  UhsSpeedGrade;          /*!< Carries information about the speed grade of UHS card      */
    __IO uint8_t  UhsAllocationUnitSize;  /*!< Carries information about the UHS card's allocation unit size  */
    __IO uint8_t  VideoSpeedClass;        /*!< Carries information about the Video Speed Class of UHS card    */
} HAL_SD_CardStatusTypeDef;

/** @defgroup SD_Exported_Types_Group4 Card Specific Data: CSD Register
  * @{
  */
typedef struct
{
    __IO uint8_t  CSDStruct;            /*!< CSD structure                         */
    __IO uint8_t  SysSpecVersion;       /*!< System specification version          */
    __IO uint8_t  Reserved1;            /*!< Reserved                              */
    __IO uint8_t  TAAC;                 /*!< Data read access time 1               */
    __IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
    __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
    __IO uint16_t CardComdClasses;      /*!< Card command classes                  */
    __IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
    __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
    __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
    __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
    __IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
    __IO uint8_t  Reserved2;            /*!< Reserved                              */
    __IO uint32_t DeviceSize;           /*!< Device Size                           */
    __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
    __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
    __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
    __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
    __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
    __IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
    __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
    __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
    __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
    __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
    __IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
    __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
    __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
    __IO uint8_t  Reserved3;            /*!< Reserved                              */
    __IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
    __IO uint8_t  FileFormatGroup;      /*!< File format group                     */
    __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
    __IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
    __IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
    __IO uint8_t  FileFormat;           /*!< File format                           */
    __IO uint8_t  ECC;                  /*!< ECC code                              */
    __IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
    __IO uint8_t  Reserved4;            /*!< Always 1                              */
} HAL_SDMMC_CardCSDTypeDef;

/** @defgroup SD_Exported_Types_Group5 Card Identification Data: CID Register
  * @{
  */
typedef struct
{
    __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
    __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
    __IO uint32_t ProdName1;       /*!< Product Name part1    */
    __IO uint8_t  ProdName2;       /*!< Product Name part2    */
    __IO uint8_t  ProdRev;         /*!< Product Revision      */
    __IO uint32_t ProdSN;          /*!< Product Serial Number */
    __IO uint8_t  Reserved1;       /*!< Reserved1             */
    __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
    __IO uint8_t  CID_CRC;         /*!< CID CRC               */
    __IO uint8_t  Reserved2;       /*!< Always 1              */
} HAL_SDMMC_CardCIDTypeDef;

typedef union {
    struct _EXT_CSD {
        uint8_t Reserved26[32];
        uint8_t FLUSH_CACHE;
        uint8_t CACHE_CTRL;
        uint8_t POWER_OFF_NOTIFICATION;
        uint8_t PACKED_FAILURE_INDEX;
        uint8_t PACKED_COMMAND_STATUS;
        uint8_t CONTEXT_CONF[15];
        uint8_t EXT_PARTITIONS_ATTRIBUTE[2];
        uint8_t EXCEPTION_EVENTS_STATUS[2];
        uint8_t EXCEPTION_EVENTS_CTRL[2];
        uint8_t DYNCAP_NEEDED;
        uint8_t CLASS_6_CTRL;
        uint8_t INI_TIMEOUT_EMU;
        uint8_t DATA_SECTOR_SIZE;
        uint8_t USE_NATIVE_SECTOR;
        uint8_t NATIVE_SECTOR_SIZE;
        uint8_t VENDOR_SPECIFIC_FIELD[64];
        uint8_t Reserved25;
        uint8_t PROGRAM_CID_CSD_DDR_SUPPORT;
        uint8_t PERIODIC_WAKEUP;
        uint8_t TCASE_SUPPORT;
        uint8_t Reserved24;
        uint8_t SEC_BAD_BLK_MGMNT;
        uint8_t Reserved23;
        uint8_t ENH_START_ADDR[4];
        uint8_t ENH_SIZE_MULT[3];
        uint8_t GP_SIZE_MULT[12];
        uint8_t PARTITION_SETTING_COMPLETED;
        uint8_t PARTITIONS_ATTRIBUTE;
        uint8_t MAX_ENH_SIZE_MULT[3];
        uint8_t PARTITIONING_SUPPORT;
        uint8_t HPI_MGMT;
        uint8_t RST_n_FUNCTION;
        uint8_t BKOPS_EN;
        uint8_t BKOPS_START;
        uint8_t SANITIZE_START;
        uint8_t WR_REL_PARAM;
        uint8_t WR_REL_SET;
        uint8_t RPMB_SIZE_MULT;
        uint8_t FW_CONFIG;
        uint8_t Reserved22;
        uint8_t USER_WP;
        uint8_t Reserved21;
        uint8_t BOOT_WP;
        uint8_t BOOT_WP_STATUS;
        uint8_t ERASE_GROUP_DEF;
        uint8_t Reserved20;
        uint8_t BOOT_BUS_CONDITIONS;
        uint8_t BOOT_CONFIG_PROT;
        uint8_t PARTITION_CONFIG;
        uint8_t Reserved19;
        uint8_t ERASED_MEM_CONT;
        uint8_t Reserved18;
        uint8_t BUS_WIDTH;
        uint8_t Reserved17;
        uint8_t HS_TIMING;
        uint8_t Reserved16;
        uint8_t POWER_CLASS;
        uint8_t Reserved15;
        uint8_t CMD_SET_REV;
        uint8_t Reserved14;
        uint8_t CMD_SET;
        uint8_t Reserved13;
        uint8_t EXT_CSD_REV;
        uint8_t Reserved12;
        uint8_t Reserved11;
        uint8_t Reserved10;
        uint8_t DEVICE_TYPE;
        uint8_t DRIVER_STRENGTH;
        uint8_t OUT_OF_INTERRUPT_TIME;
        uint8_t PARTITION_SWITCH_TIME;
        uint8_t PWR_CL_52_195;
        uint8_t PWR_CL_26_195;
        uint8_t PWR_CL_52_360;
        uint8_t PWR_CL_26_360;
        uint8_t Reserved9;
        uint8_t MIN_PERF_R_4_26;
        uint8_t MIN_PERF_W_4_26;
        uint8_t MIN_PERF_R_8_26_4_52;
        uint8_t MIN_PERF_W_8_26_4_52;
        uint8_t MIN_PERF_R_8_52;
        uint8_t MIN_PERF_W_8_52;
        uint8_t Reserved8;
        uint8_t SEC_COUNT[4];
        uint8_t Reserved7;
        uint8_t S_A_TIMEOUT;
        uint8_t Reserved6;
        uint8_t S_C_VCCQ;
        uint8_t S_C_VCC;
        uint8_t HC_WP_GRP_SIZE;
        uint8_t REL_WR_SEC_C;
        uint8_t ERASE_TIMEOUT_MULT;
        uint8_t HC_ERASE_GRP_SIZE;
        uint8_t ACC_SIZE;
        uint8_t BOOT_SIZE_MULTI;
        uint8_t Reserved5;
        uint8_t BOOT_INFO;
        uint8_t obsolete2;
        uint8_t obsolete1;
        uint8_t SEC_FEATURE_SUPPORT;
        uint8_t TRIM_MULT;
        uint8_t Reserved4;
        uint8_t MIN_PERF_DDR_R_8_52;
        uint8_t MIN_PERF_DDR_W_8_52;
        uint8_t PWR_CL_200_195;
        uint8_t PWR_CL_200_360;
        uint8_t PWR_CL_DDR_52_195;
        uint8_t PWR_CL_DDR_52_360;
        uint8_t Reserved3;
        uint8_t INI_TIMEOUT_AP;
        uint8_t CORRECTLY_PRG_SECTORS_NUM[4];
        uint8_t BKOPS_STATUS[2];
        uint8_t POWER_OFF_LONG_TIME;
        uint8_t GENERIC_CMD6_TIME;
        uint8_t CACHE_SIZE[4];
        uint8_t Reserved2[255];
        uint8_t EXT_SUPPORT;
        uint8_t LARGE_UNIT_SIZE_M1;
        uint8_t CONTEXT_CAPABILITIES;
        uint8_t TAG_RES_SIZE;
        uint8_t TAG_UNIT_SIZE;
        uint8_t DATA_TAG_SUPPORT;
        uint8_t MAX_PACKED_WRITES;
        uint8_t MAX_PACKED_READS;
        uint8_t BKOPS_SUPPORT;
        uint8_t HPI_FEATURES;
        uint8_t S_CMD_SET;
        uint8_t Reserved1[7];
    } EXT_CSD;
    uint8_t CsdBuf[512];
} HAL_MMC_CardEXT_CSDTypeDef;

typedef struct {
    HAL_SDMMC_CardCIDTypeDef CardCID;
    HAL_SDMMC_CardCSDTypeDef CardCSD;
    HAL_MMC_CardEXT_CSDTypeDef MMCExtCSD;
    HAL_SD_CardStatusTypeDef SDCardStatus;
    uint64_t CardCapacity;  /*!< Card Capacity */
    uint32_t CardBlockSize; /*!< Card Block Size */
    uint16_t RCA;
    uint8_t CardType;
} SDMMC_CardInfoTypeDef;

/**
  * @brief Supported SDMMC Memory Cards
  */
#define SDIO_STD_CAPACITY_SD_CARD_V1_1             ((uint32_t)0x00000000)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0             ((uint32_t)0x00000001)
#define SDIO_HIGH_CAPACITY_SD_CARD                 ((uint32_t)0x00000002)
#define SDIO_MULTIMEDIA_CARD                       ((uint32_t)0x00000003)
#define SDIO_SECURE_DIGITAL_IO_CARD                ((uint32_t)0x00000004)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD            ((uint32_t)0x00000005)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD          ((uint32_t)0x00000006)
#define SDIO_HIGH_CAPACITY_MMC_CARD                ((uint32_t)0x00000007)

#define BLOCKSIZE   ((uint32_t)512U) /*!< Block size is 512 bytes */

#define ERASE_TYPE_BLOCK            ((uint32_t)0x00000000)
#define ERASE_TYPE_DISCARD          ((uint32_t)0x00000001)
#define ERASE_TYPE_FULE             ((uint32_t)0x00000002)

/** @defgroup SDMMC_Exported_Constansts_Group3 SDMMC Supported Memory Cards
  * @{
  */
#define CARD_NORMAL_SPEED        ((uint32_t)0x00000000U)    /*!< Normal Speed Card <12.5Mo/s , Spec Version 1.01    */
#define CARD_HIGH_SPEED          ((uint32_t)0x00000100U)    /*!< High Speed Card <25Mo/s , Spec version 2.00        */
#define CARD_ULTRA_HIGH_SPEED    ((uint32_t)0x00000200U)    /*!< UHS-I SD Card <50Mo/s for SDR50, DDR5 Cards
                                                                 and <104Mo/s for SDR104, Spec version 3.01          */

#define CARD_SDSC                  ((uint32_t)0x00000000U)  /*!< SD Standard Capacity <2Go                          */
#define CARD_SDHC_SDXC             ((uint32_t)0x00000001U)  /*!< SD High Capacity <32Go, SD Extended Capacity <2To  */
#define CARD_SECURED               ((uint32_t)0x00000003U)

#define MMC_LOW_CAPACITY_CARD     ((uint32_t)0x00000005U)   /*!< MMC Card Capacity <=2Gbytes   */
#define MMC_HIGH_CAPACITY_CARD    ((uint32_t)0x00000007U)   /*!< MMC Card Capacity >2Gbytes and <2Tbytes   */

/** @defgroup SDMMC_Exported_Constansts_Group4 SDMMC Supported Version
  * @{
  */
#define CARD_V1_X                  ((uint32_t)0x00000000U)
#define CARD_V2_X                  ((uint32_t)0x00000001U)

/** @defgroup MMC_Exported_Constansts_Group3 MMC Voltage mode
  * @{
  */
#define MMC_HIGH_VOLTAGE_RANGE         0x80FF8000U  /*!< VALUE OF ARGUMENT            */
#define MMC_DUAL_VOLTAGE_RANGE         0x80FF8080U  /*!< VALUE OF ARGUMENT            */
#define eMMC_HIGH_VOLTAGE_RANGE        0xC0FF8000U  /*!< for eMMC > 2Gb sector mode   */
#define eMMC_DUAL_VOLTAGE_RANGE        0xC0FF8080U  /*!< for eMMC > 2Gb sector mode   */
#define MMC_INVALID_VOLTAGE_RANGE      0x0001FF01U

/**
  * @brief  Mask for errors Card Status R1 (OCR Register)
  */
#define SDMMC_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SDMMC_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SDMMC_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SDMMC_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SDMMC_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SDMMC_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SDMMC_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SDMMC_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SDMMC_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SDMMC_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SDMMC_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SDMMC_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SDMMC_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SDMMC_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SDMMC_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SDMMC_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SDMMC_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SDMMC_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SDMMC_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SDMMC_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

/** 
  * @brief  Masks for R6 Response
  */
#define SDMMC_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SDMMC_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SDMMC_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SDMMC_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SDMMC_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SDMMC_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SDMMC_CHECK_PATTERN                ((uint32_t)0x000001AA)
#define SDMMC_VOLTAGE_WINDOW_MMC           ((uint32_t)0x80FF8000)

#define SDMMC_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SDMMC_ALLZERO                      ((uint32_t)0x00000000)
#define SDMMC_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)
#define SDMMC_0TO7BITS                     ((uint32_t)0x000000FF)
#define SDMMC_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SDMMC_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SDMMC_24TO31BITS                   ((uint32_t)0xFF000000)
#define SDMMC_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SDMMC_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SDMMC_CARD_LOCKED                  ((uint32_t)0x02000000)
#define SDMMC_CARD_PROGRAMMING             ((uint32_t)0x00000007)
#define SDMMC_CARD_RECEIVING               ((uint32_t)0x00000006)
#define SDMMC_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SDMMC_HALFFIFO                     ((uint32_t)0x00000008)
#define SDMMC_HALFFIFOBYTES                ((uint32_t)0x00000020)

/** 
  * @brief  Command Class Supported
  */
#define SDMMC_CCCC_LOCK_UNLOCK             ((uint32_t)0x00000080)
#define SDMMC_CCCC_WRITE_PROT              ((uint32_t)0x00000040)
#define SDMMC_CCCC_ERASE                   ((uint32_t)0x00000020)

HAL_SDMMCStatusTypeDef SDMMC_Init(SDMMC_HandleTypeDef *hsd);
uint32_t SDMMC_SelectDeselect(SDIO_TypeDef *SDIOx, uint32_t addr);

void SDMMC_GetCardInfo(SDMMC_HandleTypeDef hsd, SDMMC_CardInfoTypeDef *CardInfo);
uint32_t EmmcGetCardInfo(SDMMC_HandleTypeDef hsd, SDMMC_CardInfoTypeDef *CardInfo);

HAL_SDMMCStatusTypeDef HAL_SD_GetCardStatus(SDMMC_HandleTypeDef *hsd, HAL_SD_CardStatusTypeDef *pStatus);
HAL_SDMMCStatusTypeDef HAL_SDMMC_GetCardCSD(SDMMC_HandleTypeDef *hsd, HAL_SDMMC_CardCSDTypeDef *pCSD);
HAL_SDMMCStatusTypeDef HAL_SDMMC_GetCardCID(SDMMC_HandleTypeDef *hsd, HAL_SDMMC_CardCIDTypeDef *pCID);
uint32_t SDMMC_ConfigWideBusOperation(SDMMC_HandleTypeDef *hsd);

/* Blocking mode: Polling & Dma mode */
HAL_SDMMCStatusTypeDef HAL_SDMMC_ReadBlocks     (SDMMC_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_SDMMCStatusTypeDef HAL_SDMMC_WriteBlocks    (SDMMC_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_SDMMCStatusTypeDef HAL_SDMMC_Erase(SDMMC_HandleTypeDef *hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd);

HAL_SDMMCStatusTypeDef SDMMC_ReadDisk(SDMMC_HandleTypeDef *hsd, uint8_t*buf, uint32_t sector, uint32_t cnt);
HAL_SDMMCStatusTypeDef SDMMC_WriteDisk(SDMMC_HandleTypeDef *hsd, uint8_t*buf, uint32_t sector, uint32_t cnt);

#ifdef __cplusplus
}
#endif

#endif
/* _NS_SDMMC_H_ */
