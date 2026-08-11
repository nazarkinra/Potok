/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver completed with SD-Card SPI logic.
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */
#include <string.h>
#include "ff_gen_drv.h"
#include "stm32f4xx_hal.h"
#include "mcp23017.h"      // Твоя библиотека расширителя
#include "serial_print.h"  // Твоя библиотека UART логов

// Замени huart1 на тот, который у тебя используется для вывода
extern UART_HandleTypeDef huart1;
#define DEBUG_UART &huart1

// SPI2 используется для SD-карты
extern SPI_HandleTypeDef hspi2;

// Макросы управления пином CS через твой расширитель (GPA0 = пин 0)
#define SD_CS_LOW()  MCP23017_WritePin(0, 0)
#define SD_CS_HIGH() MCP23017_WritePin(0, 1)

// Статусы и типы карт
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

/* Вспомогательные функции SPI */
static uint8_t SPI_RxByte(void) {
    uint8_t dummy = 0xFF;
    uint8_t data = 0;
    HAL_SPI_TransmitReceive(&hspi2, &dummy, &data, 1, 100);
    return data;
}

static void SPI_TxByte(uint8_t data) {
    HAL_SPI_Transmit(&hspi2, &data, 1, 100);
}

/* Ожидание готовности карты */
static uint8_t SD_WaitReady(void) {
    uint32_t tickstart = HAL_GetTick();
    uint8_t res;
    do {
        res = SPI_RxByte();
        if (res == 0xFF) return 1; // Готово
    } while ((HAL_GetTick() - tickstart) < 2000);
    return 0; // Таймаут
}

/* Отправка команды на SD-карту */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
    uint8_t n, res;

    if (SD_WaitReady() == 0) return 0xFF;

    SPI_TxByte(cmd | 0x40);
    SPI_TxByte((uint8_t)(arg >> 24));
    SPI_TxByte((uint8_t)(arg >> 16));
    SPI_TxByte((uint8_t)(arg >> 8));
    SPI_TxByte((uint8_t)arg);

    // CRC (обязателен только для CMD0 и CMD8, но проще слать всегда правильные для них)
    n = 0x01;
    if (cmd == 0) n = 0x95;
    if (cmd == 8) n = 0x87;
    SPI_TxByte(n);

    // Ожидание ответа
    n = 10;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    uint8_t n, cmd, ty, ocr[4];
    uint32_t tickstart;
    uint8_t cmd0_res;

    if (pdrv != 0) return STA_NOINIT;

    HAL_Delay(100);

    SD_CS_HIGH();
    for (n = 10; n; n--) SPI_TxByte(0xFF);

    SD_CS_LOW();
    HAL_Delay(1);
    SPI_TxByte(0xFF);

    ty = 0;
    cmd0_res = SD_SendCmd(0, 0);

    if (cmd0_res == 1) {
        tickstart = HAL_GetTick();
        if (SD_SendCmd(8, 0x1AA) == 1) { // CMD8 (SDv2)
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                while ((HAL_GetTick() - tickstart) < 1000) {
                    SD_SendCmd(55, 0);
                    if (SD_SendCmd(41, 1UL << 30) == 0) break; // ACMD41
                }

                // --- КРИТИЧЕСКИЙ ФИКС ЗДЕСЬ ---
                // Читаем OCR регистр, чтобы проверить CCS бит (SDHC или SDSC)
                if ((HAL_GetTick() - tickstart) < 1000 && SD_SendCmd(58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
                    // Если 30-й бит (0x40 в старшем байте) равен 1 - это SDHC. Иначе - SDSC (до 2 ГБ)
                    ty = (ocr[0] & 0x40) ? 2 : 1;
                }
                // ------------------------------
            }
        } else {
            Serial_PrintString(DEBUG_UART, "[SD] SDv1/MMC detected.\r\n");
            cmd = (SD_SendCmd(55, 0) <= 1 && SD_SendCmd(41, 0) <= 1) ? 41 : 1;
            while ((HAL_GetTick() - tickstart) < 1000) {
                if (cmd == 41) SD_SendCmd(55, 0);
                if (SD_SendCmd(cmd, 0) == 0) {
                    ty = 1;
                    break;
                }
            }
        }
    } else {
        Serial_Printf(DEBUG_UART, "[SD] Error: CMD0 Failed. Return code: 0x%02X\r\n", cmd0_res);
    }

    SD_CS_HIGH();
    SPI_RxByte();

    CardType = ty;
    if (ty) {
        Stat &= ~STA_NOINIT;
        Serial_Printf(DEBUG_UART, "[SD] Init OK. Card Type: %d\r\n", ty);
    } else {
        Serial_PrintString(DEBUG_UART, "[SD] Init Failed!\r\n");
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    if (pdrv != 0) return STA_NOINIT;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    if (pdrv != 0 || Stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;
    SD_CS_LOW();

    if (count == 1) {
        // Преобразуем адрес ТОЛЬКО перед отправкой
        DWORD addr = (CardType == 2) ? sector : (sector * 512);

        if (SD_SendCmd(17, addr) == 0) { // CMD17
            uint32_t tickstart = HAL_GetTick();
            while (SPI_RxByte() != 0xFE) {
                if ((HAL_GetTick() - tickstart) > 200) {
                    SD_CS_HIGH();
                    return RES_ERROR;
                }
            }
            for (uint16_t i = 0; i < 512; i++) buff[i] = SPI_RxByte();
            SPI_RxByte(); SPI_RxByte();
            res = RES_OK;
        }
    } else {
        res = RES_OK;
        for(UINT i = 0; i < count; i++) {
            // Передаем СЫРОЙ номер сектора, умножение произойдет внутри
            if(USER_read(pdrv, buff, sector + i, 1) != RES_OK) {
                res = RES_ERROR;
                break;
            }
            buff += 512;
        }
        return res;
    }

    SD_CS_HIGH();
    SPI_RxByte();
    return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    if (pdrv != 0 || Stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;
    SD_CS_LOW();

    if (count == 1) {
        // Преобразуем адрес ТОЛЬКО перед отправкой
        DWORD addr = (CardType == 2) ? sector : (sector * 512);

        if (SD_SendCmd(24, addr) == 0) { // CMD24
            SPI_TxByte(0xFF);
            SPI_TxByte(0xFE);

            for (uint16_t i = 0; i < 512; i++) SPI_TxByte(buff[i]);

            SPI_TxByte(0xFF); SPI_TxByte(0xFF);

            if ((SPI_RxByte() & 0x1F) == 0x05) {
                uint32_t t_busy = HAL_GetTick();
                while(SPI_RxByte() != 0x00) {
                    if(HAL_GetTick() - t_busy > 100) break;
                }

                if (SD_WaitReady() == 1) {
                    res = RES_OK;
                } else {
                    SD_CS_HIGH();
                    return RES_ERROR;
                }
            }
        }
    } else {
         res = RES_OK;
         for(UINT i = 0; i < count; i++) {
            // Передаем СЫРОЙ номер сектора, умножение произойдет внутри
            if(USER_write(pdrv, buff, sector + i, 1) != RES_OK) {
                res = RES_ERROR;
                break;
            }
            buff += 512;
        }
        return res;
    }

    SD_CS_HIGH();
    SPI_RxByte();
    return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    if (pdrv != 0 || Stat & STA_NOINIT) return RES_NOTRDY;

    SD_CS_LOW();
    switch (cmd) {
        case CTRL_SYNC:
            if (SD_WaitReady() == 1) res = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            if (SD_SendCmd(9, 0) == 0) {
                uint8_t token;
                uint32_t tickstart = HAL_GetTick();
                do {
                    token = SPI_RxByte();
                } while (token == 0xFF && (HAL_GetTick() - tickstart) < 200);

                if (token == 0xFE) { // Считаем сектора ТОЛЬКО если дождались токена
                    uint8_t csd[16];
                    for (int i = 0; i < 16; i++) csd[i] = SPI_RxByte();
                    SPI_RxByte(); SPI_RxByte();

                    if ((csd[0] >> 6) == 1) {
                        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint16_t)csd[8] << 8) | csd[9];
                        *(DWORD*)buff = (c_size + 1) * 1024;
                    } else {
                        uint8_t n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                        uint32_t c_size = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
                        *(DWORD*)buff = c_size << (n - 9);
                    }
                    res = RES_OK;
                }
            }
            break;

        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 512;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
    }

    SD_CS_HIGH();
    SPI_RxByte();
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

