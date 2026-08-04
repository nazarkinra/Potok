#include "sd_mcp23017.h"
#include "fatfs.h" // Подключаем переменные CubeMX: USERFatFS и USERPath
#include <string.h>

static SPI_HandleTypeDef *sd_hspi;

#include "sd_mcp23017.h"
#include "fatfs.h"
#include <string.h>

static SPI_HandleTypeDef *sd_hspi;

#include "sd_mcp23017.h"
#include "fatfs.h"
#include <string.h>

static SPI_HandleTypeDef *sd_hspi;

#include "sd_mcp23017.h"
#include "fatfs.h"
#include <string.h>

static SPI_HandleTypeDef *sd_hspi;

uint8_t SD_MCP_Init(SPI_HandleTypeDef *hspi) {
    FRESULT res;
    sd_hspi = hspi;

    Serial_PrintString(DEBUG_UART, "=== SD Card Init Start ===\r\n");

    if (MCP23017_Init() != HAL_OK) return 1;

    SD_CS_HIGH();
    HAL_Delay(10);

    Serial_Printf(DEBUG_UART, "[SD] Монтирование диска %s ...\r\n", USERPath);
    res = f_mount(&USERFatFS, USERPath, 1);

    // Если файловая система сломана из-за прошлых багов - форматируем заново
    if (res == FR_NO_FILESYSTEM) {
        Serial_PrintString(DEBUG_UART, "[SD] Форматирование в FAT32...\r\n");

        static BYTE work_buf[512];
        res = f_mkfs(USERPath, 0x02, 0, work_buf, sizeof(work_buf));

        if (res == FR_OK) {
            Serial_PrintString(DEBUG_UART, "[SD] Форматирование успешно!\r\n");

            // Очищаем кэш и переподключаем
            f_mount(NULL, USERPath, 0);
            memset(&USERFatFS, 0, sizeof(FATFS));
            HAL_Delay(100);

            res = f_mount(&USERFatFS, USERPath, 1);
        } else {
            Serial_Printf(DEBUG_UART, "Error: f_mkfs failed with code %d\r\n", res);
            return 2;
        }
    }

    if (res != FR_OK) {
        Serial_Printf(DEBUG_UART, "Error: f_mount failed with code %d\r\n", res);
        return 3;
    }

    Serial_PrintString(DEBUG_UART, "SD Card Mounted Successfully!\r\n");
    return 0;
}

uint8_t SD_MCP_WriteFile(const char *filename, const char *data) {
    FIL file;
    FRESULT res;
    UINT bytesWritten;

    res = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) return 1;

    res = f_write(&file, data, strlen(data), &bytesWritten);
    if (res != FR_OK || bytesWritten == 0) {
        f_close(&file);
        return 2;
    }

    res = f_close(&file);
    if (res != FR_OK) return 3;

    return 0;
}
