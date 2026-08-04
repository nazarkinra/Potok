#include "sd_mcp23017.h"
#include "fatfs.h" // Подключаем переменные CubeMX: USERFatFS и USERPath
#include <string.h>
#include <stdbool.h>

static SPI_HandleTypeDef *sd_hspi;
FIL log_file;           // Глобальный объект файла
bool is_file_open = false;

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

// Новая функция для одноразового открытия файла при старте
uint8_t SD_MCP_OpenFile(const char *filename) {
    // Открываем файл (создаем, если его не существует)
    FRESULT res = f_open(&log_file, filename, FA_OPEN_ALWAYS | FA_WRITE);

    if (res == FR_OK) {
        // Перемещаем курсор в самый конец файла для дозаписи (эквивалент APPEND)
        res = f_lseek(&log_file, f_size(&log_file));

        if (res == FR_OK) {
            is_file_open = true;
            return 0;
        } else {
            Serial_Printf(DEBUG_UART, "[SD] Ошибка f_lseek: %d\r\n", res);
            f_close(&log_file);
            return 2;
        }
    }

    // Если открыть не удалось, выводим код ошибки в консоль
    Serial_Printf(DEBUG_UART, "[SD] Ошибка f_open. Код FatFs: %d\r\n", res);
    return 1;
}

// Обновленная функция записи (работает в разы быстрее)
uint8_t SD_MCP_WriteFile(const char *filename, const char *data) {
    if (!is_file_open) return 1; // Защита, если файл не открылся

    UINT bytesWritten;
    if (f_write(&log_file, data, strlen(data), &bytesWritten) != FR_OK) {
        return 2;
    }

    // Принудительно сбрасываем данные на карту без закрытия файла.
    // С точки зрения безопасности данных, эффект такой же, как у f_close.
    if (f_sync(&log_file) != FR_OK) {
        return 3;
    }

    return 0;
}
