#include "logging.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stm32l0xx_hal.h"

extern UART_HandleTypeDef huart2;

static char output[1024];

void log_uint8_buffer(const char *prefix, const uint8_t *buffer, uint8_t length) {
    uint8_t output_len = strlen(prefix) + 3 * length + 1;
    memset(output, 0, output_len);

    strcat(output, prefix);

    for (uint16_t i = 0; i < length; i++) {
        char hex[4];
        sprintf(hex, "%02X ", buffer[i]);
        strcat(output, hex);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *) output, strlen(output), HAL_MAX_DELAY);
}

void log_info(const char *fmt, ...) {
    memset(output, 0, strlen(output));
    va_list args;
    va_start(args, fmt);
    vsprintf(output, fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t *) output, 1024, HAL_MAX_DELAY);
}
