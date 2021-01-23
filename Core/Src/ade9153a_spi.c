#include "ade9153a_spi.h"
#include "main.h"
#include "stm32l0xx_hal.h"

#define SPI_TIMEOUT 1000

extern SPI_HandleTypeDef hspi2;

ADE9153AStatus_t ade9153a_spi_write(uint8_t *data, uint8_t length) {
    ADE9153AStatus_t ret = ADE9153A_OK;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(AF_SPI_NSS_GPIO_Port, AF_SPI_NSS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, data, length, SPI_TIMEOUT);
    HAL_GPIO_WritePin(AF_SPI_NSS_GPIO_Port, AF_SPI_NSS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        ret = ADE9153A_SPI_WRITE_FAIL_ERROR;
    }

    return ret;
}

ADE9153AStatus_t ade9153a_spi_read(uint16_t cmd, uint8_t *data, uint8_t length) {
    ADE9153AStatus_t ret = ADE9153A_OK;
    HAL_StatusTypeDef status;

    uint8_t cmd_buffer[2] = {
            cmd >> 8u,
            cmd
    };

    HAL_GPIO_WritePin(AF_SPI_NSS_GPIO_Port, AF_SPI_NSS_Pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi2, cmd_buffer, 2, SPI_TIMEOUT);

    if (HAL_OK == status) {
        status = HAL_SPI_Receive(&hspi2, data, length, SPI_TIMEOUT);
    } else {
        ret = ADE9153A_SPI_WRITE_FAIL_ERROR;
    }

    HAL_GPIO_WritePin(AF_SPI_NSS_GPIO_Port, AF_SPI_NSS_Pin, GPIO_PIN_SET);

    if (HAL_OK != status) {
        ret = ADE9153A_SPI_READ_FAIL_ERROR;
    }

    return ret;
}

void ade9153a_spi_delay_ms(uint32_t delay) {
    HAL_Delay(delay);
}