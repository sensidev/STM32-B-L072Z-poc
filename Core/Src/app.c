#include "app.h"
#include "main.h"
#include "ade9153a_regs.h"
#include "ade9153a.h"
#include "logging.h"
#include "stm32l0xx_hal.h"

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;

static EnergyRegs_t energyVals;  //Energy register values are read and stored in EnergyRegs structure
static PowerRegs_t powerVals;    //Metrology data can be accessed from these structures
static RMSRegs_t rmsVals;
static PQRegs_t pqVals;
static AcalRegs_t acalVals;
static Temperature_t tempVal;

static uint8_t raw_data[10] = {};

static void af_reset();

void app_setup() {
    af_reset();

    ade9153a_spi_write_16(REG_AI_PGAGAIN, ADE9153A_AI_PGAGAIN);
    ade9153a_spi_write_32(REG_CONFIG0, ADE9153A_CONFIG0);
    ade9153a_spi_write_16(REG_CONFIG1, ADE9153A_CONFIG1);
    ade9153a_spi_write_16(REG_CONFIG2, ADE9153A_CONFIG2);
    ade9153a_spi_write_16(REG_CONFIG3, ADE9153A_CONFIG3);
    ade9153a_spi_write_16(REG_ACCMODE, ADE9153A_ACCMODE);
    ade9153a_spi_write_32(REG_VLEVEL, ADE9153A_VLEVEL);
    ade9153a_spi_write_16(REG_ZX_CFG, ADE9153A_ZX_CFG);
    ade9153a_spi_write_32(REG_MASK, ADE9153A_MASK);
    ade9153a_spi_write_32(REG_ACT_NL_LVL, ADE9153A_ACT_NL_LVL);
    ade9153a_spi_write_32(REG_REACT_NL_LVL, ADE9153A_REACT_NL_LVL);
    ade9153a_spi_write_32(REG_APP_NL_LVL, ADE9153A_APP_NL_LVL);
    ade9153a_spi_write_16(REG_COMPMODE, ADE9153A_COMPMODE);
    ade9153a_spi_write_32(REG_VDIV_RSMALL, ADE9153A_VDIV_RSMALL);
    ade9153a_spi_write_16(REG_EP_CFG, ADE9153A_EP_CFG);
    ade9153a_spi_write_16(REG_EGY_TIME, ADE9153A_EGY_TIME); //Energy accumulation ON
    ade9153a_spi_write_16(REG_TEMP_CFG, ADE9153A_TEMP_CFG);


    ade9153a_spi_write_16(REG_RUN, ADE9153A_RUN_ON);
    HAL_Delay(100);
    uint32_t version = ade9153a_spi_read_32(REG_VERSION_PRODUCT);
    if (version != 0x0009153A) {
        log_info("Wrong version! ");
    }

    log_info("AF version: %X \r\n", version);

    ade9153a_spi_write_32(REG_AIGAIN, -268435456); //AIGAIN to -1 to account for IAP-IAN swap

    HAL_Delay(500);
}

void app_loop_raw() {
    bool is_comm_ok;

    // Read Instantaneous current register
    is_comm_ok = ade9153a_spi_read_bytes(REG_AIRMS, 4, raw_data + 0);
    raw_data[5] = is_comm_ok;

    // Read Instantaneous voltage register
    is_comm_ok = ade9153a_spi_read_bytes(REG_AVRMS, 4, raw_data + 5);
    raw_data[9] = is_comm_ok;

    HAL_UART_Transmit(&huart2, raw_data, 10, 100);
}

void app_loop() {

    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

    ade9153a_read_power_regs(&powerVals);
    ade9153a_read_rms_regs(&rmsVals);
    ade9153a_read_pq_regs(&pqVals);
    ade9153a_read_temperature(&tempVal);

    log_info("RMS Voltage: %.2f V \r\n", rmsVals.VoltageRMSValue / 1000);
    log_info("RMS Current: %.2f A \r\n", rmsVals.CurrentRMSValue / 1000);
    log_info("Active Power: %.2f W \r\n", powerVals.ActivePowerValue / 1000);
    log_info("Fund Reactive Power: %.2f VAR \r\n", powerVals.FundReactivePowerValue / 1000);
    log_info("Apparent Power: %.2f VA \r\n", powerVals.ApparentPowerValue / 1000);
    log_info("Power Factor: %.2f \r\n", pqVals.PowerFactorValue);
    log_info("Frequency: %.2f Hz \r\n", pqVals.FrequencyValue);
    log_info("Temperature: %.2f °C \r\n", tempVal.TemperatureVal);
    log_info("------------------------------------------------------------\r\n");

    HAL_Delay(1000);
}

static void af_reset() {
    HAL_GPIO_WritePin(AF_RESET_GPIO_Port, AF_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(AF_RESET_GPIO_Port, AF_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    log_info("Reset done");
}
