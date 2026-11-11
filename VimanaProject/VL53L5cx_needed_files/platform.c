/*
 * platform.c - Port de la ULD del VL53L5CX para STM32 HAL + FreeRTOS
 *
 * Cada transaccion I2C toma el mutex del bus (Lidar_I2CLock), para que el
 * MPU6050 y los VL53L5CX puedan convivir en el mismo I2C desde distintas tareas.
 */
#include "platform.h"
#include "lidar.h"
#include "task.h"
 
#define VL53_I2C_TIMEOUT_MS   200U
 
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
    HAL_StatusTypeDef st;
 
    Lidar_I2CLock();
    st = HAL_I2C_Mem_Read(VL53L5CX_I2C_HANDLE, p_platform->address, RegisterAdress,
                          I2C_MEMADD_SIZE_16BIT, p_values, (uint16_t)size, VL53_I2C_TIMEOUT_MS);
    Lidar_I2CUnlock();
 
    return (st == HAL_OK) ? 0U : 1U;
}
 
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
    HAL_StatusTypeDef st;
 
    Lidar_I2CLock();
    st = HAL_I2C_Mem_Write(VL53L5CX_I2C_HANDLE, p_platform->address, RegisterAdress,
                           I2C_MEMADD_SIZE_16BIT, p_values, (uint16_t)size, VL53_I2C_TIMEOUT_MS);
    Lidar_I2CUnlock();
 
    return (st == HAL_OK) ? 0U : 1U;
}
 
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1U);
}
 
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1U);
}
 
void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i, tmp;
 
    for (i = 0; i < size; i += 4U)
    {
        tmp = ((uint32_t)buffer[i] << 24) | ((uint32_t)buffer[i + 1U] << 16) |
              ((uint32_t)buffer[i + 2U] << 8) | (uint32_t)buffer[i + 3U];
        memcpy(&buffer[i], &tmp, 4U);
    }
}
 
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{
    (void)p_platform;
 
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t ticks = pdMS_TO_TICKS(TimeMs);
        vTaskDelay((ticks > 0U) ? ticks : 1U);
    }
    else
    {
        HAL_Delay(TimeMs);
    }
    return 0U;
}
