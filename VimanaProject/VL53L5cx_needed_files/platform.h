/*
 * platform.h - Port de la ULD del VL53L5CX para STM32 HAL + FreeRTOS
 */
#ifndef PLATFORM_H_
#define PLATFORM_H_
 
#include <stdint.h>
#include <string.h>
#include "main.h"                      /* HAL + I2C_HandleTypeDef */
 
/* I2C donde estan los VL53L5CX (el mismo bus del MPU6050). Ajusta si no es I2C1. */
extern I2C_HandleTypeDef hi2c1;
#define VL53L5CX_I2C_HANDLE   (&hi2c1)
 
/* Solo necesitamos distance_mm y target_status: desactivar el resto reduce
 * mucho los bytes leidos por I2C en cada trama. */
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_MOTION_INDICATOR
 
typedef struct
{
    uint16_t address;                  /* direccion I2C de 8 bits (0x52 por defecto) */
} VL53L5CX_Platform;
 
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
void    VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);
 
#endif /* PLATFORM_H_ */
