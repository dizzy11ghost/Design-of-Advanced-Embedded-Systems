#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "vl53l5cx_api.h"
typedef struct
{
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData   res;
    GPIO_TypeDef          *lpn_port;
    uint16_t               lpn_pin;
    uint16_t               int_pin;
    uint16_t               min_mm; //dist min del último cuadrado
    uint8_t                hits;
    uint8_t                clears;
    bool                   blocked;
    TickType_t             last_frame;
} lidar_sensor_t;

#define LIDAR_OBSTACLE_MM       1500U 
#define LIDAR_CLEAR_MM          1600U 
#define LIDAR_TRIGGER_FRAMES    2U  //tramas consecutivas con obstácuñps para ya levantar bandera
#define LIDAR_CLEAR_FRAMES      3U 
#define LIDAR_RANGING_HZ        15U  //frecuencia de medición
#define LIDAR_WATCHDOG_MS       500U //si no hay tramas significa fallo y lleva a bloqueo
//banderas -----
#define LIDAR_EVT_OBSTACLE      (1U << 0)
#define LIDAR_EVT_FAULT         (1U << 1) 

#define LIDAR_COUNT             2U
#define LIDAR_RESOLUTION        VL53L5CX_RESOLUTION_4X4
#define LIDAR_ZONES             16U
#define LIDAR_COLS              4U

#define LIDAR_ROW_MASK          0x0FU

#define LIDAR_ADDR_SENSOR0      0x54U
#define LIDAR_POLL_TIMEOUT_MS   100U
#define LIDAR_REINIT_MS         3000U
#define LIDAR_TASK_STACK_WORDS  1024U

#define LIDAR_TASK_PRIORITY     (configMAX_PRIORITIES / 2)

//variables 
static lidar_sensor_t s_sensor[LIDAR_COUNT] =
{
    { .lpn_port = LIDAR1_LPN_GPIO_Port, .lpn_pin = LIDAR1_LPN_Pin, .int_pin = LIDAR1_INT_Pin },
    { .lpn_port = LIDAR2_LPN_GPIO_Port, .lpn_pin = LIDAR2_LPN_Pin, .int_pin = LIDAR2_INT_Pin },
};
 
static TaskHandle_t        s_lidarTask;
static EventGroupHandle_t  s_lidarEvents;
static SemaphoreHandle_t   s_i2cMutex;
static volatile TickType_t s_lidarAlive;
 
//prototipos de funciones
bool     Lidar_Init(void);                  // crea mutex, banderas y tarea                  
bool     Lidar_ForwardBlocked(void);        // true si hay obstaculo O falla (usar en motores)
bool     Lidar_ObstacleDetected(void);      //true solo si hay obstaculo                      
uint16_t Lidar_GetMinDistanceMm(uint8_t sensor);
void     Lidar_I2CLock(void);               /* nota!!: usarlos tambien en el codigo del MPU6050        */
void     Lidar_I2CUnlock(void);
void     Lidar_ObstacleCallback(bool blocked);

static inline void lpn_write(const lidar_sensor_t *s, GPIO_PinState v) 
{
    HAL_GPIO_WritePin(s->lpn_port, s->lpn_pin, v); //para poner en low el pin LPN y que el sensor se active
} 

// es válido, 9 tmb pero con un pulso grande, para definir usaremos 6 para evitar falsos positivos
static inline bool status_valid(uint8_t st)
{
    return (st == 5U) || (st == 6U) || (st == 9U);
}

//multiplexor del bus de i2c para mpu6050 y lidars
void Lidar_I2CLock(void) 
{
    if (s_i2cMutex != NULL) { (void)xSemaphoreTake(s_i2cMutex, portMAX_DELAY); }
}

void Lidar_I2CUnlock(void) 
{
    if (s_i2cMutex != NULL) { (void)xSemaphoreGive(s_i2cMutex); }
}

//inicialización de los sensores
static bool sensors_start(void)
{
    uint8_t alive; //el alive sirve para saber si el sensor responde a la dirección i2c, no es necesario para nada más
 
    for (uint32_t i = 0; i < LIDAR_COUNT; i++) //este for se encarga de bloquear ambos sensores y luego habilitar uno por uno para cambiarle la dirección i2c
    {
        memset(&s_sensor[i].dev, 0, sizeof(s_sensor[i].dev));
        memset(&s_sensor[i].res, 0, sizeof(s_sensor[i].res));
        s_sensor[i].min_mm  = 0xFFFFU;
        s_sensor[i].hits    = 0U;
        s_sensor[i].clears  = 0U;
        s_sensor[i].blocked = true;     //bloqueado hasta ver LIDAR_CLEAR_FRAMES tramas libre
        lpn_write(&s_sensor[i], GPIO_PIN_RESET);   // I2C deshabilitado en ambos 
    }
    vTaskDelay(pdMS_TO_TICKS(10));
 
    // Sensor 0: se habilita solo y se mueve a otra direccion 
    lpn_write(&s_sensor[0], GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(10));
 
    alive = 0U;
    s_sensor[0].dev.platform.address = LIDAR_ADDR_SENSOR0;      //reintentamos en caso de que el sensor ya estaba en la dirección correcta, si no responde se pone en la default y se le cambia a la nueva
    if (!((vl53l5cx_is_alive(&s_sensor[0].dev, &alive) == 0U) && (alive != 0U)))
    {
        s_sensor[0].dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
        if (vl53l5cx_set_i2c_address(&s_sensor[0].dev, LIDAR_ADDR_SENSOR0) != 0U)
        {
            return false;
        }
    }
 
    // Sensor 1: ahora si se habilita; responde en 0x52
    lpn_write(&s_sensor[1], GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(10));
    s_sensor[1].dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
 
    // Init + configuracion de cada uno
    for (uint32_t i = 0; i < LIDAR_COUNT; i++)
    {
        VL53L5CX_Configuration *d = &s_sensor[i].dev;
 
        alive = 0U;
        if ((vl53l5cx_is_alive(d, &alive) != 0U) || (alive == 0U))                return false;
        if (vl53l5cx_init(d) != 0U)                                               return false; /* sube firmware, ~1-2 s */
        if (vl53l5cx_set_resolution(d, LIDAR_RESOLUTION) != 0U)                   return false;
        if (vl53l5cx_set_ranging_frequency_hz(d, LIDAR_RANGING_HZ) != 0U)         return false;
        if (vl53l5cx_set_ranging_mode(d, VL53L5CX_RANGING_MODE_CONTINUOUS) != 0U) return false;
        if (vl53l5cx_start_ranging(d) != 0U)                                      return false;
 
        s_sensor[i].last_frame = xTaskGetTickCount(); //ese xtaskgettickcount es para el watchdog, si no hay tramas en LIDAR_WATCHDOG_MS se considera fallo
    }
    return true;
}

//proceso de una trama utilizando umbral + histeresis (friendly reminder: el sensor devuelve 16 zonas de 4x4, cada zona es un cuadrado de 4x4 pixeles)
static void process_frame(lidar_sensor_t *s)
{
  uint16_t min_mm = 0xFFFFU; //distancia mínima de la trama, se inicializa a un valor grande para que cualquier distancia válida lo reemplace
 
    for (uint32_t z = 0; z < LIDAR_ZONES; z++)
    {
        if ((LIDAR_ROW_MASK & (1UL << (z / LIDAR_COLS))) == 0U) //si la zona no está habilitada, se salta
        {
            continue;
        }
 
        uint32_t idx = z * VL53L5CX_NB_TARGET_PER_ZONE; //cada zona tiene 4 targets, se toma el primero (el más cercano)
        int16_t  d   = s->res.distance_mm[idx]; //distancia en mm del target más cercano de la zona z
 
        if (status_valid(s->res.target_status[idx]) && (d > 0) && ((uint16_t)d < min_mm)) //si el target es válido y la distancia es menor que la mínima encontrada hasta ahora, se actualiza min_mm
        {
            min_mm = (uint16_t)d; //histeresis!!!!
        }
    }
    s->min_mm = min_mm;
 
    if (min_mm <= LIDAR_OBSTACLE_MM)
    { 
        s->clears = 0U; //s es un puntero a la estructura del sensor, se resetea el contador de clears porque se detectó un obstáculo
        if (s->hits < 255U) { s->hits++; } //se incrementa el contador de hits, si es menor que 255 (para evitar overflow)
        if (s->hits >= LIDAR_TRIGGER_FRAMES) { s->blocked = true; } //si se detectaron suficientes tramas con obstáculo, se marca el sensor como bloqueado
    }
    else if (min_mm > LIDAR_CLEAR_MM)
    {
        s->hits = 0U;
        if (s->clears < 255U) { s->clears++; }
        if (s->clears >= LIDAR_CLEAR_FRAMES) { s->blocked = false; }
    }
    else
    {
        /* Banda de histeresis (1.5 - 1.6 m): se mantiene el estado actual */
        s->hits   = 0U;
        s->clears = 0U;
    }
}

//ahora si los meros moles de nuestros bellos sensores, su tarea ------------------

static void Lidar_Task(void *arg)
{
    (void)arg;
 
    bool       prev_stop    = true; 
    bool       fault_active = false;
    TickType_t fault_since  = 0U;
 
    while (!sensors_start())
    {
        vTaskDelay(pdMS_TO_TICKS(LIDAR_REINIT_MS));   //la bandera de LIDAR_EVT_FAULT sigue arriba
    }
    s_lidarAlive = xTaskGetTickCount();
    xEventGroupClearBits(s_lidarEvents, LIDAR_EVT_FAULT);
 
    for (;;)
    {
        /* Duerme hasta que llegue INT de algun sensor (o 100 ms como respaldo) */
        (void)xTaskNotifyWait(0U, UINT32_MAX, NULL, pdMS_TO_TICKS(LIDAR_POLL_TIMEOUT_MS));
 
        /* Se consultan ambos siempre: la INT solo sirve para despertar rapido */
        for (uint32_t i = 0; i < LIDAR_COUNT; i++)
        {
            lidar_sensor_t *s = &s_sensor[i];
            uint8_t ready = 0U;
 
            if ((vl53l5cx_check_data_ready(&s->dev, &ready) == 0U) && (ready != 0U) &&
                (vl53l5cx_get_ranging_data(&s->dev, &s->res) == 0U))
            {
                process_frame(s);
                s->last_frame = xTaskGetTickCount(); //ese taskgettickcount es para el watchdog, si no hay tramas en LIDAR_WATCHDOG_MS se considera fallo
            }
        }
 
        //bloqueado si CUALQUIERA ve algo; libre solo si AMBOS libres 
        TickType_t now     = xTaskGetTickCount();
        bool       blocked = false;
        bool       fault   = false;
 
        for (uint32_t i = 0; i < LIDAR_COUNT; i++)
        {
            if (s_sensor[i].blocked) { blocked = true; }
            if ((TickType_t)(now - s_sensor[i].last_frame) > pdMS_TO_TICKS(LIDAR_WATCHDOG_MS)) //pdMS_to_ticks convierte milisegundos a ticks de FreeRTOS
            {
                fault = true;
            }
        }
        s_lidarAlive = now;
 
        //Banderas!! básicamente usamos las banderas de obstáculo y falla para que la función Lidar_ForwardBlocked() pueda consultarlas y decidir si se puede avanzar o no
        if (blocked) { xEventGroupSetBits(s_lidarEvents, LIDAR_EVT_OBSTACLE); }
        else         { xEventGroupClearBits(s_lidarEvents, LIDAR_EVT_OBSTACLE); }
 
        if (fault)   { xEventGroupSetBits(s_lidarEvents, LIDAR_EVT_FAULT); }
        else         { xEventGroupClearBits(s_lidarEvents, LIDAR_EVT_FAULT); }
 
        // Evento de cambio de estado (flanco)
        bool stop = blocked || fault;
        if (stop != prev_stop)
        {
            prev_stop = stop;
            Lidar_ObstacleCallback(stop);
        }
 
        // Si un sensor deja de responder, reintenta arrancarlos cada LIDAR_REINIT_MS
        if (fault)
        {
            if (!fault_active)
            {
                fault_active = true;
                fault_since  = now;
            }
            else if ((TickType_t)(now - fault_since) > pdMS_TO_TICKS(LIDAR_REINIT_MS))
            {
                (void)sensors_start();
                fault_since = xTaskGetTickCount();
            }
        }
        else
        {
            fault_active = false;
        }
    }
}
 
//API de los VL53L5cx, inicializa mutex, banderas y tarea
bool Lidar_Init(void)
{
    s_i2cMutex    = xSemaphoreCreateMutex();
    s_lidarEvents = xEventGroupCreate();
    if ((s_i2cMutex == NULL) || (s_lidarEvents == NULL))
    {
        return false;
    }
 
    xEventGroupSetBits(s_lidarEvents, LIDAR_EVT_FAULT);   /* fail-safe: bloqueado hasta que arranquen */
    s_lidarAlive = xTaskGetTickCount();
 
    return (xTaskCreate(Lidar_Task, "lidar", LIDAR_TASK_STACK_WORDS, NULL,
                        LIDAR_TASK_PRIORITY, &s_lidarTask) == pdPASS);
}
 
bool Lidar_ForwardBlocked(void) //esta función devuelve true si hay obstáculo o falla en cualquiera de los sensores, y false si ambos sensores están libres y funcionando correctamente
{
    if (s_lidarEvents == NULL)
    {
        return true;
    }
    /* Si la tarea del lidar se colgo o no corre, tambien se bloquea */
    if ((TickType_t)(xTaskGetTickCount() - s_lidarAlive) > pdMS_TO_TICKS(LIDAR_WATCHDOG_MS))
    {
        return true;
    }
    return (xEventGroupGetBits(s_lidarEvents) & (LIDAR_EVT_OBSTACLE | LIDAR_EVT_FAULT)) != 0U;
}
 
bool Lidar_ObstacleDetected(void) //esta sirve para saber si hay obstáculo, sin considerar la falla de los sensores, es decir, devuelve true si hay obstáculo detectado por cualquiera de los sensores, y false si ambos sensores están libres
{
    return (s_lidarEvents != NULL) && ((xEventGroupGetBits(s_lidarEvents) & LIDAR_EVT_OBSTACLE) != 0U);
}
 
uint16_t Lidar_GetMinDistanceMm(uint8_t sensor) //esta función devuelve la distancia mínima detectada por el sensor especificado (0 o 1), o 0xFFFF si el índice del sensor es inválido
{
    return (sensor < LIDAR_COUNT) ? s_sensor[sensor].min_mm : 0xFFFFU;
}
 
/* Corre en contexto de interrupcion (EXTI): solo despierta a la tarea. */
void Lidar_EXTI_Handler(uint16_t GPIO_Pin)
{
    BaseType_t woken = pdFALSE;
 
    if (s_lidarTask == NULL)
    {
        return;
    }
 
    if (GPIO_Pin == s_sensor[0].int_pin)
    {
        xTaskNotifyFromISR(s_lidarTask, (1UL << 0), eSetBits, &woken);
    }
    else if (GPIO_Pin == s_sensor[1].int_pin)
    {
        xTaskNotifyFromISR(s_lidarTask, (1UL << 1), eSetBits, &woken);
    }
    portYIELD_FROM_ISR(woken);
}
 
/* Se llama (desde Lidar_Task, NO desde ISR) cada vez que cambia el estado:
 *   blocked = true  -> aparecio obstaculo o falla de sensor -> FRENAR
 *   blocked = false -> ambos sensores libres */
void Lidar_ObstacleCallback(bool blocked) //a estas les falta la lógica de los motores para frenar!! eso va aqui merito
{
    if (blocked /* && Chair_IsMovingForward() */)
    {
        /* Chair_Stop();   <- PWM = 0 en RPWM/LPWM de los dos BTS7960 */
    }
}
