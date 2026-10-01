#include <MKL25Z4.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define DS3231_ADDR             0x68u
#define REG_SECONDS             0x00u
#define REG_CONTROL             0x0Eu
#define REG_STATUS              0x0Fu
#define CTRL_EOSC               0x80u
#define CTRL_INTCN              0x04u
#define CTRL_A2IE               0x02u
#define CTRL_A1IE               0x01u
#define STATUS_OSF              0x80u
#define STATUS_A2F              0x02u
#define STATUS_A1F              0x01u
#define I2C_ICR                 0x3Fu
#define I2C_TIMEOUT_MS          250u
#define FORCE_SET_TIME          0
#define INIT_YEAR               26u
#define INIT_MONTH              9u
#define INIT_DAY                30u
#define INIT_HOUR               15u
#define INIT_MIN                30u
#define INIT_SEC                0u

#define RS (1u << 2)   // PTA2
#define RW (1u << 4)   // PTA4
#define EN (1u << 5)   // PTA5

#define RTC_INT_PIN             12u
#define RTC_INT_MASK            (1u << RTC_INT_PIN)
#define BUZZER_MASK             (1u << 1)
#define LED_RED_MASK            (1u << 18)

#define REG_ALARM1              0x07u


#define SCD30_ADDR              0x61u
#define SCD30_CMD_START         0x0010u
#define SCD30_CMD_STOP          0x0104u
#define SCD30_CMD_READY         0x0202u
#define SCD30_CMD_READ          0x0300u
#define SCD30_CMD_INTERVAL      0x4600u
#define SCD30_INTERVAL_SECONDS  2u

#define SCD30_OK                0
#define SCD30_NOT_READY         1
#define SCD30_ERROR_I2C         2
#define SCD30_ERROR_CRC         3
#define SCD30_ERROR_VALUE       4

// Definiciones del MAX7219
#define DECODE    9
#define INTENSITY 10
#define SCANLIMIT 11
#define SHUTDOWN  12
#define TEST      15

#define TEMP_READ_INTERVAL_MS  2000

// Mapa de caracteres para el teclado matricial 4x4
const char keypad_map[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

// Modos de operación del sistema
typedef enum {
    MODE_NORMAL,
    MODE_SET_TIME,
    MODE_SET_DATE,
    MODE_SET_ALARM
} system_mode_t;

// Variables globales para el flujo de configuración
system_mode_t mode = MODE_NORMAL;
char digits[6];                // Dógitos tecleados por el usuario
uint8_t ndigits = 0;           // Cantidad de dígitos ingresados
uint8_t cfg_hour, cfg_min;
uint8_t last_sec_shown = 0xFF;

typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t dow;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} rtc_time_t;

static volatile uint32_t milliseconds = 0;

static int16_t temperature_tenths = 0;
static uint8_t temperature_valid = 0;

static int timebase_init(void);
static void delayMs(uint32_t n);
static void delayUs(uint32_t n);

/* Prototipos para la pantalla LCD */
void LCD_init(void);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_string(const char cadena[]);
void LCD_goto(uint8_t row, uint8_t col);
void lcd_line(uint8_t row, const char *str);

void I2C0_init(void);
static int i2c_wait(void);
static int i2c_start(void);
static void i2c_stop(void);
static int i2c_write(uint8_t data);

static volatile uint8_t alarm_irq_pending = 0;
static volatile uint8_t alarm_armed = 0;
static uint8_t alarm_ringing = 0;
static uint8_t alarm_hour = 0;
static uint8_t alarm_minute = 0;

static uint8_t sensor_initialized = 0;

static uint8_t sensor_error = 0;

static uint32_t last_temp_read_ms = 0;

void SPI0_init(void);
void max7219_write(unsigned char command, unsigned char data);
void max7219_init(void);
void max7219_show_time(const rtc_time_t *t);

void keypad_init(void);
char keypad_getkey(void);
void process_keypad(void);
void handle_key(char key);
void config_enter(void);
void config_exit(void);
void config_draw(void);
void config_confirm(void);
void config_show_invalid(void);
void lcd_line(uint8_t row, const char *str);
void display_temperature(void);
void alarm_clear_flag(void);
void alarm_trigger(void);
void scd30_task(void);
void update_display_normal(const rtc_time_t *time);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_string(const char cadena[]);
static int i2c_write_raw(uint8_t address, const uint8_t *data, uint8_t length);
static int i2c_read_raw(uint8_t address, uint8_t *data, uint8_t length);

uint8_t days_in_month(uint8_t m, uint8_t y);

int i2c_write_bytes(uint8_t dev, uint8_t reg,
                    const uint8_t *buf, uint8_t n);

int i2c_read_bytes(uint8_t dev, uint8_t reg,
                   uint8_t *buf, uint8_t n);

static int i2c_probe(uint8_t address);

uint8_t bcd2bin(uint8_t b);
uint8_t bin2bcd(uint8_t b);
uint8_t day_of_week(uint8_t y, uint8_t m, uint8_t d);

int rtc_set_time(const rtc_time_t *t);
int rtc_get_time(rtc_time_t *t);
static int rtc_prepare(void);

static void alarm_outputs_init(void);
static void alarm_outputs_off(void);
static void rtc_interrupt_init(void);
static int rtc_alarm_set(uint8_t hour, uint8_t minute);
static void alarm_stop(void);

static uint8_t scd30_crc8(const uint8_t *data, uint8_t length);
static int scd30_command(uint16_t command, uint8_t has_argument, uint16_t argument);
static int scd30_init(void);
static int scd30_read_temperature(int16_t *tenths);
 void display_temperature(void);


void SysTick_Handler(void)
{
    milliseconds++;
}

static int timebase_init(void)
{
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) {
        return 1;
    }

    NVIC_SetPriority(SysTick_IRQn, 3u);
    return 0;
}

static void delayMs(uint32_t n)
{
    uint32_t start = milliseconds;

    while ((uint32_t)(milliseconds - start) < n) {
    }
}

static void delayUs(uint32_t n)
{
    uint32_t previous;
    uint32_t current;
    uint32_t elapsed;
    uint32_t remaining;
    uint32_t period = SysTick->LOAD + 1u;

    remaining = (uint32_t)(
        ((uint64_t)SystemCoreClock * n + 999999u) / 1000000u
    );

    previous = SysTick->VAL;

    while (remaining != 0u) {
        current = SysTick->VAL;

        if (previous >= current) {
            elapsed = previous - current;
        } else {
            elapsed = previous + period - current;
        }

        if (elapsed >= remaining) {
            break;
        }

        remaining -= elapsed;
        previous = current;
    }
}

void update_display_normal(const rtc_time_t *time)
{
    char line_buf[17];
    snprintf(line_buf, sizeof(line_buf), "%02d/%02d/%02d %s",
             time->day, time->month, time->year,
             alarm_armed ? "AL:ON " : "AL:OFF");
    LCD_goto(0, 0);
    LCD_string(line_buf);
    display_temperature();
}

void scd30_task(void)
{
    // Verificar si ha transcurrido el intervalo de 2 segundos
    if ((milliseconds - last_temp_read_ms) >= TEMP_READ_INTERVAL_MS) {
        last_temp_read_ms = milliseconds;

        // Intentar leer la temperatura desde el sensor SCD30
        if (scd30_read_temperature(&temperature_tenths) == SCD30_OK) {
            temperature_valid = 1;  // Lógica exitosa
        } else {
            temperature_valid = 0;  // Error de comunicación o checksum en I2C
        }
    }
}

void I2C0_init(void)
{
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PTC->PDDR &= ~((1u << 8) | (1u << 9));

    PORTC->PCR[8] = PORT_PCR_MUX(2) |
                    PORT_PCR_PE_MASK |
                    PORT_PCR_PS_MASK;

    PORTC->PCR[9] = PORT_PCR_MUX(2) |
                    PORT_PCR_PE_MASK |
                    PORT_PCR_PS_MASK;

    I2C0->C1 = 0;
    I2C0->F = I2C_ICR;
    I2C0->S = I2C_S_IICIF_MASK | I2C_S_ARBL_MASK;
    I2C0->C1 = I2C_C1_IICEN_MASK;
}

static int i2c_wait(void)
{
    uint32_t start = milliseconds;

    while (!(I2C0->S & I2C_S_IICIF_MASK)) {
        if (I2C0->S & I2C_S_ARBL_MASK) {
            I2C0->S = I2C_S_ARBL_MASK | I2C_S_IICIF_MASK;
            return 1;
        }

        if ((uint32_t)(milliseconds - start) >= I2C_TIMEOUT_MS) {
            return 1;
        }
    }

    if (I2C0->S & I2C_S_ARBL_MASK) {
        I2C0->S = I2C_S_ARBL_MASK | I2C_S_IICIF_MASK;
        return 1;
    }

    I2C0->S = I2C_S_IICIF_MASK;
    return 0;
}

static int i2c_start(void)
{
    uint32_t start = milliseconds;

    while (I2C0->S & I2C_S_BUSY_MASK) {
        if ((uint32_t)(milliseconds - start) >= I2C_TIMEOUT_MS) {
            return 1;
        }
    }

    I2C0->S = I2C_S_IICIF_MASK | I2C_S_ARBL_MASK;
    I2C0->C1 &= ~I2C_C1_TXAK_MASK;
    I2C0->C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;

    return 0;
}

static void i2c_stop(void)
{
    I2C0->C1 &= ~(I2C_C1_MST_MASK |
                  I2C_C1_TX_MASK |
                  I2C_C1_TXAK_MASK);

    delayUs(5);
}

static int i2c_write(uint8_t data)
{
    I2C0->D = data;

    if (i2c_wait()) {
        return 1;
    }

    return (I2C0->S & I2C_S_RXAK_MASK) ? 1 : 0;
}

int i2c_write_bytes(uint8_t dev, uint8_t reg,
                    const uint8_t *buf, uint8_t n)
{
    uint8_t i;
    int error = 0;

    if (buf == 0 || n == 0u) {
        return 1;
    }

    if (i2c_start()) {
        return 1;
    }

    if (i2c_write((uint8_t)(dev << 1)) || i2c_write(reg)) {
        error = 1;
    }

    for (i = 0; i < n && !error; i++) {
        if (i2c_write(buf[i])) {
            error = 1;
        }
    }

    i2c_stop();
    return error;
}

int i2c_read_bytes(uint8_t dev, uint8_t reg,
                   uint8_t *buf, uint8_t n)
{
    uint8_t i;
    volatile uint8_t dummy;

    if (buf == 0 || n == 0u) {
        return 1;
    }

    if (i2c_start()) {
        return 1;
    }

    if (i2c_write((uint8_t)(dev << 1)) || i2c_write(reg)) {
        i2c_stop();
        return 1;
    }

    I2C0->C1 |= I2C_C1_RSTA_MASK;

    if (i2c_write((uint8_t)((dev << 1) | 1u))) {
        i2c_stop();
        return 1;
    }

    I2C0->C1 &= ~I2C_C1_TX_MASK;

    if (n == 1u) {
        I2C0->C1 |= I2C_C1_TXAK_MASK;
    } else {
        I2C0->C1 &= ~I2C_C1_TXAK_MASK;
    }

    dummy = I2C0->D;
    (void)dummy;

    for (i = 0; i < n; i++) {
        if (i2c_wait()) {
            i2c_stop();
            return 1;
        }

        if (n > 1u && i == (uint8_t)(n - 2u)) {
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        }

        if (i == (uint8_t)(n - 1u)) {
            I2C0->C1 &= ~I2C_C1_MST_MASK;
        }

        buf[i] = I2C0->D;
    }

    I2C0->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
    delayUs(5);

    return 0;
}

static int i2c_probe(uint8_t address)
{
    int error;

    if (i2c_start()) {
        return 1;
    }

    error = i2c_write((uint8_t)(address << 1));
    i2c_stop();

    return error;
}

uint8_t bcd2bin(uint8_t b)
{
    return (uint8_t)((b >> 4) * 10u + (b & 0x0Fu));
}

uint8_t bin2bcd(uint8_t b)
{
    return (uint8_t)(((b / 10u) << 4) | (b % 10u));
}

uint8_t day_of_week(uint8_t y, uint8_t m, uint8_t d)
{
    static const uint8_t table[12] = {
        0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4
    };

    uint16_t year = (uint16_t)(2000u + y);

    if (m < 1u || m > 12u) {
        return 0;
    }

    if (m < 3u) {
        year--;
    }

    return (uint8_t)(
        ((year + year / 4u - year / 100u + year / 400u +
          table[m - 1u] + d) % 7u) + 1u
    );
}

int rtc_set_time(const rtc_time_t *t)
{
    uint8_t buf[7];

    buf[0] = bin2bcd(t->sec);
    buf[1] = bin2bcd(t->min);
    buf[2] = bin2bcd(t->hour);
    buf[3] = t->dow;
    buf[4] = bin2bcd(t->day);
    buf[5] = bin2bcd(t->month);
    buf[6] = bin2bcd(t->year);

    return i2c_write_bytes(DS3231_ADDR, REG_SECONDS, buf, 7);
}

int rtc_get_time(rtc_time_t *t)
{
    uint8_t buf[7];
    uint8_t hour;

    if (i2c_read_bytes(DS3231_ADDR, REG_SECONDS, buf, 7)) {
        return 1;
    }

    t->sec = bcd2bin(buf[0] & 0x7Fu);
    t->min = bcd2bin(buf[1] & 0x7Fu);

    if (buf[2] & 0x40u) {
        hour = bcd2bin(buf[2] & 0x1Fu);
        hour %= 12u;

        if (buf[2] & 0x20u) {
            hour += 12u;
        }

        t->hour = hour;
    } else {
        t->hour = bcd2bin(buf[2] & 0x3Fu);
    }

    t->dow = buf[3] & 0x07u;
    t->day = bcd2bin(buf[4] & 0x3Fu);
    t->month = bcd2bin(buf[5] & 0x1Fu);
    t->year = bcd2bin(buf[6]);

    return 0;
}

static int rtc_prepare(void)
{
    uint8_t control;
    uint8_t status;

#if FORCE_SET_TIME
    rtc_time_t initial;
#endif

    if (i2c_read_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        return 1;
    }

    control &= (uint8_t)~(CTRL_EOSC | CTRL_A1IE | CTRL_A2IE);
    control |= CTRL_INTCN;

    if (i2c_write_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        return 1;
    }

    if (i2c_read_bytes(DS3231_ADDR, REG_STATUS, &status, 1)) {
        return 1;
    }

#if FORCE_SET_TIME
    initial.sec = INIT_SEC;
    initial.min = INIT_MIN;
    initial.hour = INIT_HOUR;
    initial.day = INIT_DAY;
    initial.month = INIT_MONTH;
    initial.year = INIT_YEAR;
    initial.dow = day_of_week(initial.year,
                              initial.month,
                              initial.day);

    if (rtc_set_time(&initial)) {
        return 1;
    }

    status &= (uint8_t)~STATUS_OSF;
#else
    if (status & STATUS_OSF) {
        return 2;
    }
#endif

    status &= (uint8_t)~(STATUS_A1F | STATUS_A2F);

    if (i2c_write_bytes(DS3231_ADDR, REG_STATUS, &status, 1)) {
        return 1;
    }

    return 0;
}


void SPI0_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD->PCR[1] = PORT_PCR_MUX(2);   /* PTD1 -> SPI0_SCK  */
    PORTD->PCR[2] = PORT_PCR_MUX(2);   /* PTD2 -> SPI0_MOSI */
    PORTD->PCR[0] = PORT_PCR_MUX(1);   /* PTD0 -> GPIO: usado como CS */

    PTD->PDDR |= (1 << 0);             /* CS como salida */
    PTD->PSOR  = (1 << 0);             /* CS en alto = inactivo */

    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK; /* Reloj del módulo SPI0 */

    SPI0->C1  = SPI_C1_MSTR_MASK;      /* Modo maestro (CPOL=0, CPHA=0) */
    SPI0->BR  = 0x60;                  /* Divisor de baud rate (~1-1.5 MHz) */
    SPI0->C1 |= SPI_C1_SPE_MASK;       /* Habilita el módulo SPI */
}

// Envío de un comando y dato de 8 bits al MAX7219 vía SPI
void max7219_write(unsigned char command, unsigned char data)
{
    volatile char dummy;

    PTD->PCOR = (1 << 0);                          /* CS en bajo: inicia trama */

    while (!(SPI0->S & SPI_S_SPTEF_MASK)) { }      /* Espera buffer de TX vacío */
    SPI0->D = command;                             /* Byte 1: registro */
    while (!(SPI0->S & SPI_S_SPRF_MASK)) { }       /* Espera a que termine el byte */
    dummy = SPI0->D;                               /* Lectura para limpiar SPRF */

    while (!(SPI0->S & SPI_S_SPTEF_MASK)) { }
    SPI0->D = data;                                /* Byte 2: dato */
    while (!(SPI0->S & SPI_S_SPRF_MASK)) { }
    dummy = SPI0->D;

    PTD->PSOR = (1 << 0);                          /* CS en alto: fin de trama */
}

// Configuración inicial de los registros del MAX7219
void max7219_init(void)
{
    /* Decode mode 0x0F: los dígitos 0-3 usan "Code B" (decodificación BCD) */
    max7219_write(DECODE, 0x0F);
    max7219_write(SCANLIMIT, 3);   /* Solo escanea 4 dígitos (0 a 3) */
    max7219_write(INTENSITY, 4);   /* Nivel de brillo (0 a 15) */
    max7219_write(TEST, 0);        /* Modo de prueba desactivado */
    max7219_write(SHUTDOWN, 1);    /* 1 = Operación normal */
}

// Formateo e impresión de las horas y minutos en la matriz / display de 7 segmentos
void max7219_show_time(const rtc_time_t *t)
{
    uint8_t dp = (t->sec & 1) ? 0x80 : 0x00;   /* Segundos impares -> punto encendido */

    max7219_write(4, t->hour / 10);            /* Decenas de hora */
    max7219_write(3, (t->hour % 10) | dp);     /* Unidades de hora + punto */
    max7219_write(2, t->min / 10);             /* Decenas de minuto */
    max7219_write(1, t->min % 10);             /* Unidades de minuto */
}

void keypad_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[5] = 0x103;     /* Filas: PTE2..PTE5 con Pull-up/down */
    PORTE->PCR[4] = 0x103;
    PORTE->PCR[3] = 0x103;
    PORTE->PCR[2] = 0x103;

    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* Columnas: PTB8..PTB11 */
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[9]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[8]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PTE->PDDR &= ~(0x0F << 2); /* Entradas por defecto */
    PTB->PDDR &= ~(0x0F << 8);
}

// Escaneo y lectura de la tecla presionada
char keypad_getkey(void)
{
    int row;
    uint32_t col_val;
    const uint8_t row_pins[4] = {5, 4, 3, 2};

    PTE->PDDR |= (0x0F << 2);
    PTE->PCOR  = (0x0F << 2);
    delayUs(2);
    col_val = (PTB->PDIR >> 8) & 0x0F;
    PTE->PDDR &= ~(0x0F << 2);

    if (col_val == 0x0F) return 0;

    /* Barrido fila por fila para detectar la tecla */
    for (row = 0; row < 4; row++) {
        PTE->PDDR &= ~(0x0F << 2);
        PTE->PDDR |= (1 << row_pins[row]);
        PTE->PCOR  = (1 << row_pins[row]);
        delayUs(2);
        col_val = (PTB->PDIR >> 8) & 0x0F;
        if (col_val != 0x0F) break;
    }
    PTE->PDDR &= ~(0x0F << 2);

    if (row == 4) return 0;

    /* Identificación de columna */
    if (col_val == 0x07) return row * 4 + 1;
    if (col_val == 0x0B) return row * 4 + 2;
    if (col_val == 0x0D) return row * 4 + 3;
    if (col_val == 0x0E) return row * 4 + 4;
    return 0;
}

// Procesamiento y detección de flanco de subida al presionar una tecla
void process_keypad(void)
{
    static uint8_t last_code = 0;
    uint8_t code = (uint8_t)keypad_getkey();

    if (code != 0 && last_code == 0) {
        handle_key(keypad_map[code - 1]);
    }
    last_code = code;
}

void handle_key(char key)
{
    if (key == 0) return;

    /* Prioridad 1: Apagar la alarma si está sonando */
    if (alarm_ringing || alarm_irq_pending) {
        if (key == '#') {  // Tecla para silenciar/desactivar la alarma active
            alarm_stop();
            alarm_clear_flag();
            alarm_irq_pending = 0;
            alarm_ringing = 0;

            LCD_goto(0, 0);
            LCD_string("Alarma Apagada! ");
            for (volatile int i = 0; i < 1000000; i++); // Breve retardo visual
            return;
        }
        return; // Ignorar otras teclas mientras suena la alarma hasta presionar '#'
    }

    /* Prioridad 2: Cancelación de modo de configuración */
    if (key == '*') {
        mode = MODE_NORMAL;
        ndigits = 0;
        return;
    }

    /* Prioridad 3: Procesamiento en MODE_NORMAL */
    if (mode == MODE_NORMAL) {
        if (key == 'C') {
            mode = MODE_SET_TIME;
            ndigits = 0;
            memset(digits, 0, sizeof(digits));
            LCD_goto(0, 0);
            LCD_string("Set HHMMSS:     ");
            LCD_goto(1, 0);
            LCD_string("                ");
            return;
        }
        if (key == 'D') {
            mode = MODE_SET_DATE;
            ndigits = 0;
            memset(digits, 0, sizeof(digits));
            LCD_goto(0, 0);
            LCD_string("Set DDMMYY:     ");
            LCD_goto(1, 0);
            LCD_string("                ");
            return;
        }
        if (key == 'A') {
            mode = MODE_SET_ALARM;
            ndigits = 0;
            memset(digits, 0, sizeof(digits));
            LCD_goto(0, 0);
            LCD_string("Set Alarm HHMM: ");
            LCD_goto(1, 0);
            LCD_string("                ");
            return;
        }
        return;
    }

    /* Prioridad 4: Ingreso de dígitos para modos de configuración */
    if (key >= '0' && key <= '9') {
        uint8_t max_len = (mode == MODE_SET_ALARM) ? 4 : 6;
        if (ndigits < max_len) {
        	digits[ndigits++] = (uint8_t)(key - '0');
            LCD_goto(1, ndigits - 1);
            LCD_data(key);
        }
        return;
    }

    /* Confirmar configuración con '#' */
    if (key == '#') {
        uint8_t required_len = (mode == MODE_SET_ALARM) ? 4 : 6;
        if (ndigits == required_len) {
            config_confirm();
        }
    }
}

// Entrada al modo de edición de tiempo
void config_enter(void)
{
    mode = MODE_SET_TIME;
    ndigits = 0;
    LCD_command(0x01);          /* Limpia pantalla LCD */
    delayMs(2);
    config_draw();
}

// Salida del modo de edición
void config_exit(void)
{
    mode = MODE_NORMAL;
    ndigits = 0;
    LCD_command(0x01);
    delayMs(2);
    last_sec_shown = 0xFF;      /* Fuerza actualización en la LCD */
}

// Despliegue dinámico de la plantilla e ingreso de caracteres en la LCD
void config_draw(void)
{
    char v[9];
    const char *tmpl;
    const uint8_t *pos;
    uint8_t i;

    static const uint8_t pos_time[4] = {0, 1, 3, 4};
    static const uint8_t pos_date[6] = {0, 1, 3, 4, 6, 7};

    if (mode == MODE_SET_TIME) { tmpl = "__:__    "; pos = pos_time; }
    else                       { tmpl = "__/__/__"; pos = pos_date; }

    for (i = 0; i < 8; i++) v[i] = tmpl[i];
    v[8] = 0;
    for (i = 0; i < ndigits; i++) v[pos[i]] = digits[i];

    LCD_goto(0, 0);
    LCD_string(mode == MODE_SET_TIME ? "SET TIME (HH:MM)" : "SET DATE (DMY)  ");
    LCD_goto(1, 0);
    LCD_string(v);
    LCD_string(" #OK *NO");
}

// Notificación de valor fuera de rango
void config_show_invalid(void)
{
    LCD_goto(1, 0);
    LCD_string("INVALID VALUE   ");
    delayMs(1500);
    config_draw();
}

 void config_confirm(void)
{
    rtc_time_t t;
    rtc_get_time(&t); // Recuperar la hora actual para no sobrescribir datos no editados

    if (mode == MODE_SET_TIME) {
        t.hour = (digits[0] * 10) + digits[1];
        t.min  = (digits[2] * 10) + digits[3];
        t.sec  = (digits[4] * 10) + digits[5];

        if (t.hour < 24 && t.min < 60 && t.sec < 60) {
            rtc_set_time(&t);
        }
    }
    else if (mode == MODE_SET_DATE) {
        t.day   = (digits[0] * 10) + digits[1];
        t.month = (digits[2] * 10) + digits[3];
        t.year  = (digits[4] * 10) + digits[5];

        if (t.day >= 1 && t.day <= 31 && t.month >= 1 && t.month <= 12) {
            rtc_set_time(&t);
        }
    }
    else if (mode == MODE_SET_ALARM) {
        uint8_t a_hour = (digits[0] * 10) + digits[1];
        uint8_t a_min  = (digits[2] * 10) + digits[3];

        if (a_hour < 24 && a_min < 60) {
            rtc_alarm_set(a_hour, a_min);
            LCD_goto(0, 0);
            LCD_string("Alarm Set OK!   ");
            delayMs(1000);
        }
    }

    mode = MODE_NORMAL;
    ndigits = 0;
}


// Obtención de los días máximos por mes (incluyendo años bisiestos)
uint8_t days_in_month(uint8_t m, uint8_t y)
{
    static const uint8_t dm[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (m == 2 && (y % 4) == 0) return 29;
    return dm[m - 1];
}


void PORTA_IRQHandler(void)
{
    PORTA->ISFR = (1 << RTC_INT_PIN);
    alarm_irq_pending = 1;
    alarm_ringing = 1;
}

void alarm_trigger(void)
{
    // Enciende el buzzer y el LED (lógica inversa o directa según tu hardware)
    PTE->PSOR = BUZZER_MASK;
    PTB->PCOR = LED_RED_MASK;
}

// Inicialización de GPIOs para el Buzzer (PTE1) y LED (PTB18)
static void alarm_outputs_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTB_MASK;

    PORTE->PCR[1] = PORT_PCR_MUX(1);
    PORTB->PCR[18] = PORT_PCR_MUX(1);

    PTE->PCOR = BUZZER_MASK;
    PTB->PSOR = LED_RED_MASK;

    PTE->PDDR |= BUZZER_MASK;
    PTB->PDDR |= LED_RED_MASK;
}

// Desactivación de salidas físicas
static void alarm_outputs_off(void)
{
    PTE->PCOR = BUZZER_MASK;
    PTB->PSOR = LED_RED_MASK;
}

// Configuración del pin de interrupción PTA12 para recibir el pulso de la alarma del RTC
static void rtc_interrupt_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PTA->PDDR &= ~RTC_INT_MASK;

    PORTA->PCR[RTC_INT_PIN] = PORT_PCR_MUX(1) |
                              PORT_PCR_PE_MASK |
                              PORT_PCR_PS_MASK |
                              PORT_PCR_IRQC(0x0A); /* Interrupción por flanco de bajada */

    PORTA->ISFR = RTC_INT_MASK;

    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_SetPriority(PORTA_IRQn, 1u);
    NVIC_EnableIRQ(PORTA_IRQn);
}

// Programación del registro de la Alarma 1 en el DS3231 e inserción de la interrupción
static int rtc_alarm_set(uint8_t hour, uint8_t minute)
{
    uint8_t alarm_registers[4];
    uint8_t control;

    if (hour > 23u || minute > 59u) return 1;

    alarm_armed = 0;
    alarm_irq_pending = 0;
    alarm_ringing = 0;
    alarm_outputs_off();

    alarm_registers[0] = bin2bcd(0);       /* Segundo 0 */
    alarm_registers[1] = bin2bcd(minute);  /* Minuto */
    alarm_registers[2] = bin2bcd(hour);    /* Hora */
    alarm_registers[3] = 0x80u;            /* Dispara por hora, minuto y segundo coincidente */

    if (i2c_write_bytes(DS3231_ADDR, REG_ALARM1, alarm_registers, 4)) return 1;

    if (i2c_read_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) return 1;

    control |= CTRL_INTCN | CTRL_A1IE;    /* Habilita interrupción A1 */
    control &= (uint8_t)~(CTRL_EOSC | CTRL_A2IE);

    PORTA->ISFR = RTC_INT_MASK;
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    alarm_hour = hour;
    alarm_minute = minute;
    alarm_armed = 1;

    if (i2c_write_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        alarm_armed = 0;
        alarm_outputs_off();
        return 1;
    }

    return 0;
}



void alarm_clear_flag(void)
{
    uint8_t status = 0;

    /* 1. Detener actuadores (Buzzer y LED) */
    alarm_stop();

    /* 2. Leer el registro de estado del DS3231 (0x0F / REG_STATUS) */
    if (i2c_read_bytes(DS3231_ADDR, REG_STATUS, &status, 1) == 0) {

        /* 3. Limpiar bit A1F (Alarm 1 Flag, bit 0 / STATUS_A1F) */
        status &= (uint8_t)~STATUS_A1F;

        /* 4. Reescribir el registro de estado actualizado */
        i2c_write_bytes(DS3231_ADDR, REG_STATUS, &status, 1);
    }

    /* 5. Restablecer banderas locales de estado */
    alarm_ringing = 0;
    alarm_irq_pending = 0;
}

// Apagado de la alarma en ejecución y reajuste de banderas del DS3231
static void alarm_stop(void)
{
    alarm_armed = 0;
    alarm_irq_pending = 0;
    alarm_ringing = 0;

    alarm_outputs_off();

    PORTA->ISFR = RTC_INT_MASK;
    NVIC_ClearPendingIRQ(PORTA_IRQn);
}

// Cálculo del polinomio CRC-8 para la validación de comandos e I2C del sensor SCD30
static uint8_t scd30_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFFu;
    uint8_t i, bit;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8u; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ 0x31u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

// Envío de comandos estructurados I2C hacia el SCD30 (con o sin argumento)
static int scd30_command(uint16_t command, uint8_t has_argument, uint16_t argument)
{
    uint8_t frame[5];
    uint8_t length = 2u;

    frame[0] = (uint8_t)(command >> 8);
    frame[1] = (uint8_t)(command & 0xFFu);

    if (has_argument) {
        frame[2] = (uint8_t)(argument >> 8);
        frame[3] = (uint8_t)(argument & 0xFFu);
        frame[4] = scd30_crc8(&frame[2], 2u);
        length = 5u;
    }

    if (i2c_write_raw(SCD30_ADDR, frame, length)) return 1;

    delayMs(10);
    return 0;
}

// Inicialización del sensor SCD30 (detención previa, intervalo y disparo de lectura continuada)
static int scd30_init(void)
{
    if (i2c_probe(SCD30_ADDR)) return 1;
    if (scd30_command(SCD30_CMD_STOP, 0u, 0u)) return 1;
    if (scd30_command(SCD30_CMD_INTERVAL, 1u, SCD30_INTERVAL_SECONDS)) return 1;
    if (scd30_command(SCD30_CMD_START, 1u, 0u)) return 1;

    return 0;
}

// Lectura del valor de temperatura del SCD30 con deserialización de datos flotantes (IEEE-754)
static int scd30_read_temperature(int16_t *tenths)
{
    uint8_t ready_data[3];
    uint8_t measurement[18];
    uint8_t i;
    uint16_t ready;
    uint32_t bits;
    float temperature;
    float scaled;

    if (tenths == 0) return SCD30_ERROR_VALUE;

    /* Consulta de datos listos */
    if (scd30_command(SCD30_CMD_READY, 0u, 0u)) return SCD30_ERROR_I2C;
    if (i2c_read_raw(SCD30_ADDR, ready_data, 3u)) return SCD30_ERROR_I2C;
    if (scd30_crc8(ready_data, 2u) != ready_data[2]) return SCD30_ERROR_CRC;

    ready = (uint16_t)(((uint16_t)ready_data[0] << 8) | ready_data[1]);
    if (ready == 0u) return SCD30_NOT_READY;
    if (ready != 1u) return SCD30_ERROR_VALUE;

    /* Lectura de la ráfaga de mediciones */
    if (scd30_command(SCD30_CMD_READ, 0u, 0u)) return SCD30_ERROR_I2C;
    if (i2c_read_raw(SCD30_ADDR, measurement, 18u)) return SCD30_ERROR_I2C;

    /* Verificación de CRC para cada bloque de 3 bytes (2 datos + 1 CRC) */
    for (i = 0; i < 18u; i += 3u) {
        if (scd30_crc8(&measurement[i], 2u) != measurement[i + 2u]) {
            return SCD30_ERROR_CRC;
        }
    }

    /* Extrae la temperatura flotante de 32 bits (bytes 6, 7, 9, 10 del arreglo) */
    bits = ((uint32_t)measurement[6] << 24) |
           ((uint32_t)measurement[7] << 16) |
           ((uint32_t)measurement[9] << 8)  |
           ((uint32_t)measurement[10]);

    memcpy(&temperature, &bits, sizeof(temperature));

    if (!(temperature >= -40.0f && temperature <= 125.0f)) {
        return SCD30_ERROR_VALUE;
    }

    scaled = temperature * 10.0f;
    *tenths = (scaled >= 0.0f) ? (int16_t)(scaled + 0.5f) : (int16_t)(scaled - 0.5f);

    return SCD30_OK;
}

void display_temperature(void)
{
    char buf[17];
    int16_t temp;

    if (!temperature_valid) {
        lcd_line(1, "Temp: ERROR SCD30");
        return;
    }

    temp = temperature_tenths;
    if (temp < 0) {
        temp = -temp;
        snprintf(buf, sizeof(buf), "Temp: -%d.%d C   ", temp / 10, temp % 10);
    } else {
        snprintf(buf, sizeof(buf), "Temp:  %d.%d C   ", temp / 10, temp % 10);
    }

    LCD_goto(1, 0);
    LCD_string(buf);
}



// funciones de la LCD -------------------------------------------
void LCD_init(void) //inicializamos pines
{
    SIM->SCGC5     |= 0x1000;
    PORTD->PCR[4]   = 0x100;
    PORTD->PCR[5]   = 0x100;
    PORTD->PCR[6]   = 0x100;
    PORTD->PCR[7]   = 0x100;
    PTD->PDDR       = 0xFF;
    SIM->SCGC5     |= 0x0200;
    PORTA->PCR[2]   = 0x100;
    PORTA->PCR[4]   = 0x100;
    PORTA->PCR[5]   = 0x100;
    PTA->PDDR      |= 0x34;

    PTA->PCOR = RS | RW;
    delayMs(100);

    PTD->PDOR = (PTD->PDOR & 0x0F) | (0x03 << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN; delayMs(20);
    PTD->PDOR = (PTD->PDOR & 0x0F) | (0x03 << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN; delayMs(5);
    PTD->PDOR = (PTD->PDOR & 0x0F) | (0x03 << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN; delayMs(5);
    PTD->PDOR = (PTD->PDOR & 0x0F) | (0x02 << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN; delayMs(5);

    LCD_command(0x28);
    LCD_command(0x06);
    LCD_command(0x01);
    delayMs(4);
    LCD_command(0x0C);   /* display on, cursor off */
}

void LCD_command(unsigned char command)
{
    PTA->PCOR = RS | RW;
    PTD->PDOR = (PTD->PDOR & 0x0F) | ((command >> 4) << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN;
    PTD->PDOR = (PTD->PDOR & 0x0F) | ((command & 0x0F) << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN;
    delayMs((command < 4) ? 4 : 1);
}

void LCD_data(unsigned char data)
{
    PTA->PSOR = RS;
    PTA->PCOR = RW;
    PTD->PDOR = (PTD->PDOR & 0x0F) | ((data >> 4) << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN;
    PTD->PDOR = (PTD->PDOR & 0x0F) | ((data & 0x0F) << 4);
    PTA->PSOR = EN; delayMs(1); PTA->PCOR = EN;
    delayMs(1);
}

void LCD_string(const char cadena[])
{
    int i = 0;
    while (cadena[i] != 0) { LCD_data((unsigned char)cadena[i]); i++; }
}

void LCD_goto(uint8_t row, uint8_t col)
{
    LCD_command((row ? 0xC0 : 0x80) + col);
}

void lcd_line(uint8_t row, const char *str)
{
    LCD_goto(row, 0);
    LCD_string((char*)str);
}

void LCD_2digits(uint8_t n)
{
    LCD_data('0' + (n / 10) % 10);
    LCD_data('0' + (n % 10));
}

static int i2c_write_raw(uint8_t address,
                          const uint8_t *data,
                          uint8_t length)
{
    uint8_t i;
    int error;

    if (data == 0 || length == 0u) {
        return 1;
    }

    if (i2c_start()) {
        return 1;
    }

    error = i2c_write((uint8_t)(address << 1));

    for (i = 0; i < length && !error; i++) {
        error = i2c_write(data[i]);
    }

    i2c_stop();
    return error;
}

static int i2c_read_raw(uint8_t address,
                         uint8_t *data,
                         uint8_t length)
{
    uint8_t i;
    volatile uint8_t dummy;

    if (data == 0 || length == 0u) {
        return 1;
    }

    if (i2c_start()) {
        return 1;
    }

    if (i2c_write((uint8_t)((address << 1) | 1u))) {
        i2c_stop();
        return 1;
    }

    I2C0->C1 &= ~I2C_C1_TX_MASK;

    if (length == 1u) {
        I2C0->C1 |= I2C_C1_TXAK_MASK;
    } else {
        I2C0->C1 &= ~I2C_C1_TXAK_MASK;
    }

    dummy = I2C0->D;
    (void)dummy;

    for (i = 0; i < length; i++) {
        if (i2c_wait()) {
            i2c_stop();
            return 1;
        }

        if (length > 1u && i == (uint8_t)(length - 2u)) {
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        }

        if (i == (uint8_t)(length - 1u)) {
            I2C0->C1 &= ~I2C_C1_MST_MASK;
        }

        data[i] = I2C0->D;
    }

    I2C0->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
    delayUs(5);

    return 0;
}




int main(void)
{
    rtc_time_t current_time;
    uint8_t last_sec_shown = 255; // Fuerza el primer refresco de pantalla

    /* -------------------------------------------------------------
     * 1. Inicialización de Periféricos y Módulos
     * ------------------------------------------------------------- */
    timebase_init();       // Base de tiempo (milisegundos)
    I2C0_init();           // Bus I2C para RTC DS3231 y Sensor SCD30
    SPI0_init();           // Bus SPI para controlador MAX7219
    max7219_init();        // Display de 7 segmentos
    LCD_init();            // Pantalla LCD 16x2
    keypad_init();         // Teclado matricial
    alarm_outputs_init();  // Salidas para Buzzer / LED de alarma
    rtc_interrupt_init();  // Configuración del pin de interrupción SQW/INT del RTC
    rtc_prepare();         // Preparar registros base del RTC DS3231
    scd30_init();          // Inicialización del sensor SCD30

    /* Mensaje de bienvenida en pantalla LCD */
    LCD_goto(0, 0);
    LCD_string("  Sistema RTC   ");
    LCD_goto(1, 0);
    LCD_string("  Inicializado  ");
    delayMs(1500);         // Retardo inicial usando la base de tiempo

    /* -------------------------------------------------------------
     * 2. Bucle Principal (Super-Loop)
     * ------------------------------------------------------------- */
    while (1)
    {
        /* A. Procesar entradas del teclado matricial */
        process_keypad();

        /* B. Muestreo periódico del sensor de temperatura SCD30 (Cada 2 s) */
        scd30_task();

        /* C. Gestión de Alarma Activa (Disparada por Interrupción / RTC) */
        if (alarm_irq_pending || alarm_ringing)
        {
            // Activar salidas físicas (Buzzer y LED)
            alarm_trigger();

            // Desplegar mensaje de advertencia en LCD
            LCD_goto(0, 0);
            LCD_string("*** ALARMA ***  ");
            LCD_goto(1, 0);
            LCD_string("Presione # apagar");
        }
        /* D. Operación Normal del Sistema */
        else if (mode == MODE_NORMAL)
        {
            // Leer hora y fecha unificadas en current_time desde el RTC DS3231
            if (rtc_get_time(&current_time) == 0)
            {
                // Refrescar en tiempo real los displays de 7 segmentos (MAX7219)
                max7219_show_time(&current_time);

                // Refrescar la pantalla LCD solo cuando cambie el segundo
                if (current_time.sec != last_sec_shown)
                {
                    last_sec_shown = current_time.sec;
                    update_display_normal(&current_time);
                }
            }
        }
        /* E. Modos de Configuración (MODE_SET_TIME, MODE_SET_DATE, MODE_SET_ALARM) */
        else
        {
            // Forzar refresco inmediato de la LCD al regresar a MODE_NORMAL
            last_sec_shown = 255;
        }
    }

    return 0;
}
