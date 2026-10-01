#include <MKL25Z4.h>
#include <stdint.h>
#include <string.h>
#define RS                      (1u << 2)
#define RW                      (1u << 4)
#define EN                      (1u << 5)
#define LCD_DATA_MASK           (0x0Fu << 4)
#define RTC_INT_PIN             12u
#define RTC_INT_MASK            (1u << RTC_INT_PIN)
#define BUZZER_MASK             (1u << 1)
#define LED_RED_MASK            (1u << 18)
#define KEYPAD_ROWS_MASK        (0x0Fu << 2)
#define KEYPAD_COLS_MASK        (0x0Fu << 8)
#define DS3231_ADDR             0x68u
#define REG_SECONDS             0x00u
#define REG_ALARM1              0x07u
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

typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t dow;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} rtc_time_t;

typedef enum {
    MODE_NORMAL = 0,
    MODE_SET_ALARM
} system_mode_t;

static const char keypad_map[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

static volatile uint32_t milliseconds = 0;
static volatile uint8_t alarm_irq_pending = 0;
static volatile uint8_t alarm_armed = 0;

static system_mode_t mode = MODE_NORMAL;

static uint8_t alarm_ringing = 0;
static uint8_t alarm_hour = 0;
static uint8_t alarm_minute = 0;
static char digits[4];
static uint8_t ndigits = 0;

static rtc_time_t now;
static uint8_t rtc_read_ok = 0;
static uint8_t redraw = 1;

static uint8_t sensor_initialized = 0;
static uint8_t temperature_valid = 0;
static uint8_t sensor_error = 0;
static int16_t temperature_tenths = 0;
static uint32_t temperature_updated_at = 0;

static int timebase_init(void);
static void delayMs(uint32_t n);
static void delayUs(uint32_t n);

void LCD_init(void);
void LCD_command(uint8_t command);
void LCD_data(uint8_t data);
void LCD_string(const char *text);
void LCD_goto(uint8_t row, uint8_t col);
void LCD_2digits(uint8_t n);

static void lcd_write_nibble(uint8_t nibble);
static void lcd_line(uint8_t row, const char *text);

void I2C0_init(void);
static int i2c_wait(void);
static int i2c_start(void);
static void i2c_stop(void);
static int i2c_write(uint8_t data);

int i2c_write_bytes(uint8_t dev, uint8_t reg,
                    const uint8_t *buf, uint8_t n);

int i2c_read_bytes(uint8_t dev, uint8_t reg,
                   uint8_t *buf, uint8_t n);

static int i2c_probe(uint8_t address);

static int i2c_write_raw(uint8_t address,
                          const uint8_t *data,
                          uint8_t length);

static int i2c_read_raw(uint8_t address,
                         uint8_t *data,
                         uint8_t length);

uint8_t bcd2bin(uint8_t b);
uint8_t bin2bcd(uint8_t b);
uint8_t day_of_week(uint8_t y, uint8_t m, uint8_t d);

int rtc_set_time(const rtc_time_t *t);
int rtc_get_time(rtc_time_t *t);

static int rtc_prepare(void);
static int rtc_clear_alarm_flags(void);
static int rtc_alarm_disable(void);
static int rtc_alarm_set(uint8_t hour, uint8_t minute);

void keypad_init(void);
uint8_t keypad_getkey(void);
static void process_keypad(void);
static void handle_key(char key);

static void alarm_outputs_init(void);
static void alarm_outputs_off(void);
static void rtc_interrupt_init(void);

static void config_enter(void);
static void config_draw(void);
static void config_confirm(void);

static void alarm_stop(void);
static void display_normal(void);
static void display_alarm(void);
static void display_temperature(void);
static void show_message(const char *line1, const char *line2);

static uint8_t scd30_crc8(const uint8_t *data, uint8_t length);

static int scd30_command(uint16_t command,
                          uint8_t has_argument,
                          uint16_t argument);

static int scd30_init(void);
static int scd30_read_temperature(int16_t *tenths);

void SysTick_Handler(void)
{
    milliseconds++;
}

void PORTA_IRQHandler(void)
{
    if (PORTA->ISFR & RTC_INT_MASK) {
        PORTA->ISFR = RTC_INT_MASK;

        if (alarm_armed) {
            PTE->PSOR = BUZZER_MASK;
            PTB->PCOR = LED_RED_MASK;
            alarm_irq_pending = 1;
        }
    }
}

int main(void)
{
    uint32_t last_rtc_read;
    uint32_t last_sensor_poll;
    uint32_t last_sensor_retry;
    uint8_t rtc_present;
    uint8_t sensor_present;
    int result;

    if (timebase_init() != 0) {
        while (1) {
        }
    }

    alarm_outputs_init();
    LCD_init();
    I2C0_init();
    keypad_init();

    lcd_line(0, "Iniciando I2C");
    lcd_line(1, "RTC + SCD30");
    delayMs(2000);

    rtc_present = (i2c_probe(DS3231_ADDR) == 0);
    sensor_present = (i2c_probe(SCD30_ADDR) == 0);

    lcd_line(0, rtc_present ?
                 "RTC 0x68: OK" :
                 "RTC 0x68: ERROR");

    lcd_line(1, sensor_present ?
                 "SCD 0x61: OK" :
                 "SCD 0x61: ERROR");

    delayMs(1500);

    if (!rtc_present) {
        lcd_line(0, "RTC NO DETECTADO");
        lcd_line(1, "Revisar PTC8/9");

        while (1) {
        }
    }

    result = rtc_prepare();

    if (result == 1) {
        lcd_line(0, "ERROR RTC / I2C");
        lcd_line(1, "Revisar conexion");

        while (1) {
        }
    }

    if (result == 2) {
        lcd_line(0, "RTC SIN HORA");
        lcd_line(1, "FORCE_SET_TIME=1");

        while (1) {
        }
    }

    rtc_interrupt_init();
    rtc_read_ok = (rtc_get_time(&now) == 0);

    if (sensor_present && scd30_init() == 0) {
        sensor_initialized = 1;
        sensor_error = 0;
    } else {
        sensor_initialized = 0;
        sensor_error = 1;
    }

    temperature_valid = 0;

    last_rtc_read = milliseconds;
    last_sensor_poll = milliseconds;
    last_sensor_retry = milliseconds;
    temperature_updated_at = milliseconds;

    display_normal();
    redraw = 0;

    while (1) {
        if (alarm_irq_pending) {
            alarm_irq_pending = 0;

            if (alarm_armed) {
                alarm_ringing = 1;
                mode = MODE_NORMAL;
                ndigits = 0;
                redraw = 1;
            }
        }

        process_keypad();

        if ((uint32_t)(milliseconds - last_rtc_read) >= 1000u) {
            last_rtc_read = milliseconds;
            rtc_read_ok = (rtc_get_time(&now) == 0);

            if (mode == MODE_NORMAL && !alarm_ringing) {
                redraw = 1;
            }
        }

        if (!sensor_initialized &&
            (uint32_t)(milliseconds - last_sensor_retry) >= 5000u) {

            last_sensor_retry = milliseconds;

            if (scd30_init() == 0) {
                sensor_initialized = 1;
                sensor_error = 0;
                temperature_valid = 0;
                temperature_updated_at = milliseconds;
                last_sensor_poll = milliseconds;
                redraw = 1;
            }
        }

        if (sensor_initialized &&
            (uint32_t)(milliseconds - last_sensor_poll) >= 500u) {

            last_sensor_poll = milliseconds;

            result = scd30_read_temperature(&temperature_tenths);

            if (result == SCD30_OK) {
                temperature_valid = 1;
                sensor_error = 0;
                temperature_updated_at = milliseconds;

                if (mode == MODE_NORMAL && !alarm_ringing) {
                    redraw = 1;
                }
            } else if (result == SCD30_NOT_READY) {
                if ((uint32_t)(
                        milliseconds - temperature_updated_at
                    ) >= 10000u) {

                    sensor_error = 1;
                    temperature_valid = 0;
                    sensor_initialized = 0;
                    last_sensor_retry = milliseconds;
                    redraw = 1;
                }
            } else {
                sensor_error = 1;
                temperature_valid = 0;

                if (result == SCD30_ERROR_I2C) {
                    sensor_initialized = 0;
                    last_sensor_retry = milliseconds;
                }

                if (mode == MODE_NORMAL && !alarm_ringing) {
                    redraw = 1;
                }
            }
        }

        if (alarm_irq_pending) {
            alarm_irq_pending = 0;

            if (alarm_armed) {
                alarm_ringing = 1;
                mode = MODE_NORMAL;
                ndigits = 0;
                redraw = 1;
            }
        }

        if (redraw) {
            redraw = 0;

            if (alarm_ringing) {
                display_alarm();
            } else if (mode == MODE_SET_ALARM) {
                config_draw();
            } else {
                display_normal();
            }
        }

        delayMs(5);
    }
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

static void lcd_write_nibble(uint8_t nibble)
{
    PTD->PCOR = LCD_DATA_MASK;
    PTD->PSOR = ((uint32_t)nibble & 0x0Fu) << 4;

    delayUs(2);
    PTA->PSOR = EN;
    delayUs(2);
    PTA->PCOR = EN;
    delayUs(2);
}

void LCD_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK |
                  SIM_SCGC5_PORTA_MASK;

    PORTD->PCR[4] = PORT_PCR_MUX(1);
    PORTD->PCR[5] = PORT_PCR_MUX(1);
    PORTD->PCR[6] = PORT_PCR_MUX(1);
    PORTD->PCR[7] = PORT_PCR_MUX(1);

    PORTA->PCR[2] = PORT_PCR_MUX(1);
    PORTA->PCR[4] = PORT_PCR_MUX(1);
    PORTA->PCR[5] = PORT_PCR_MUX(1);

    PTD->PCOR = LCD_DATA_MASK;
    PTA->PCOR = RS | RW | EN;

    PTD->PDDR |= LCD_DATA_MASK;
    PTA->PDDR |= RS | RW | EN;

    delayMs(50);

    lcd_write_nibble(0x03);
    delayMs(5);

    lcd_write_nibble(0x03);
    delayMs(1);

    lcd_write_nibble(0x03);
    delayMs(1);

    lcd_write_nibble(0x02);
    delayMs(1);

    LCD_command(0x28);
    LCD_command(0x08);
    LCD_command(0x01);
    LCD_command(0x06);
    LCD_command(0x0C);
}

void LCD_command(uint8_t command)
{
    PTA->PCOR = RS | RW;

    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command & 0x0Fu);

    if (command == 0x01u || command == 0x02u) {
        delayMs(2);
    } else {
        delayUs(50);
    }
}

void LCD_data(uint8_t data)
{
    PTA->PSOR = RS;
    PTA->PCOR = RW;

    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data & 0x0Fu);

    delayUs(50);
}

void LCD_string(const char *text)
{
    while (*text != '\0') {
        LCD_data((uint8_t)*text++);
    }
}

void LCD_goto(uint8_t row, uint8_t col)
{
    LCD_command((uint8_t)((row ? 0xC0u : 0x80u) + col));
}

void LCD_2digits(uint8_t n)
{
    LCD_data((uint8_t)('0' + (n / 10u) % 10u));
    LCD_data((uint8_t)('0' + n % 10u));
}

static void lcd_line(uint8_t row, const char *text)
{
    uint8_t i = 0;

    LCD_goto(row, 0);

    while (i < 16u && *text != '\0') {
        LCD_data((uint8_t)*text++);
        i++;
    }

    while (i < 16u) {
        LCD_data(' ');
        i++;
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

static int rtc_clear_alarm_flags(void)
{
    uint8_t status;

    if (i2c_read_bytes(DS3231_ADDR, REG_STATUS, &status, 1)) {
        return 1;
    }

    status &= (uint8_t)~(STATUS_A1F | STATUS_A2F);

    return i2c_write_bytes(DS3231_ADDR, REG_STATUS, &status, 1);
}

static int rtc_alarm_disable(void)
{
    uint8_t control;

    if (i2c_read_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        return 1;
    }

    control |= CTRL_INTCN;
    control &= (uint8_t)~(CTRL_A1IE | CTRL_A2IE);

    return i2c_write_bytes(DS3231_ADDR, REG_CONTROL, &control, 1);
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

static int rtc_alarm_set(uint8_t hour, uint8_t minute)
{
    uint8_t alarm_registers[4];
    uint8_t control;

    if (hour > 23u || minute > 59u) {
        return 1;
    }

    if (rtc_alarm_disable()) {
        return 1;
    }

    alarm_armed = 0;
    alarm_irq_pending = 0;
    alarm_ringing = 0;
    alarm_outputs_off();

    alarm_registers[0] = bin2bcd(0);
    alarm_registers[1] = bin2bcd(minute);
    alarm_registers[2] = bin2bcd(hour);
    alarm_registers[3] = 0x80u;

    if (i2c_write_bytes(DS3231_ADDR, REG_ALARM1,
                        alarm_registers, 4)) {
        return 1;
    }

    if (rtc_clear_alarm_flags()) {
        return 1;
    }

    if (i2c_read_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        return 1;
    }

    control |= CTRL_INTCN | CTRL_A1IE;
    control &= (uint8_t)~(CTRL_EOSC | CTRL_A2IE);

    PORTA->ISFR = RTC_INT_MASK;
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    alarm_hour = hour;
    alarm_minute = minute;
    alarm_armed = 1;

    if (i2c_write_bytes(DS3231_ADDR, REG_CONTROL, &control, 1)) {
        alarm_armed = 0;
        alarm_irq_pending = 0;
        alarm_outputs_off();
        (void)rtc_alarm_disable();
        return 1;
    }

    return 0;
}

static void alarm_outputs_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK |
                  SIM_SCGC5_PORTB_MASK;

    PORTE->PCR[1] = PORT_PCR_MUX(1);
    PORTB->PCR[18] = PORT_PCR_MUX(1);

    PTE->PCOR = BUZZER_MASK;
    PTB->PSOR = LED_RED_MASK;

    PTE->PDDR |= BUZZER_MASK;
    PTB->PDDR |= LED_RED_MASK;
}

static void alarm_outputs_off(void)
{
    PTE->PCOR = BUZZER_MASK;
    PTB->PSOR = LED_RED_MASK;
}

static void rtc_interrupt_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PTA->PDDR &= ~RTC_INT_MASK;

    PORTA->PCR[RTC_INT_PIN] =
        PORT_PCR_MUX(1) |
        PORT_PCR_PE_MASK |
        PORT_PCR_PS_MASK |
        PORT_PCR_IRQC(0x0A);

    PORTA->ISFR = RTC_INT_MASK;

    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_SetPriority(PORTA_IRQn, 1u);
    NVIC_EnableIRQ(PORTA_IRQn);
}

void keypad_init(void)
{
    uint8_t pin;

    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTE_MASK;

    for (pin = 2; pin <= 5; pin++) {
        PORTE->PCR[pin] = PORT_PCR_MUX(1) |
                          PORT_PCR_PE_MASK |
                          PORT_PCR_PS_MASK;
    }

    for (pin = 8; pin <= 11; pin++) {
        PORTB->PCR[pin] = PORT_PCR_MUX(1) |
                          PORT_PCR_PE_MASK |
                          PORT_PCR_PS_MASK;
    }

    PTE->PCOR = KEYPAD_ROWS_MASK;
    PTE->PDDR &= ~KEYPAD_ROWS_MASK;
    PTB->PDDR &= ~KEYPAD_COLS_MASK;
}

uint8_t keypad_getkey(void)
{
    static const uint8_t row_pins[4] = {5, 4, 3, 2};
    uint8_t row;
    uint8_t columns;

    PTE->PDDR &= ~KEYPAD_ROWS_MASK;
    PTE->PCOR = KEYPAD_ROWS_MASK;

    for (row = 0; row < 4u; row++) {
        PTE->PDDR |= 1u << row_pins[row];
        delayUs(5);

        columns = (uint8_t)((PTB->PDIR >> 8) & 0x0Fu);

        PTE->PDDR &= ~(1u << row_pins[row]);

        switch (columns) {
            case 0x07:
                return (uint8_t)(row * 4u + 1u);

            case 0x0B:
                return (uint8_t)(row * 4u + 2u);

            case 0x0D:
                return (uint8_t)(row * 4u + 3u);

            case 0x0E:
                return (uint8_t)(row * 4u + 4u);

            case 0x0F:
                break;

            default:
                return 0;
        }
    }

    return 0;
}

static void process_keypad(void)
{
    static uint8_t candidate = 0;
    static uint8_t stable = 0;
    static uint32_t changed_at = 0;

    uint8_t code = keypad_getkey();
    uint32_t tick = milliseconds;

    if (code != candidate) {
        candidate = code;
        changed_at = tick;
        return;
    }

    if (candidate != stable &&
        (uint32_t)(tick - changed_at) >= 30u) {

        uint8_t previous = stable;
        stable = candidate;

        if (stable != 0u && previous == 0u) {
            handle_key(keypad_map[stable - 1u]);
        }
    }
}

static void handle_key(char key)
{
    if (alarm_ringing || alarm_irq_pending) {
        if (key == '#') {
            alarm_stop();
        }
        return;
    }

    if (mode == MODE_NORMAL) {
        if (key == 'A') {
            config_enter();
        } else if (key == '*') {
            alarm_stop();
        }
        return;
    }

    if (key >= '0' && key <= '9') {
        if (ndigits < 4u) {
            digits[ndigits++] = key;
            redraw = 1;
        }
    } else if (key == 'D') {
        if (ndigits > 0u) {
            ndigits--;
            redraw = 1;
        }
    } else if (key == '*') {
        mode = MODE_NORMAL;
        ndigits = 0;
        redraw = 1;
    } else if (key == '#') {
        config_confirm();
    }
}

static void config_enter(void)
{
    mode = MODE_SET_ALARM;
    ndigits = 0;
    redraw = 1;
}

static void config_draw(void)
{
    static const uint8_t positions[4] = {0, 1, 3, 4};
    char line[17] = "__:__ #OK *NO   ";
    uint8_t i;

    for (i = 0; i < ndigits; i++) {
        line[positions[i]] = digits[i];
    }

    lcd_line(0, "ALARMA HH:MM");
    lcd_line(1, line);
}

static void config_confirm(void)
{
    uint8_t hour;
    uint8_t minute;
    uint32_t start;

    if (ndigits != 4u) {
        show_message("FALTAN DIGITOS", "Escribe HHMM");
        return;
    }

    hour = (uint8_t)((digits[0] - '0') * 10 +
                     digits[1] - '0');

    minute = (uint8_t)((digits[2] - '0') * 10 +
                       digits[3] - '0');

    if (hour > 23u || minute > 59u) {
        show_message("HORA INVALIDA", "Usa 00:00-23:59");
        return;
    }

    if (rtc_alarm_set(hour, minute)) {
        show_message("ERROR AL PROGRAM", "Revisar RTC/I2C");
        return;
    }

    mode = MODE_NORMAL;
    ndigits = 0;

    lcd_line(0, "ALARM SET");

    LCD_goto(1, 0);
    LCD_2digits(alarm_hour);
    LCD_data(':');
    LCD_2digits(alarm_minute);
    LCD_string("           ");

    start = milliseconds;

    while ((uint32_t)(milliseconds - start) < 1000u) {
        if (alarm_irq_pending) {
            break;
        }
    }

    redraw = 1;
}

static void alarm_stop(void)
{
    if (rtc_alarm_disable()) {
        show_message("ERROR AL APAGAR", "# o * reintentar");
        return;
    }

    alarm_armed = 0;
    alarm_irq_pending = 0;
    alarm_ringing = 0;

    alarm_outputs_off();

    PORTA->ISFR = RTC_INT_MASK;
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    mode = MODE_NORMAL;
    ndigits = 0;
    redraw = 1;

    if (rtc_clear_alarm_flags()) {
        show_message("ALARMA APAGADA", "Error limpiar RTC");
    }
}

static void display_normal(void)
{
    if (rtc_read_ok) {
        LCD_goto(0, 0);
        LCD_string("Time: ");
        LCD_2digits(now.hour);
        LCD_data(':');
        LCD_2digits(now.min);
        LCD_data(':');
        LCD_2digits(now.sec);
        LCD_string("  ");
    } else {
        lcd_line(0, "ERROR LEER RTC");
    }

    display_temperature();
}

static void display_alarm(void)
{
    lcd_line(0, "*** ALARMA ***");
    lcd_line(1, "# para detener");
}

static void show_message(const char *line1, const char *line2)
{
    uint32_t start;

    lcd_line(0, line1);
    lcd_line(1, line2);

    start = milliseconds;

    while ((uint32_t)(milliseconds - start) < 800u) {
        if (alarm_irq_pending) {
            break;
        }
    }

    redraw = 1;
}

static uint8_t scd30_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFFu;
    uint8_t i;
    uint8_t bit;

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

static int scd30_command(uint16_t command,
                          uint8_t has_argument,
                          uint16_t argument)
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

    if (i2c_write_raw(SCD30_ADDR, frame, length)) {
        return 1;
    }

    delayMs(10);
    return 0;
}

static int scd30_init(void)
{
    if (i2c_probe(SCD30_ADDR)) {
        return 1;
    }

    if (scd30_command(SCD30_CMD_STOP, 0u, 0u)) {
        return 1;
    }

    if (scd30_command(SCD30_CMD_INTERVAL,
                      1u, SCD30_INTERVAL_SECONDS)) {
        return 1;
    }

    if (scd30_command(SCD30_CMD_START, 1u, 0u)) {
        return 1;
    }

    return 0;
}

static int scd30_read_temperature(int16_t *tenths)
{
    uint8_t ready_data[3];
    uint8_t measurement[18];
    uint8_t i;
    uint16_t ready;
    uint32_t bits;
    float temperature;
    float scaled;

    if (tenths == 0) {
        return SCD30_ERROR_VALUE;
    }

    if (scd30_command(SCD30_CMD_READY, 0u, 0u)) {
        return SCD30_ERROR_I2C;
    }

    if (i2c_read_raw(SCD30_ADDR, ready_data, 3u)) {
        return SCD30_ERROR_I2C;
    }

    if (scd30_crc8(ready_data, 2u) != ready_data[2]) {
        return SCD30_ERROR_CRC;
    }

    ready = (uint16_t)(
        ((uint16_t)ready_data[0] << 8) | ready_data[1]
    );

    if (ready == 0u) {
        return SCD30_NOT_READY;
    }

    if (ready != 1u) {
        return SCD30_ERROR_VALUE;
    }

    if (scd30_command(SCD30_CMD_READ, 0u, 0u)) {
        return SCD30_ERROR_I2C;
    }

    if (i2c_read_raw(SCD30_ADDR, measurement, 18u)) {
        return SCD30_ERROR_I2C;
    }

    for (i = 0; i < 18u; i += 3u) {
        if (scd30_crc8(&measurement[i], 2u) !=
            measurement[i + 2u]) {
            return SCD30_ERROR_CRC;
        }
    }

    bits = ((uint32_t)measurement[6] << 24) |
           ((uint32_t)measurement[7] << 16) |
           ((uint32_t)measurement[9] << 8) |
           ((uint32_t)measurement[10]);

    memcpy(&temperature, &bits, sizeof(temperature));

    if (!(temperature >= -40.0f && temperature <= 125.0f)) {
        return SCD30_ERROR_VALUE;
    }

    scaled = temperature * 10.0f;

    if (scaled >= 0.0f) {
        *tenths = (int16_t)(scaled + 0.5f);
    } else {
        *tenths = (int16_t)(scaled - 0.5f);
    }

    return SCD30_OK;
}

static void display_temperature(void)
{
    char line[17];
    uint8_t i;
    uint8_t position;
    uint16_t magnitude;
    uint16_t whole;

    if (sensor_error) {
        lcd_line(1, "Temp: ERROR SCD30");
        return;
    }

    if (!temperature_valid) {
        lcd_line(1, "Temp: esperando");
        return;
    }

    for (i = 0; i < 16u; i++) {
        line[i] = ' ';
    }

    line[16] = '\0';
    memcpy(line, "Temp: ", 6u);
    position = 6u;

    if (temperature_tenths < 0) {
        line[position++] = '-';
        magnitude = (uint16_t)(-temperature_tenths);
    } else {
        magnitude = (uint16_t)temperature_tenths;
    }

    whole = magnitude / 10u;

    if (whole >= 100u) {
        line[position++] = (char)('0' + whole / 100u);
    }

    if (whole >= 10u) {
        line[position++] = (char)('0' + (whole / 10u) % 10u);
    }

    line[position++] = (char)('0' + whole % 10u);
    line[position++] = '.';
    line[position++] = (char)('0' + magnitude % 10u);
    line[position++] = ' ';
    line[position++] = 'C';

    lcd_line(1, line);
}
