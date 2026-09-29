//Practica de I2C parte 2
//SPI Display with MAX7219

#include <MKL25Z4.h>
#include <stdio.h>

//LCD -------------
#define RS 0x04
#define RW 0x10
#define EN 0x20

//MAX7219
#define DECODE    9
#define INTENSITY 10
#define SCANLIMIT 11
#define SHUTDOWN  12
#define TEST      15

//I2C - DS3231
#define DS3231_ADDR   0x68
#define REG_SECONDS   0x00
#define REG_CONTROL   0x0E
#define REG_STATUS    0x0F

#define I2C_ICR 0x16

//fecja y hora iniciales
#define FORCE_SET_TIME 0
#define INIT_YEAR 26
#define INIT_MONTH 9
#define INIT_DAY 29
#define INIT_HOUR 15
#define INIT_MIN 30
#define INIT_SEC 0

typedef struct {
    uint8_t sec, min, hour;     /* 24 h */
    uint8_t dow;                /* 1..7 */
    uint8_t day, month, year;   /* year = 0..99 */
} rtc_time_t;

uint8_t day_of_week(uint8_t y, uint8_t m, uint8_t d);

//prototipos de funciones
void delayMs(int n);
void delayUs(int n);

void LCD_init(void);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_string(char cadena[]);
void LCD_goto(uint8_t row, uint8_t col);
void LCD_2digits(uint8_t n);

void I2C0_init(void);
static int i2c_wait(void);
static int i2c_start(void);
static void i2c_stop(void);
static int  i2c_write(uint8_t data);
int  i2c_write_bytes(uint8_t dev, uint8_t reg, const uint8_t *buf, uint8_t n);
int  i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n);
uint8_t i2c_scan(void);

void SPI0_init(void);
void max7219_write(unsigned char command, unsigned char data);
void max7219_init(void);
void max7219_show_time(const rtc_time_t *t);

uint8_t bcd2bin(uint8_t b);
uint8_t bin2bcd(uint8_t b);
int rtc_set_time(const rtc_time_t *t);
int rtc_get_time(rtc_time_t *t);
void lcd_show_datetime(const rtc_time_t *t);

// main --------------------------------------------------------
int main(void) {

	rtc_time_t now;
	uint8_t status;

	//vamos a usar TPM0 como base del tiempo
	SIM->SCGC6 |= 0x01000000;
	    SIM->SOPT2 |= 0x01000000;
	    TPM0->SC    = 0;
	    TPM0->SC    = 0x02;
	    TPM0->MOD   = 0x2000;
	    TPM0->SC   |= 0x80;
	    TPM0->SC   |= 0x08;

	    //inicializamos todooo
	    LCD_init();
	    I2C0_init();
	    SPI0_init();
	    max7219_init();

	//config de fecha y hora si el RTC llega a perder energía o se fuerza
	if (i2c_read_bytes(DS3231_ADDR, REG_STATUS, &status, 1)!= 0) status = 0x80;

	if ((status & 0x80) || FORCE_SET_TIME){
		rtc_time_t init;
		init.year =INIT_YEAR; init.month = INIT_MONTH; init.day = INIT_DAY;
		init.hour =INIT_HOUR; init.min = INIT_MIN; init.sec = INIT_SEC;
		init.dow = day_of_week(INIT_YEAR, INIT_MONTH, INIT_DAY);
		rtc_set_time(&init);

		//control = 0 (oscilador activo) y status = 0 (limpia la bandera de OSF)
		{
			uint8_t cs[2] = {0x00, 0x00};
			i2c_write_bytes(DS3231_ADDR, REG_CONTROL, cs, 2);
		}
	}

	LCD_command(0x01); delayMs(2);

	//para estar leyendo y mostrando continuamente
	while(1){
		if (rtc_get_time(&now) == 0){
			lcd_show_datetime(&now);
			max7219_show_time(&now);
		}
		delayMs(250);
	}
}

//polling I2C0
void I2C0_init(void){
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;/* reloj I2C0 */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;/* reloj Puerto E */

	/* PTE24 = SCL, PTE25 = SDA (ALT5). Pull-up interna solo como respaldo */
	PORTE->PCR[24] = PORT_PCR_MUX(5) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTE->PCR[25] = PORT_PCR_MUX(5) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	I2C0->F  = I2C_ICR;
	I2C0->C1 = I2C_C1_IICEN_MASK;/* habilita modulo */
}

//Esperamos a que termine una transferencia, 0=ok 1= timeout
static int i2c_wait(void){
	uint32_t t = 100000;
	while (!(I2C0->S & I2C_S_IICIF_MASK)) {
	    if (--t == 0) return 1;
	}
	I2C0->S |= I2C_S_IICIF_MASK;            /* limpia (escribir 1) */
	return 0;
}

static int i2c_start(void)
{
    uint32_t t = 100000;
    while (I2C0->S & I2C_S_BUSY_MASK) {     /* espera bus libre */
        if (--t == 0) return 1;
    }
    I2C0->C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;   /* START + transmitir */
    return 0;
}

static void i2c_stop(void)
{
    I2C0->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK); /* STOP */
}

//enviamos un byte, 0 si hubo acknowledgement y 1 si fue nack/timeout
static int i2c_write(uint8_t data)
{
    I2C0->D = data;
    if (i2c_wait()) return 1;
    return (I2C0->S & I2C_S_RXAK_MASK) ? 1 : 0;
}

//La función escribe n bytes a partir del reg, 0 = ok
int i2c_write_bytes(uint8_t dev, uint8_t reg, const uint8_t *buf, uint8_t n)
{
    uint8_t i;
    int err = 0;

    if (i2c_start()) return 1;
    if (i2c_write((uint8_t)(dev << 1)) || i2c_write(reg)) err = 1;
    for (i = 0; i < n && !err; i++) {
        if (i2c_write(buf[i])) err = 1;
    }
    i2c_stop();
    return err;
}

int i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n){
	uint8_t i;
	volatile uint8_t test;

	if (n==0) return 1;
	if (i2c_start())return 1;

	if (i2c_write((uint8_t)(dev << 1)) || i2c_write(reg)) {
	    i2c_stop();
	    return 1;
		}
	I2C0 ->C1 |= I2C_C1_RSTA_MASK;
	//START repetido
	if (i2c_write((uint8_t)((dev << 1) | 1))) {//direccion Y lectura
		i2c_stop();
	    return 1;
	}
	I2C0->C1 &= ~I2C_C1_TX_MASK;//modo de recepción
	if (n == 1) I2C0->C1 |= I2C_C1_TXAK_MASK;//nack al último byte
	else        I2C0->C1 &= ~I2C_C1_TXAK_MASK; //acknowledge

	test = I2C0->D; //iniciamos el reloj para una lectura de prueba
	(void)test;

	for (i = 0; i < n; i++) {
	    if (i2c_wait()) { i2c_stop(); return 1; }
	    if (i == n - 2) I2C0->C1 |= I2C_C1_TXAK_MASK; /* NACK al ultimo byte */
	    if (i == n - 1) i2c_stop();                 /* STOP antes de leer el ultimo */
	    buf[i] = I2C0->D;
	}
	return 0;
}

//escaneamos desde 0x01 a 0x7E y devuelve la primera dirección que responda (si es 0 es ninguna)
uint8_t i2c_scan(void){
	uint8_t a;
	 for (a = 1; a < 0x7F; a++) {
	    int nack;
	    if (i2c_start()) return 0;
	    nack = i2c_write((uint8_t)(a << 1));
	    i2c_stop();
	    delayUs(2);
	    if (!nack) return a;
	}
	return 0;

}

//SPI0 y MAX7219
void SPI0_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD->PCR[1] = PORT_PCR_MUX(2);   /* PTD1 -> SPI0_SCK  */
    PORTD->PCR[2] = PORT_PCR_MUX(2);   /* PTD2 -> SPI0_MOSI */
    PORTD->PCR[0] = PORT_PCR_MUX(1);   /* PTD0 -> GPIO: lo usamos como CS */

    PTD->PDDR |= (1 << 0);             /* CS como salida */
    PTD->PSOR  = (1 << 0);             /* CS en alto = inactivo */

    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK; /* reloj del modulo SPI0 */

    SPI0->C1  = SPI_C1_MSTR_MASK;      /* modo maestro (CPOL=0, CPHA=0, que es
                                          lo que necesita el MAX7219) */
    SPI0->BR  = 0x60;                  /* divisor de baud rate (~1-1.5 MHz) */
    SPI0->C1 |= SPI_C1_SPE_MASK;       /* habilita el modulo SPI */
}

void max7219_write(unsigned char command, unsigned char data)
{
    volatile char dummy;

    PTD->PCOR = (1 << 0);                          /* CS en bajo: inicia trama */

    while (!(SPI0->S & SPI_S_SPTEF_MASK)) { }      /* espera buffer de TX vacio */
    SPI0->D = command;                             /* byte 1: registro */
    while (!(SPI0->S & SPI_S_SPRF_MASK)) { }       /* espera a que termine el byte */
    dummy = SPI0->D;                               /* SPI recibe mientras envia;
                                                      leer D limpia SPRF */

    while (!(SPI0->S & SPI_S_SPTEF_MASK)) { }
    SPI0->D = data;                                /* byte 2: dato */
    while (!(SPI0->S & SPI_S_SPRF_MASK)) { }
    dummy = SPI0->D;

    PTD->PSOR = (1 << 0);                          /* CS en alto: fin de trama */
}

void max7219_init(void)
{
    /* Decode mode 0x0F: los digitos 0-3 usan "Code B", es decir, escribimos
       el numero 0-9 directamente y el chip enciende los segmentos correctos. */
    max7219_write(DECODE, 0x0F);
    max7219_write(SCANLIMIT, 3);   /* solo escanea 4 digitos (0 a 3) */
    max7219_write(INTENSITY, 4);   /* brillo (0 a 15) */
    max7219_write(TEST, 0);        /* modo de prueba apagado */
    max7219_write(SHUTDOWN, 1);    /* 1 = operacion normal (0 = apagado) */
}

void max7219_show_time(const rtc_time_t *t)
{
    uint8_t dp = (t->sec & 1) ? 0x80 : 0x00;   /* segundos impares -> punto encendido */

    max7219_write(4, t->hour / 10);            /* decenas de hora  */
    max7219_write(3, (t->hour % 10) | dp);     /* unidades de hora + punto */
    max7219_write(2, t->min / 10);             /* decenas de minuto */
    max7219_write(1, t->min % 10);             /* unidades de minuto */
}

//funciones para el DS3231 --------------------------------------

uint8_t bcd2bin(uint8_t b) { return (uint8_t)((b >> 4) * 10 + (b & 0x0F)); }
uint8_t bin2bcd(uint8_t b) { return (uint8_t)(((b / 10) << 4) | (b % 10)); }

uint8_t day_of_week(uint8_t y, uint8_t m, uint8_t d){ //vamos a tomar 1 como domingo y 7 como sábad
	static const uint8_t t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
	uint16_t yy = 2000 + y;
	if (m < 3) yy--;
	return (uint8_t)(((yy + yy / 4 - yy / 100 + yy / 400 + t[m - 1] + d) % 7) + 1);
}

int rtc_set_time(const rtc_time_t*t){
	uint8_t buf[7];
	buf[0] = bin2bcd(t->sec);
	buf[1] = bin2bcd(t->min);
	buf[2] = bin2bcd(t->hour);          /* bit6 = 0 -> formato 24 h */
	buf[3] = bin2bcd(t->dow);
	buf[4] = bin2bcd(t->day);
	buf[5] = bin2bcd(t->month);
	buf[6] = bin2bcd(t->year);
	return i2c_write_bytes(DS3231_ADDR, REG_SECONDS, buf, 7);
}

int rtc_get_time(rtc_time_t *t){
	uint8_t buf[7];
	if (i2c_read_bytes(DS3231_ADDR, REG_SECONDS, buf, 7)) return 1;
	t->sec   = bcd2bin(buf[0] & 0x7F);
	t->min   = bcd2bin(buf[1] & 0x7F);
	t->hour  = bcd2bin(buf[2] & 0x3F);
	t->dow   = bcd2bin(buf[3] & 0x07);
	t->day   = bcd2bin(buf[4] & 0x3F);
	t->month = bcd2bin(buf[5] & 0x1F);  /* bit7 = siglo, se ignora */
	t->year  = bcd2bin(buf[6]);
	return 0;
}

//mostramos los datos con el formato DD/MM/YY
void lcd_show_datetime(const rtc_time_t *t)
{
    LCD_goto(0, 0); LCD_string("Date: ");
    LCD_2digits(t->day);   LCD_data('/');
    LCD_2digits(t->month); LCD_data('/');
    LCD_2digits(t->year);

    LCD_goto(1, 0); LCD_string("Time: ");
    LCD_2digits(t->hour);  LCD_data(':');
    LCD_2digits(t->min);   LCD_data(':');
    LCD_2digits(t->sec);
}

	//funciones de la LCD -------------------------------------------
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

void LCD_string(char cadena[])
{
    int i = 0;
    while (cadena[i] != 0) { LCD_data(cadena[i]); i++; }
}

void LCD_goto(uint8_t row, uint8_t col)
{
    LCD_command((row ? 0xC0 : 0x80) + col);
}

void LCD_2digits(uint8_t n)
{
    LCD_data('0' + (n / 10) % 10);
    LCD_data('0' + (n % 10));
}

void LCD_hex8(uint8_t n)
{
    const char hex[] = "0123456789ABCDEF";
    LCD_data(hex[n >> 4]);
    LCD_data(hex[n & 0x0F]);
}

//nuestros bellos y conocidos delayMs y delayUs
void delayMs(int n)
{
    for (int i = 0; i < n; i++) {
        while ((TPM0->SC & 0x80) == 0) { }
        TPM0->SC |= 0x80;
    }
}

void delayUs(int n)
{
    (void)n;                   /* resolucion minima = 1 tick TPM */
    while ((TPM0->SC & 0x80) == 0) { }
    TPM0->SC |= 0x80;
}
