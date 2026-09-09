/*Programming MAX7219 via SPI with FRDM-KL25Z */
/* MAX7219 is connected to a 2-digit 7-segment LED*/
/* The program displays “57” on the 7-segment LED */
/* PTD1 pin as SPI SCK */
/* PTD2 pin as SPI MOSI */
/* PTD0 pin as chip select */

#include <MKL25Z4.H>
void SPI0_init(void);
void SPI0_write(unsigned char data);
void max7219_write(unsigned char command, unsigned char data);
#define DECODE 9
#define INTENSITY 10
#define SCANLIMIT 11
#define SHUTDOWN 12
#define TEST 15

int main(void) {

	SPI0_init(); /* enable SPI0 */

	max7219_write(DECODE, 0x0F);
	/* enable decode for digit 1, 0 */

	max7219_write(SCANLIMIT, 3); /* scan two digits */
	max7219_write(INTENSITY, 4); /* set 1/4 intensity */
	max7219_write(TEST, 0); /* disable test mode */
	max7219_write(SHUTDOWN, 1); /* enable device */

	while(1) {
		max7219_write(0x01, 1); /* display 1 */
		max7219_write(0x02, 2); /* display 2 */
		max7219_write(0x03, 3); /* display 3 */
		max7219_write(0x04, 3); /* display 4 */
	}
}

void SPI0_init(void) {
	SIM->SCGC5 |= 0x1000; /* enable clock to Port D */

	PORTD->PCR[1] = 0x200; /* make PTD1 pin as SPI SCK */
	PORTD->PCR[2] = 0x200; /* make PTD2 pin as SPI MOSI */
	PORTD->PCR[0] = 0x100; /* make PTD0 pin as GPIO */
	PTD->PDDR |= 0x01; /* make PTD0 as output pin for CS */
	PTD->PSOR = 0x01; /* make PTD0 idle high */

	SIM->SCGC4 |= 0x400000; /* enable clock to SPI0 */

	SPI0->C1 = 0x10; /* disable SPI and make SPI0 master */
	SPI0->BR = 0x60; /* set Baud rate to 1 MHz */
	SPI0->C1 |= 0x40; /* Enable SPI module */
}
void max7219_write(unsigned char command, unsigned char data){
	volatile char dummy;
	PTD->PCOR = 1; /* assert /CS */
	while(!(SPI0->S & 0x20)) { } /* wait until tx ready */
	SPI0->D = command; /* send command byte first */
	while(!(SPI0->S & 0x80)) { } /* wait tx complete */
	dummy = SPI0->D; /* clear SPRF */
	while(!(SPI0->S & 0x20)) { } /* wait tx ready */
	SPI0->D = data; /* send data byte */
	while(!(SPI0->S & 0x80)) { } /* wait tx complete */
	dummy = SPI0-> D; /* clear SPRF */
	PTD->PSOR = 1; /* de-assert /CS */
}
