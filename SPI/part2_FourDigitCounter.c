/*Programming MAX7219 via SPI with FRDM-KL25Z */
/* MAX7219 is connected to a 4-digit 7-segment LED*/
/* PTD1 pin as SPI SCK */
/* PTD2 pin as SPI MOSI */
/* PTD0 pin as chip select */

#include <MKL25Z4.H>
void SPI0_init(void);
void SPI0_write(unsigned char data);
void max7219_write(unsigned char command, unsigned char data);
void delayMs(int n);
void debounceDelay(void);
void contador_ascendente(void);
void contador_descendente(void);
void init_Buttons(void);
void optionButtons(void);
void refrescar_display(void);

int cont_disp_1 = 0, cont_disp_2 = 0, cont_disp_3 = 0, cont_disp_4 = 0;
int running = 0;
int modo_ascendente = 1;
volatile unsigned int ms_tick = 0;

#define DECODE 9
#define INTENSITY 10
#define SCANLIMIT 11
#define SHUTDOWN 12
#define TEST 15

int main(void) {

	SPI0_init(); /* enable SPI0 */

	max7219_write(DECODE, 0x0F);
	/* enable decode for digit 1, 0 */

	max7219_write(SCANLIMIT, 3); /* scan four digits */
	max7219_write(INTENSITY, 4); /* set 1/4 intensity */
	max7219_write(TEST, 0); /* disable test mode */
	max7219_write(SHUTDOWN, 1); /* enable device */
	refrescar_display();
	init_Buttons();

	SysTick->LOAD = 41940 - 1;
	SysTick->CTRL = 0x5;

	while(1) {
		optionButtons();
		if (running) {
			if (SysTick->CTRL & 0x10000) {
				ms_tick++;
				if (ms_tick >= 300) {
					ms_tick = 0;
					if (modo_ascendente) {
						contador_ascendente();
					}
					else {
						contador_descendente();
					}
				}
			}
		}
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
void delayMs(int n) {
	int i;
	SysTick->LOAD = 41940 - 1;
	SysTick->CTRL = 0x5;

	for (i = 0; i < n; i++) {
		while ((SysTick->CTRL & 0x10000) == 0)
		{
		}
	}
	SysTick->CTRL = 0;
}

void debounceDelay(void){
	volatile int i;
	for (i = 0; i < 30000; i++){
	}
}

void refrescar_display(void){
	max7219_write(0x01, cont_disp_1);
	max7219_write(0x02, cont_disp_2);
	max7219_write(0x03, cont_disp_3);
	max7219_write(0x04, cont_disp_4);
}

void contador_ascendente(void){
	cont_disp_1 ++;
	if (cont_disp_1 > 9){
		cont_disp_1 = 0;
		cont_disp_2 ++;
		if(cont_disp_2 > 9){
			cont_disp_2 = 0;
			cont_disp_3 ++;
			if (cont_disp_3 > 9){
				cont_disp_3 = 0;
				cont_disp_4 ++;
				if (cont_disp_4 > 9){
					cont_disp_4 = 0;
				}
				max7219_write(0x04, cont_disp_4); /* display 4 */
			}
			max7219_write(0x03, cont_disp_3); /* display 3 */
		}
		max7219_write(0x02, cont_disp_2); /* display 2 */
	}
	max7219_write(0x01, cont_disp_1); /* display 1 */
}
void contador_descendente(void){
	cont_disp_1 --;
	if (cont_disp_1 < 0){
		cont_disp_1 = 9;
		cont_disp_2 --;
		if(cont_disp_2 < 0){
			cont_disp_2 = 9;
			cont_disp_3 --;
			if (cont_disp_3 < 0){
				cont_disp_3 = 9;
				cont_disp_4 --;
				if (cont_disp_4 < 0){
					cont_disp_4 = 9;
				}
				max7219_write(0x04, cont_disp_4); /* display 4 */
			}
			max7219_write(0x03, cont_disp_3); /* display 3 */
		}
		max7219_write(0x02, cont_disp_2); /* display 2 */
	}
	max7219_write(0x01, cont_disp_1); /* display 1 */
}

void init_Buttons(void)
{
	SIM->SCGC5 |= 0x2000;
	PORTE->PCR[29] = 0x0100;
	PORTE->PCR[30] = 0x0100;
	PORTE->PCR[23] = 0x0100;
	PORTE->PCR[22] = 0x0100;
	PORTE->PCR[29] |= 0x03;
	PORTE->PCR[30] |= 0x03;
	PORTE->PCR[23] |= 0x03;
	PORTE->PCR[22] |= 0x03;
	PTE->PDDR &= ~(1 << 29);
	PTE->PDDR &= ~(1 << 30);
	PTE->PDDR &= ~(1 << 23);
	PTE->PDDR &= ~(1 << 22);
}

void optionButtons(void)
{
    static int prevB1 = 1;   /* PTE29 - Play/Pause */
    static int prevB2 = 1;   /* PTE30 - Modo asc/desc */
    static int prevB3 = 1;   /* PTE23 - Paso manual */
    static int prevB4 = 1;   /* PTE22 - Reset */
    int curB1, curB2, curB3, curB4;

    curB1 = (PTE->PDIR >> 29) & 0x01;
    curB2 = (PTE->PDIR >> 30) & 0x01;
    curB3 = (PTE->PDIR >> 23) & 0x01;
    curB4 = (PTE->PDIR >> 22) & 0x01;

    if (curB1 == 0 && prevB1 == 1){
    	running = !running;
    	debounceDelay();
    }

    if (curB2 == 0 && prevB2 == 1){
    	modo_ascendente = !modo_ascendente;
    	debounceDelay();
    }

    if (curB3 == 0 && prevB3 == 1){
    	if (!running){
    		if (modo_ascendente){
    			contador_ascendente();
    		}
    		else{
    			contador_descendente();
    		}
    	}
    	debounceDelay();
    }

    if (curB4 == 0 && prevB4 == 1){
    	if (modo_ascendente){
    		cont_disp_1 = 0;
    		cont_disp_2 = 0;
    		cont_disp_3 = 0;
    		cont_disp_4 = 0;
    	}
    	else{
    		cont_disp_1 = 9;
    		cont_disp_2 = 9;
    		cont_disp_3 = 9;
    		cont_disp_4 = 9;
    	}
    	refrescar_display();
    	debounceDelay();
    }

    prevB1 = curB1;
    prevB2 = curB2;
    prevB3 = curB3;
    prevB4 = curB4;
}
