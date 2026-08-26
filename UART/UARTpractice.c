#include <MKL25Z4.h>
#include <stdio.h>

/* UART functions */
void UART0_init(void);
void UART0_putc(char c);
char UART0_getc(void);
void UART0_puts(const char *str);

/* Menu functions */
void showMenu(void);
void processCommand(char option);

void LED_init(void);
void ADC0_init(void);
void init_Buttons(void);
void keypad_init(void);
char keypad_getkey(void);
char get_key_pressed(void);
void optionLED(void);
void optionADC(void);
void optionKeypad(void);
void optionButtons(void);

void UART0_IRQHandler(void); //handler for UART0 receive interrupt

const char keymap[16] = {
    '1','2','3','A',
    '4','5','6','B',
    '7','8','9','C',
    '*','0','#','D'
};

//flags shared with UART0_IRQHandler
volatile char rxData = 0; //holds the last received byte
volatile int rxReady = 0; //gets set to 1 by the Interrupt Service Routine when a new byte has arrived


void LED_init(void)
{
    /* Enable clock for PORTB and PORTD */
    SIM->SCGC5 |= 0x0400; /* Clock to PORTB */
    SIM->SCGC5 |= 0x1000; /* Clock to PORTD */

    /* Configure pins as GPIO */
    PORTB->PCR[18] = 0x0100; /* PTB18 = Red */
    PORTB->PCR[19] = 0x0100; /* PTB19 = Green */
    PORTD->PCR[1]  = 0x0100; /* PTD1  = Blue */

    /* Configure direction as Output */
    PTB->PDDR |= 0x40000; /* PTB18 output */
    PTB->PDDR |= 0x80000; /* PTB19 output */
    PTD->PDDR |= 0x02;    /* PTD1 output */

    /* Turn OFF  */
    PTB->PSOR = 0x40000;
    PTB->PSOR = 0x80000;
    PTD->PSOR = 0x02;
}

/* -------------------------------------------------
 * UART FUNCTIONS
 * ------------------------------------------------- */

void UART0_init(void)
{
    /* Enable clock for UART0 */
    SIM->SCGC4 |= 0x0400;
    /* Use FLL output for UART baud rate generator */
    SIM->SOPT2 |= 0x04000000;
    /* Turn off UART0 while changing configurations */
    UART0->C2 = 0x00;
    /* 115200 Baud */
    UART0->BDH = 0x00;
    UART0->BDL = 0x17;
    /* Over Sampling Ratio = 16 */
    UART0->C4 = 0x0F;
    /* 8-bit data, no parity */
    UART0->C1 = 0x00;
    /* Enable transmitter AND receiver */
    UART0->C2 = 0x0C;
    /* Enable clock for PORTA */
    SIM->SCGC5 |= 0x0200;
    /* PTA2 = UART0_TX */
    PORTA->PCR[2] = 0x0200;
    /* PTA1 = UART0_RX */
    PORTA->PCR[1] = 0x0200;

    //we enable NVIC (nested vector interrupt controller)interrupt for UART0 so reception is interrupt driven
    //friendly reminder: es el bloque de hardware en los microcontroladores con arquitectura ARM Cortex-M encargado de gestionar, priorizar y atender de forma rápida las interrupciones y excepciones
    NVIC_SetPriority(UART0_IRQn, 2);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

}

void UART0_putc(char c)
{
    /* Wait until transmit data register is empty */
    while (!(UART0->S1 & 0x80))
    {
    }

    UART0->D = c;
}


char UART0_getc(void)
{
    /* Wait for UART0_IRQHandler to signal a received byte */
	while (!rxReady)
	    {
	        __WFI();
	    }
	    rxReady = 0;
	    return rxData;
}


void UART0_puts(const char *str)
{
    while (*str != '\0')
    {
        UART0_putc(*str);
        str++;
    }
}


// UART0 receive interrupt handler: fires when RDRF is set,
//reads the data register (which clears RDRF(receive data register full )) and signals the rest of the program
void UART0_IRQHandler(void)
{
    if (UART0->S1 & 0x20)
    {
        rxData = UART0->D;
        rxReady = 1;
    }
}


/* -------------------------------------------------
 * MENU
 * ------------------------------------------------- */

void showMenu(void)
{
	    UART0_puts("Commands:\r\n");
	    UART0_puts("L-LED control\r\n");
	    UART0_puts("A-Read ADC\r\n");
	    UART0_puts("K-Read keypad\r\n");
	    UART0_puts("B-Button status\r\n");
}


/* -------------------------------------------------
 * MENU OPTIONS
 * ------------------------------------------------- */

void optionLED(void)
{
		char c = '0';

	    UART0_puts("\r\nLED CONTROL\r\n");
	    UART0_puts("1-Red\r\n");
	    UART0_puts("2-Green\r\n");
	    UART0_puts("3-Blue\r\n");
	    UART0_puts("0-OFF\r\n");

	    while (c != 'Q' && c != 'q')
	    {
	        /* Check if a character has been received (flag set by UART0_IRQHandler) */
	        if (rxReady)
	        {
	            c = rxData;
	            rxReady = 0;

	            /* Turn OFF all LEDs first */
	            PTB->PSOR = 0x40000;
	            PTB->PSOR = 0x80000;
	            PTD->PSOR = 0x02;

	            switch (c)
	            {
	                case '1':
	                    PTB->PCOR = 0x40000; /* Red ON */
	                    UART0_puts("Red ON\r\n");
	                    break;

	                case '2':
	                    PTB->PCOR = 0x80000; /* Green ON */
	                    UART0_puts("Green ON\r\n");
	                    break;

	                case '3':
	                    PTD->PCOR = 0x02;    /* Blue ON */
	                    UART0_puts("Blue ON\r\n");
	                    break;

	                case '0':
	                    UART0_puts("OFF\r\n");
	                    break;

	                case 'Q':
	                case 'q':
	                    break;

	                default:
	                    UART0_puts("Invalid command\r\n");
	                    break;
	            }
	        }
	    }

}

}


// Función para inicializar el ADC0
void ADC0_init(void)
{
	SIM->SCGC5 |= 0x2000; /* clock to PORTE */
	PORTE->PCR[20] = 0; /* PTE20 analog input */
	SIM->SCGC6 |= 0x8000000; /* clock to ADC0 */
	ADC0->SC2 &= ~0x40; /* software trigger */
	/* clock div by 4, long sample time, single ended 12 bit, bus clock */
	ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
}

void optionADC(void)
{
	char c = '0';
	int result = 0;
	int Vin = 0;
	while(c != 'Q' && c != 'q'){
		ADC0->SC1[0] = 0; /* start conversion on channel 0 */
		while (!(ADC0->SC1[0] & 0x80)) {
		} /* wait COCO */
		result = ADC0->R[0];
		/* read conversion result and clear COCO flag */
		Vin = ((int)result * 3300) / 4096;
		char buffer[12];
		sprintf(buffer, "%d mV\r\n", Vin);
		UART0_puts(buffer);
		delayMs(300);
		//we check for received character (flag set by UART0_IRQHandler)
		if (rxReady){
				           c = rxData;
				           rxReady = 0;
				}
	}
	return;
}


void optionKeypad(void)
{
    char c = "0";
    char key;
    int code;
    int prevPressed = 0;

    UART0_puts("\r\nPress a key:\r\n");

    while (c != "Q" && c != "q")
    {
    	code = keypad_getkey();
    	//we detect the debounced rising edge of a key press
    	if (code != 0 && prevPressed == 0){
    		delayMs(20); //debounce
    		code = keypad_getkey();
    		if (code != 0){
    			key = keymap[code-1];
    			UART0_puts("Key pressed: ");
    			UART0_putc(key);
    			UART0_puts("\r\n");
    			UART0_puts("Press a key:\r\n");
    		}
    	}
    	prevPressed = (code != 0);

    	//we check for received character (flag de UART0_IRQHandler)
    	if (rxReady){
    		c = rxData;
    		rxReady = 0;
    	}
    }
}

void keypad_init(void)
{
	SIM->SCGC5 |= 0x0800;  /* enable clock to Port C */
	PORTC->PCR[0] = 0x103; /* PTC0, GPIO, enable pullup*/
	PORTC->PCR[1] = 0x103; /* PTC1, GPIO, enable pullup*/
	PORTC->PCR[2] = 0x103; /* PTC2, GPIO, enable pullup*/
	PORTC->PCR[3] = 0x103; /* PTC3, GPIO, enable pullup*/
	PORTC->PCR[4] = 0x103; /* PTC4, GPIO, enable pullup*/
	PORTC->PCR[5] = 0x103; /* PTC5, GPIO, enable pullup*/
	PORTC->PCR[6] = 0x103; /* PTC6, GPIO, enable pullup*/
	PORTC->PCR[7] = 0x103; /* PTC7, GPIO, enable pullup*/
	PTC->PDDR = 0x0F; /* make PTC7-0 as input pins */
}

char keypad_getkey(void)
{
	int row, col;
	const char row_select[] = {0x01, 0x02, 0x04, 0x08};

	PTC->PDDR |= 0x0F; /* enable all rows */
	PTC->PCOR = 0x0F;
	delayUs(2); /* wait for signal to settle */
	col = PTC->PDIR & 0xF0; /* read all columns */
	PTC->PDDR = 0; /* disable all rows */
	if (col == 0xF0)
		return 0; /* no key pressed */
	/* If a key is pressed, find out which one */
	for (row = 0; row < 4; row++)
	{
		PTC->PDDR = 0; /* disable all rows */
		PTC->PDDR |= row_select[row]; /* enable one row */
		PTC->PCOR = row_select[row]; /* drive active row low */

		delayUs(2); /* wait for signal to settle */
		col = PTC->PDIR & 0xF0; /* read all columns */

		if (col != 0xF0) break;
	}

	PTC->PDDR = 0; /* disable all rows */

	if (row == 4)
		return 0; /* no key is pressed */

	/* check which column it is */
	if (col == 0xE0) return row*4 + 1; /* key in column 0 */
	if (col == 0xD0) return row*4 + 2; /* key in column 1 */
	if (col == 0xB0) return row*4 + 3; /* key in column 2 */
	if (col == 0x70) return row*4 + 4; /* key in column 3 */
	return 0;
}

void init_Buttons(void)
{
	SIM->SCGC5 |= 0x2000;
	PORTE->PCR[29] = 0x0100;
	PORTE->PCR[30] = 0x0100;
	PORTE->PCR[29] |= 0x03;
	PORTE->PCR[30] |= 0x03;
	PTE->PDDR &= ~(1 << 29);
	PTE->PDDR &= ~(1 << 30);
}
void optionButtons(void)
{
    char c = '0';
    int prevB1 = 1;   /* PTE29 */
    int prevB2 = 1;   /* PTE30 */
    int curB1, curB2;
    while (c != 'Q' && c != 'q')
    {
        curB1 = (PTE->PDIR >> 29) & 0x01;
        curB2 = (PTE->PDIR >> 30) & 0x01;
        if (curB1 != prevB1 || curB2 != prevB2)
        {
            if (curB1 == 0)
                UART0_puts("Button 1: PRESSED\r\n");
            else
                UART0_puts("Button 1: RELEASED\r\n");
            if (curB2 == 0)
                UART0_puts("Button 2: PRESSED\r\n");
            else
                UART0_puts("Button 2: RELEASED\r\n");
            prevB1 = curB1;
            prevB2 = curB2;
        }
        delayMs(20);
        if (rxReady)
        {
            c = rxData;
            rxReady = 0;
        }
    }
}


/* -------------------------------------------------
 * COMMAND PROCESSING
 * ------------------------------------------------- */

void processCommand(char option)
{
	switch (option)
	    {
	        case 'L':
	        case 'l':
	            optionLED();
	            showMenu();
	            break;

	        case 'A':
	        case 'a':
	            optionADC();
	            showMenu();
	            break;

	        case 'K':
	        case 'k':
	            optionKeypad();
	            showMenu();
	            break;

	        case 'B':
	        case 'b':
	            optionButtons();
	            showMenu();
	            break;

	        default:
	            UART0_puts("\r\nInvalid command\r\n");
	            UART0_puts("select an option:\r\n");
	            break;
	    }
}


/* -------------------------------------------------
 * MAIN
 * ------------------------------------------------- */

int main(void)
{
    char option;

    /* Initialize peripherals */
    	UART0_init();
        ADC0_init();
        init_Buttons();
        LED_init();
        keypad_init();

    /* Show menu */
    showMenu();

    while (1)
    {
        option = UART0_getc();

        processCommand(option);
    }
}
