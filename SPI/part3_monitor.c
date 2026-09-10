#include <MKL25Z4.H>

/* Mapeo del teclado 4x4 */
const char keypad_map[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

/* Registros MAX7219 */
#define DECODE    9
#define INTENSITY 10
#define SCANLIMIT 11
#define SHUTDOWN  12
#define TEST      15

/* Definicion de Modos del Sistema */
typedef enum {
    MODE_NORMAL = 0,
    MODE_MIN,
    MODE_MAX,
    MODE_THRESHOLD
} system_mode_t;

/* Variables Globales de Estado */
system_mode_t current_mode = MODE_NORMAL;
uint16_t adc_value = 0;
uint16_t min_value = 0;
uint16_t max_value = 0;
uint16_t threshold_value = 2000;

/* Buffer para captura de datos por Teclado */
char key_buffer[5] = {0};
uint8_t buffer_idx = 0;

/* Base de tiempo global no bloqueante */
volatile uint32_t ms_ticks = 0;

/* Prototipos de Funciones */
void SysTick_Init(void);
void delayUs(uint32_t us);
void SPI0_init(void);
void max7219_write(unsigned char command, unsigned char data);
void display_show_number(uint16_t num);
void LED_init(void);
void set_rgb(uint8_t red, uint8_t green, uint8_t blue);
void update_rgb_led(void);
void ADC0_init(void);
uint16_t ADC0_read(void);
void buttons_init(void);
void keypad_init(void);
char keypad_getkey(void);
void process_buttons(void);
void process_keypad(void);


void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; /* Interrupcion cada 1 ms */
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    ms_ticks++;
}

void delayUs(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 7; i++) {
        __NOP();
    }
}

void SPI0_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    /* SCK en PTC5 */
    PORTC->PCR[5] = PORT_PCR_MUX(2); /* PTC5 -> SPI0_SCK */
    PORTD->PCR[2] = PORT_PCR_MUX(2); /* PTD2 -> SPI0_MOSI */
    PORTD->PCR[0] = PORT_PCR_MUX(1); /* PTD0 -> GPIO (CS) */

    PTD->PDDR |= (1 << 0);
    PTD->PSOR = (1 << 0);            /* CS en alto (inactivo) */

    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;

    SPI0->C1 = SPI_C1_MSTR_MASK;     /* Modo Maestro */
    SPI0->BR = 0x60;                 /* ~1 MHz */
    SPI0->C1 |= SPI_C1_SPE_MASK;     /* Habilita SPI0 */
}

void max7219_write(unsigned char command, unsigned char data) {
    volatile char dummy;
    PTD->PCOR = (1 << 0);            /* CS en bajo */

    while (!(SPI0->S & SPI_S_SPTEF_MASK));
    SPI0->D = command;
    while (!(SPI0->S & SPI_S_SPRF_MASK));
    dummy = SPI0->D;

    while (!(SPI0->S & SPI_S_SPTEF_MASK));
    SPI0->D = data;
    while (!(SPI0->S & SPI_S_SPRF_MASK));
    dummy = SPI0->D;

    PTD->PSOR = (1 << 0);            /* CS en alto */
}

void display_show_number(uint16_t num) {
    if (num > 9999) num = 9999;

    max7219_write(1, num % 10);
    max7219_write(2, (num / 10) % 10);
    max7219_write(3, (num / 100) % 10);
    max7219_write(4, (num / 1000) % 10);
}

void LED_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    PORTB->PCR[18] = PORT_PCR_MUX(1); /* PTB18 = Rojo */
    PORTB->PCR[19] = PORT_PCR_MUX(1); /* PTB19 = Verde */
    PORTD->PCR[1]  = PORT_PCR_MUX(1); /* PTD1  = Azul */

    PTB->PDDR |= (1 << 18) | (1 << 19);
    PTD->PDDR |= (1 << 1);

    set_rgb(0, 0, 0);                 /* Apagado (logica invertida) */
}

void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (red)   PTB->PCOR = (1 << 18); else PTB->PSOR = (1 << 18);
    if (green) PTB->PCOR = (1 << 19); else PTB->PSOR = (1 << 19);
    if (blue)  PTD->PCOR = (1 << 1);  else PTD->PSOR = (1 << 1);
}

void ADC0_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[0] = 0;                /* PTB0 como entrada analogica (ADC0_SE8) */

    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK; /* Disparo por software */
    ADC0->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1); /* 12 bits */
}

uint16_t ADC0_read(void) {
    ADC0->SC1[0] = 8;                 /* Canal ADC0_SE8 (PTB0) */
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

void buttons_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[29] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* PTE29 = PB1 */
    PORTE->PCR[30] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* PTE30 = PB2 */
    PTE->PDDR &= ~((1 << 29) | (1 << 30));
}

void keypad_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    /* PTE2..PTE5 = Filas */
    PORTE->PCR[5] = 0x103;
    PORTE->PCR[4] = 0x103;
    PORTE->PCR[3] = 0x103;
    PORTE->PCR[2] = 0x103;

    /* PTB8..PTB11 = Columnas */
    PORTB->PCR[11] = 0x103;
    PORTB->PCR[10] = 0x103;
    PORTB->PCR[9]  = 0x103;
    PORTB->PCR[8]  = 0x103;

    PTE->PDDR &= ~(0x0F << 2);
    PTB->PDDR &= ~(0x0F << 8);
}

char keypad_getkey(void) {
    int row;
    uint32_t col_val;
    const uint8_t row_pins[4] = {5, 4, 3, 2};

    /* Prueba rapida: si nada esta presionado, salir sin escanear fila por fila */
    PTE->PDDR |= (0x0F << 2);
    PTE->PCOR = (0x0F << 2);
    delayUs(2);

    col_val = (PTB->PDIR >> 8) & 0x0F;
    PTE->PDDR &= ~(0x0F << 2);

    if (col_val == 0x0F) return 0;

    for (row = 0; row < 4; row++) {
        PTE->PDDR &= ~(0x0F << 2);
        PTE->PDDR |= (1 << row_pins[row]);
        PTE->PCOR = (1 << row_pins[row]);

        delayUs(2);
        col_val = (PTB->PDIR >> 8) & 0x0F;

        if (col_val != 0x0F) break;
    }

    PTE->PDDR &= ~(0x0F << 2);

    if (row == 4) return 0;

    if (col_val == 0x07) return row * 4 + 1;
    if (col_val == 0x0B) return row * 4 + 2;
    if (col_val == 0x0D) return row * 4 + 3;
    if (col_val == 0x0E) return row * 4 + 4;

    return 0;
}


void process_buttons(void) {
    static uint8_t prev_pb1 = 1, prev_pb2 = 1;
    uint8_t curr_pb1 = (PTE->PDIR >> 29) & 0x01;
    uint8_t curr_pb2 = (PTE->PDIR >> 30) & 0x01;

    /* PB1: flanco de bajada -> cambia de modo */
    if (prev_pb1 == 1 && curr_pb1 == 0) {
        current_mode = (system_mode_t)((current_mode + 1) % 4);
        buffer_idx = 0; /* Descarta cualquier entrada incompleta del teclado */
    }

    /* PB2: flanco de bajada -> resetea min/max con la lectura actual */
    if (prev_pb2 == 1 && curr_pb2 == 0) {
        min_value = adc_value;
        max_value = adc_value;
    }

    prev_pb1 = curr_pb1;
    prev_pb2 = curr_pb2;
}

void process_keypad(void) {
    static uint8_t last_code = 0;
    uint8_t code = keypad_getkey();

    /* Detecta flanco de tecla nueva (evita repeticiones mientras se mantiene presionada) */
    if (code != 0 && last_code == 0) {
        char key = keypad_map[code - 1];

        if (current_mode == MODE_THRESHOLD) {
            if (key >= '0' && key <= '9' && buffer_idx < 4) {
                key_buffer[buffer_idx++] = key;          /* Acumula digito */
            }
            else if (key == '#' && buffer_idx == 4) {    /* Confirma con exactamente 4 digitos */
                threshold_value = (key_buffer[0] - '0') * 1000 +
                                   (key_buffer[1] - '0') * 100 +
                                   (key_buffer[2] - '0') * 10 +
                                   (key_buffer[3] - '0');
                buffer_idx = 0;
            }
            else if (key == '*') {                       /* Cancela entrada actual */
                buffer_idx = 0;
            }
        }
    }
    last_code = code;
}

void update_rgb_led(void) {
    switch (current_mode) {
        case MODE_NORMAL:
            if (adc_value >= threshold_value) set_rgb(1, 0, 0); /* ROJO: supera umbral */
            else set_rgb(0, 0, 0);                              /* APAGADO: bajo umbral */
            break;

        case MODE_MIN:
            set_rgb(0, 0, 1); /* AZUL: modo Minimo */
            break;

        case MODE_MAX:
            set_rgb(0, 1, 0); /* VERDE: modo Maximo */
            break;

        case MODE_THRESHOLD:
            set_rgb(1, 0, 1); /* MAGENTA: modo Umbral */
            break;
    }
}



int main(void) {
    SysTick_Init();
    SPI0_init();
    LED_init();
    ADC0_init();
    buttons_init();
    keypad_init();

    /* Inicializacion del MAX7219 */
    max7219_write(DECODE, 0x0F);
    max7219_write(SCANLIMIT, 3);
    max7219_write(INTENSITY, 4);
    max7219_write(TEST, 0);
    max7219_write(SHUTDOWN, 1);

    /* Lectura inicial de ADC ANTES del bucle: inicializa min/max con
       la primera medicion, tal como exige el enunciado. */
    adc_value = ADC0_read();
    min_value = adc_value;
    max_value = adc_value;

    uint32_t last_adc_time = ms_ticks;
    uint32_t last_input_time = ms_ticks;

    while (1) {
        /* Tarea 1: lectura del ADC y actualizacion de min/max (cada 100 ms) */
        if ((ms_ticks - last_adc_time) >= 100) {
            last_adc_time = ms_ticks;
            adc_value = ADC0_read();

            if (adc_value < min_value) min_value = adc_value;
            if (adc_value > max_value) max_value = adc_value;
        }

        /* Tarea 2: escaneo de botones y teclado (cada 20 ms) */
        if ((ms_ticks - last_input_time) >= 20) {
            last_input_time = ms_ticks;
            process_buttons();
            process_keypad();
        }

        /* Tarea 3: actualizacion de los displays segun el modo actual */
        switch (current_mode) {
            case MODE_NORMAL:
                display_show_number(adc_value);
                break;
            case MODE_MIN:
                display_show_number(min_value);
                break;
            case MODE_MAX:
                display_show_number(max_value);
                break;
            case MODE_THRESHOLD:
                if (buffer_idx == 0) {
                    display_show_number(threshold_value);
                } else {
                    /* Muestra progresivamente los digitos que se van tecleando */
                    uint16_t temp_val = 0;
                    for (int i = 0; i < buffer_idx; i++) {
                        temp_val = temp_val * 10 + (key_buffer[i] - '0');
                    }
                    display_show_number(temp_val);
                }
                break;
        }

        /* Tarea 4: actualizacion del LED RGB segun el modo/umbral */
        update_rgb_led();
    }
}
