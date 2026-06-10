#include <xc.h>

#define _XTAL_FREQ 20000000

#pragma config FOSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF
#pragma config MCLRE = ON

// Entrada
#define ENTRADA PORTBbits.RB0

// Salidas
#define BUZZER LATBbits.LATB1
#define LED    LATBbits.LATB2
#define SALIDA LATBbits.LATB3

void main(void) {

    ADCON1 = 0x0F; // Pines digitales

    // Configurar pines
    TRISBbits.TRISB0 = 1; // RB0 entrada

    TRISBbits.TRISB1 = 0; // RB1 buzzer
    TRISBbits.TRISB2 = 0; // RB2 LED
    TRISBbits.TRISB3 = 0; // RB3 salida constante

    // Apagar todo al inicio
    BUZZER = 0;
    LED = 0;
    SALIDA = 0;

    while(1) {

        if(ENTRADA == 1) {

            // Si la entrada recibe 1
            SALIDA = 1; // salida constante en 1

            LED = 0;
            BUZZER = 0;
        }

        else {

            // Si la entrada recibe 0
            SALIDA = 0;

            LED = 1;
            BUZZER = 1;
            __delay_ms(500);

            LED = 0;
            BUZZER = 0;
            __delay_ms(500);
        }
    }
}

