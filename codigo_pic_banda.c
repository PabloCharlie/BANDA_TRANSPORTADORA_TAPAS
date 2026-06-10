// BITS DE CONFIGURACION

#pragma config PLLDIV   = 5         // PLL divide 20MHz/5=4MHz (para USB, no usado)
#pragma config CPUDIV   = OSC1_PLL2 // CPU clock sin division adicional
#pragma config USBDIV   = 2
#pragma config FOSC     = HSPLL_HS  // Cristal HS + PLL -> 20 MHz efectivos
                                    // Sin PLL usar FOSC = HS por <default>
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = ON        // Power-up timer activado
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = OFF
#pragma config WDT      = OFF       // Watchdog desactivado <para evitar la creación de un loop fisico>
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF       // PORTB como digital al reset
#pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config ICPRT    = OFF
#pragma config XINST    = OFF       // Obligatorio OFF para XC8
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CP2      = OFF
#pragma config CP3      = OFF
#pragma config CPB      = OFF
#pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRT2     = OFF
#pragma config WRT3     = OFF
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
#pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTR2    = OFF
#pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF


#include <xc.h> // Comando para activar los componentes internos de la PIC
#include <stdint.h> // Importación de la libreria C

#define _XTAL_FREQ      20000000UL  // Crital 20 MHz 

 //TIMER0: genera interrupciones cada 1 ms (tick del sistema)
 //PRELOAD = 65536 - (20000000 / 4 / 8 / 1000) = 64911

#define TIMER0_PRELOAD  64911U // Contador que inicia en 1 milisegundo

// PARAMETROS PWM DE SERVOMOTORES <PWM para que los servomotores sepan en que posicion quedarse>
// I se me mueve a 0 grados
// 0 se me mueve a 45 grados

#define PULSO_0_GRADOS      500U    // microsegundos para 0°
#define PULSO_45_GRADOS     1000U   // microsegundos para 45°
#define PERIODO_SERVO_US    20000U  // 20 ms = periodo completo PWM

// PARAMETROS DE LA SECUENCIA

#define TIEMPO_EN_0_MS      17000U  // Cuanto tiempo permanece en 0° (ms)
#define PASOS_TRANSICION    40U     // Cuantos pasos tarda el movimiento suave
#define DEBOUNCE_MS         50U     // Tiempo de antirrebote para sensores


// PINES <asignar puertos>
#define SERVO1_PIN      LATDbits.LATD0
#define SERVO2_PIN      LATDbits.LATD1
#define SENSOR1         PORTBbits.RB0
#define SENSOR2         PORTBbits.RB1

// PINES <asignar puertos para LEDS independientes>
#define LED1_PIN        LATCbits.LATC2  // LED para Actuador 1 en RC2
#define LED2_PIN        LATCbits.LATC6  // LED para Actuador 2 en RC6

// MAQUINA DE ESTADOS: estados posibles de cada servo
typedef enum {
    SERVO_IDLE = 0,  // Reposo en 45°, esperando sensor
    SERVO_BAJANDO,   // Moviendose de 45° a 0°
    SERVO_EN_CERO,   // Manteniendo 0° durante TIEMPO_EN_0_MS
    SERVO_SUBIENDO   // Regresando de 0° a 45°
} ServoEstado;

// ESTRUCTURA DE CONTROL DE CADA SERVO

typedef struct {
    ServoEstado estado;         // Estado actual en la maquina de estados
    uint16_t    pulso_us;       // Ancho de pulso actual en microsegundos
    uint16_t    contador_ms;    // Contador de tiempo en el estado actual
    uint8_t     paso_actual;    // Paso actual en la transicion
} ServoControl;

// Variables Globales
volatile uint32_t tick_ms = 0;          // Contador global de milisegundos
volatile ServoControl servo1;           // Estado del Servo 1
volatile ServoControl servo2;           // Estado del Servo 2

//Variables de antirrebote para sensores
volatile uint8_t debounce1_contador = 0; 
volatile uint8_t debounce2_contador = 0; 
volatile uint8_t sensor1_confirmado = 0; 
volatile uint8_t sensor2_confirmado = 0; 
volatile uint8_t sensor1_prev = 1;       
volatile uint8_t sensor2_prev = 1;       

// PROTOTIPOS
void inicializar_sistema(void); // Configura qué pines son entradas para los sensores...
//...cuáles son salidas para los motores, asegura que los LEDs inicien apagados
void inicializar_timer0(void); // Reloj interno
void actualizar_servo(volatile ServoControl *srv, uint8_t sensor_detectado); // Decide el paso del servomotor
uint16_t interpolar(uint16_t desde, uint16_t hasta, uint8_t paso, uint8_t total); // Avisa si el sensor infrarrojo...
//...correspondiente acaba de confirmar el tamaño de la tapa
void generar_pulso_us(uint8_t canal, uint16_t pulso_us);

//RUTINA DE INTERRUPCION DEL TIMER0 (ISR)
void __interrupt() isr(void)
{
    if(INTCONbits.TMR0IF)   
    {
        TMR0H = (TIMER0_PRELOAD >> 8) & 0xFF;
        TMR0L = TIMER0_PRELOAD & 0xFF;
        INTCONbits.TMR0IF = 0;  

        tick_ms++;

        /* ---- Antirrebote Sensor 1 ---- */
        if(SENSOR1 == 0) {
            if(debounce1_contador < DEBOUNCE_MS) {
                debounce1_contador++;
                if(debounce1_contador == DEBOUNCE_MS) {
                    if(sensor1_prev == 1) {
                        sensor1_confirmado = 1;
                    }
                    sensor1_prev = 0;
                }
            }
        } else {
            debounce1_contador = 0;
            sensor1_prev = 1;
        }

        /* ---- Antirrebote Sensor 2 ---- */
        if(SENSOR2 == 0) {
            if(debounce2_contador < DEBOUNCE_MS) {
                debounce2_contador++;
                if(debounce2_contador == DEBOUNCE_MS) {
                    if(sensor2_prev == 1) {
                        sensor2_confirmado = 1;
                    }
                    sensor2_prev = 0;
                }
            }
        } else {
            debounce2_contador = 0;
            sensor2_prev = 1;
        }

        //Avance de maquina de estados de ambos servos
        actualizar_servo(&servo1, sensor1_confirmado);
        if(sensor1_confirmado) sensor1_confirmado = 0; 

        actualizar_servo(&servo2, sensor2_confirmado);
        if(sensor2_confirmado) sensor2_confirmado = 0; 
    }
}

//FUNCION: interpolar
uint16_t interpolar(uint16_t desde, uint16_t hasta, uint8_t paso, uint8_t total)
{
    int32_t delta = (int32_t)hasta - (int32_t)desde;
    int32_t resultado = (int32_t)desde + (delta * (int32_t)paso) / (int32_t)total;
    return (uint16_t)resultado;
}

//FUNCION: actualizar_servo
void actualizar_servo(volatile ServoControl *srv, uint8_t sensor_detectado)
{
    switch(srv->estado)
    {
        case SERVO_IDLE:
            srv->pulso_us = PULSO_0_GRADOS;
            if(sensor_detectado) {
                srv->estado      = SERVO_BAJANDO;
                srv->paso_actual = 0;
                srv->contador_ms = 0;
            }
            break;

        case SERVO_BAJANDO:
            srv->contador_ms++;
            if(srv->contador_ms >= 10U) {  
                srv->contador_ms = 0;
                srv->paso_actual++;
                if(srv->paso_actual >= PASOS_TRANSICION) {
                    srv->pulso_us    = PULSO_45_GRADOS;
                    srv->paso_actual = 0;
                    srv->contador_ms = 0;
                    srv->estado      = SERVO_EN_CERO;
                } else {
                    srv->pulso_us = interpolar(
                        PULSO_0_GRADOS,
                        PULSO_45_GRADOS,
                        srv->paso_actual,
                        PASOS_TRANSICION
                    );
                }
            }
            break;

        case SERVO_EN_CERO:
            srv->pulso_us = PULSO_45_GRADOS;
            srv->contador_ms++;
            if(srv->contador_ms >= TIEMPO_EN_0_MS) {
                srv->paso_actual = 0;
                srv->contador_ms = 0;
                srv->estado      = SERVO_SUBIENDO;
            }
            break;

        case SERVO_SUBIENDO:
            srv->contador_ms++;
            if(srv->contador_ms >= 10U) {
                srv->contador_ms = 0;
                srv->paso_actual++;
                if(srv->paso_actual >= PASOS_TRANSICION) {
                    srv->pulso_us    = PULSO_0_GRADOS;
                    srv->paso_actual = 0;
                    srv->contador_ms = 0;
                    srv->estado      = SERVO_IDLE;
                } else {
                    srv->pulso_us = interpolar(
                        PULSO_45_GRADOS,
                        PULSO_0_GRADOS,
                        srv->paso_actual,
                        PASOS_TRANSICION
                    );
                }
            }
            break;

        default:
            srv->estado      = SERVO_IDLE;
            srv->pulso_us    = PULSO_0_GRADOS;
            srv->paso_actual = 0;
            srv->contador_ms = 0;
            break;
    }
}

//FUNCION: generar_pulso_us
void generar_pulso_us(uint8_t canal, uint16_t pulso_us)
{
    uint16_t i;
    if(canal == 1) {
        SERVO1_PIN = 1;
        for(i = 0; i < pulso_us; i++) { __delay_us(1); }
        SERVO1_PIN = 0;
    } else {
        SERVO2_PIN = 1;
        for(i = 0; i < pulso_us; i++) { __delay_us(1); }
        SERVO2_PIN = 0;
    }
}

// FUNCION: inicializar_timer0
void inicializar_timer0(void)
{
    T0CON = 0b00000010;  
    TMR0H = (TIMER0_PRELOAD >> 8) & 0xFF;
    TMR0L = TIMER0_PRELOAD & 0xFF;
    INTCONbits.TMR0IF = 0;  
    INTCONbits.TMR0IE = 1;  
    T0CONbits.TMR0ON  = 1;  
}

// FUNCION: inicializar_sistema
void inicializar_sistema(void)
{
    ADCON1 = 0x0F;
    ADCON0 = 0x00;

    // PORTB: entradas (sensores IR) con pull-ups
    TRISB = 0xFF;
    INTCON2bits.RBPU = 0;  

    // PORTD: salidas para señales PWM de los servos
    TRISD = 0x00;
    LATD  = 0x00;

    // PORTC: Configuramos RC2 y RC6 como salidas digitales para los LEDs
    // 0xBA = 1011 1010 (Bit 2 y Bit 6 en '0' son salidas)
    TRISC = 0xBA;
    LED1_PIN = 0; // Inicializar LEDs apagados
    LED2_PIN = 0;

    TRISA = 0xFF;
    TRISE = 0x07;
    CMCON = 0x07;

    servo1.estado      = SERVO_IDLE;
    servo1.pulso_us    = PULSO_0_GRADOS;
    servo1.contador_ms = 0;
    servo1.paso_actual = 0;

    servo2.estado      = SERVO_IDLE;
    servo2.pulso_us    = PULSO_0_GRADOS;
    servo2.contador_ms = 0;
    servo2.paso_actual = 0;

    inicializar_timer0();

    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;

    __delay_ms(100);  
}


// FUNCION PRINCIPAL
void main(void)
{
    inicializar_sistema();

    uint16_t pulso_s1, pulso_s2;
    uint16_t tiempo_muerto;
    uint16_t i;

    // Enviar posicion inicial durante 1 segundo antes de arrancar
    uint8_t k;
    for(k = 0; k < 50; k++) {
        generar_pulso_us(1, PULSO_0_GRADOS);
        generar_pulso_us(2, PULSO_0_GRADOS);
        uint16_t usado = PULSO_0_GRADOS + PULSO_0_GRADOS;
        uint16_t resto = PERIODO_SERVO_US - usado;
        for(i = 0; i < resto; i++) { __delay_us(1); }
    }

    // Bucle principal
    while(1)
    {
        // Leer los pulsos actuales de forma segura
        INTCONbits.GIE = 0;
        pulso_s1 = servo1.pulso_us;
        pulso_s2 = servo2.pulso_us;
        
        // CONTROL LOGICO DE ENCENDIDO PARA LED 1
        // Si el servo 1 NO está en reposo (IDLE), significa que está actuando. Enciende el LED.
        if(servo1.estado != SERVO_IDLE) {
            LED1_PIN = 1;
        } else {
            LED1_PIN = 0;
        }

        // CONTROL LOGICO DE ENCENDIDO PARA LED 2
        // Si el servo 2 NO está en reposo (IDLE), enciende el LED.
        if(servo2.estado != SERVO_IDLE) {
            LED2_PIN = 1;
        } else {
            LED2_PIN = 0;
        }
        INTCONbits.GIE = 1;

        // Generar pulso Servo 1
        generar_pulso_us(1, pulso_s1);

        // Generar pulso Servo 2
        generar_pulso_us(2, pulso_s2);

        // Completar el periodo de 20 ms con tiempo en bajo
        tiempo_muerto = PERIODO_SERVO_US - pulso_s1 - pulso_s2;
        for(i = 0; i < tiempo_muerto; i++) { __delay_us(1); }
    }
}
