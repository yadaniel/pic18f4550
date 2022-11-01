/*
 * File:   main.c
 */

#include <xc.h>

// unknown configuration setting
//#pragma config OSC = HSPLL

// CONFIG1L
//#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config PLLDIV = 5         // 20MHz / 5 => 4MHz
//#pragma config CPUDIV = OSC4_PLL6       // PLL6 => divided by 6
//#pragma config CPUDIV = OSC3_PLL4     // PLL4 => divided by 4 ... faster
//#pragma config CPUDIV = OSC2_PLL3     // PLL3 => divided by 3 ... even faster
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
//#pragma config FOSC = XT
//#pragma config FOSC = HS
//#pragma config FOSC = XTPLL_XT
#pragma config FOSC = HSPLL_HS
//#pragma config FOSC = INTOSCIO_EC
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
//#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config LVP = OFF         // must be off, otherwise PB5 not working
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// PORTD
#define LED1 LATDbits.LATD0
#define LED2 LATDbits.LATD1
#define LED3 LATDbits.LATD2
#define LED4 LATDbits.LATD3
#define TRIS_PORTD 0xF0u   /* MSB 1111.0000 LSB */

// PORTB
#define BT2 PORTBbits.RB4
#define BT1 PORTBbits.RB5
#define BT4 PORTBbits.RB6
#define BT3 PORTBbits.RB7
#define TRIS_PORTB 0xCFu   /* MSB 0011.1111 LSB */

void initUART(void) {
    TXSTAbits.TX9 = 0;      // 8 bit mode
    TXSTAbits.TX9D = 0;     // used in 9 bit mode only
    TXSTAbits.SYNC = 0;     // asynchron rx/tx
    TXSTAbits.CSRC = 0;     // don't care when asynchron
    TXSTAbits.TXEN = 1;     // enable transmit
    TXSTAbits.SENDB = 0;    // sync break transmission completed => BREAK transmitter holds line '0' for longer than required for bit
    TXSTAbits.BRGH = 1;     // high speed
    if(TXSTAbits.TRMT == 0) {}  // readonly, trasmitt shift register empty
    
    BAUDCONbits.BRG16 = 0;  // 8 bit baudrate generator
    SPBRGH = 0;             // ignore when BRG16 = 0
    SPBRG = 25;             
    // (defn baud [fosc, n] (/ fosc (* 16 (+ n 1))))
    // (defn baud115200[n] (/ n 115200))
    // (baud 48e6 25) = 115384.61538461539
    // (baud115200 (baud 48e6 25)) => 1.001602564102564
    
    TRISCbits.TRISC6 = 0;   // TX output
    TRISCbits.TRISC7 = 1;   // RX input
    
    ADCON1 = 0x0E;          // setup RA1,RA2,RA3 as digital inputs, RA0 as analog
    TRISAbits.TRISA2 = 0;   // CTS output
    TRISAbits.TRISA3 = 1;   // RTS input
    
    
    RCSTAbits.RC9 = 0;
    // if (RCSTAbits.FERR == 1) {}  // readony, frame error
    // if (RCSTAbits.OERR == 1) {}  // readonly, overrun
    RCSTAbits.SPEN = 1;
}

uint8_t sendUART(uint8_t c) {
    uint8_t ret = 0;
    if(TXSTAbits.TRMT == 1) {
        // transmit register empty
        TXREG = c;
        ret = 1;
    }
    return ret;
}

void initADC(void) {
    ADCON2bits.ADCS = 3;    // FOSC/32
    ADCON2bits.ACQT = 7;    // 20 Tad
    ADCON2bits.ADFM = 1;    // right adjusted
    ADCON1bits.VCFG0 = 0;   // VREF+ derived from VDD (RA3 is digital)
    ADCON1bits.VCFG1 = 0;   // VREF- derived from VSS (RA2 is digital)
    ADCON0bits.CHS = 0x00;  // A0 only
    ADCON0bits.ADON = 1;
}

void main(void) {
    uint16_t cnt = 0;
    uint8_t n = 0;
    uint8_t k = 0;
    uint16_t A0 = 0;
    
    GIE = 0;
    TRISB = TRIS_PORTB;
    TRISD = TRIS_PORTD;
    initUART();
    initADC();
    ADCON0bits.GO_nDONE = 1;    // start sample
    
    while(1) {    
        cnt += 1;
        if(cnt >= 10000) {
            cnt = 0;
            
            n += 1;
            if(PORTAbits.RA3 == 1) {
                LATAbits.LATA2 = 0;     // send only when RTS = 1, CTS enabled
                //sendUART(n);
                sendUART((A0 >> 2) & 0xFF);
            } else {
                LATAbits.LATA2 = 1;     // otherwise CTS = 1, disabled
            }
        }
        
        //if(ADCON0bits.GO_nDONE == 0) {
        if(PIR1bits.ADIF == 1) {
            A0 = ADRESH;
            A0 <<= 8;
            A0 |= ADRESL;
            PIR1bits.ADIF  = 0;         // must be cleared in software
            ADCON0bits.GO_nDONE = 1;    // start sample
        }
        //A0 = 0x0100;  // test
        
        //if((cnt % 1000) == 0) {
        //    sendUART(n);
        //}
        
        /*
        for(uint32_t i=0x0000; i<0x1FFF; i++);
        LED1 = 1;
        for(uint32_t i=0x0000; i<0x1FFF; i++);
        LED1 = 0;
     
        if(BT1 == 0) {
            LED2 = 1;
        }
        
        if(BT2 == 0) {
            LED3 = 1;
        }
        
        if(BT3 == 0) {
            LED4 = 1;
        }
        
        if(BT4 == 0) {
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
        }
        */
        
    }
    return;
}
