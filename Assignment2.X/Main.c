#define _XTAL_FREQ   4000000UL  // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#define STARTVALUE 40536
#define SUMMER_T   60
#define WINTER_T   40

#ifndef XC_LCD_X8_H
#define _XTAL_FREQ   4000000UL 
#define LCD_TYPE 2
#define LCD_LINE_TWO 0x40
#define LCD_LINE_SIZE 16
#define lcd_output_enable(x) PORTEbits.RE1 = x
#define lcd_output_rs(x) PORTEbits.RE2 = x
struct lcd_pin_map {
    unsigned un1    : 1;
    unsigned rs     : 1;
    unsigned rw     : 1;
    unsigned enable : 1;
    unsigned data   : 4;
} lcd __at(0xF83);
#endif



void restartTimer3(void);
void init_adc_no_lib(void);
float read_adc_voltage(unsigned char channel);
int read_adc_raw_no_lib(unsigned char channel);
void delay_cycles(unsigned char n);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);

unsigned short HS = 0;

char Buffer[32];
float SP, OT, RT, coolError, heatError;
unsigned short operationMode = 0;
unsigned short autoMode = 0;
unsigned short percentHeatCounter = 0;
unsigned short percentCoolCounter = 0;
unsigned short timerCounter = 0;
unsigned short RPS_count = 0;


unsigned short seconds = 0;
unsigned short minutes = 0;
unsigned short hours   = 0;
unsigned short clkMode = 0;
unsigned short stpMode = 0;
unsigned short initial = 1;
unsigned char LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};

void restartTimer3(void) {
    TMR1H = 0;
    TMR1L = 0;
    TMR3H = (unsigned char)((STARTVALUE >>  8) & 0x00FF);
    TMR3L = (unsigned char)(STARTVALUE & 0x00FF );   
}

void delay_ms(unsigned int n) {
    int i;
    for (i=0; i < n; i++) __delaywdt_ms(1) ; 
}

void init_adc_no_lib(void) {
    ADCON0 = 0;
    ADCON0bits.ADON = 1;
    ADCON2 = 0b10001001;
}

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value;
    ADCON0bits.CHS = channel;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO) {};
    raw_value = ADRESH << 8 | ADRESL;
    return raw_value;
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
}

void delay_cycles(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) CLRWDT();
}

void lcd_send_nibble(unsigned char n) {
    lcd.data = n;
    delay_cycles(1);
    lcd_output_enable(1);
    __delaywdt_us(2);
    lcd_output_enable(0);
}

void lcd_send_byte(unsigned char cm_data, unsigned char n) {
    lcd_output_rs(cm_data);
    delay_cycles(1);
    delay_cycles(1);
    lcd_output_enable(0);
    lcd_send_nibble(n >> 4);
    lcd_send_nibble(n & 0x0f);
    if (cm_data) __delaywdt_us(200);
    else delay_ms(2);
}

void lcd_init(void) {
    unsigned char i;
    lcd_output_rs(0);
    lcd_output_enable(0);

    delay_ms(25);   
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        delay_ms(6);
    }

    lcd_send_nibble(2);
    for (i = 0; i <= 3; ++i) lcd_send_byte(0, LCD_INIT_STRING[i]);
}

void lcd_gotoxy(unsigned char x, unsigned char y) {
    unsigned char address;

    switch (y) {
        case 1: address = 0x80;
            break;
        case 2: address = 0xc0;
            break;
        case 3: address = 0x80 + LCD_LINE_SIZE;
            break;
        case 4: address = 0xc0 + LCD_LINE_SIZE;
            break;
    }
    address += x - 1;
    lcd_send_byte(0, (unsigned char) (0x80 | address));
}

void lcd_putc(char c) {
    switch (c) {
        case '\f': lcd_send_byte(0, 1);
            delay_ms(2);
            break;
        case '\n': lcd_gotoxy(1, 2);
            break;
        case '\b': lcd_send_byte(0, 0x10);
            break;
        default: lcd_send_byte(1, c);
            break;
    }
}

void lcd_puts(char *s) {
    while (*s) {
        lcd_putc(*s);
        s++;
    }
}

void changeOperationMode(){
    if (operationMode++ == 2) operationMode = 0;
    timerCounter = 0;
}

void init_pwm1(void) {
    PR2 = 255;
    T2CON = 0;
    CCP1CON = 0x0C;
    T2CONbits.TMR2ON = 1;
    TRISCbits.RC2 =0;
}

void set_pwm1_raw(unsigned int raw_value) {
    CCPR1L = (raw_value >> 2) & 0x00FF;
    CCP1CONbits.DC1B = raw_value & 0x0003;
}

void set_pwm1_percent(float value) {
    float tmp = value*1023.0/100.0;
    int raw_val = (int)(tmp +0.5);
    if ( raw_val> 1023) raw_val = 1023;
    set_pwm1_raw(raw_val);
}

void autoCool(void) {
    coolError = RT - SP;
    percentHeatCounter = 0;
    if (coolError > 0) {
        
        float value = coolError * 10;
        set_pwm1_percent(value < 25 ? 25 : value);
        lcd_gotoxy(11, 3);
        sprintf(Buffer, "R:%4.2f", RPS_count * 40 / 7.0);
        lcd_puts(Buffer);
    }
    if (RT < (SP - HS)) percentHeatCounter = 5;//turn off cooler
}

void autoHeat(void) {
    set_pwm1_percent(0);

    heatError = SP - RT;

    if (heatError > 0) {
        percentHeatCounter = 2*heatError;
        if (percentHeatCounter>20)percentHeatCounter=20; 
        if (percentHeatCounter < 10)percentHeatCounter = 10;
        if ( SP > 52)percentHeatCounter = 10;
    } else if (RT > (SP + HS)) {
        percentHeatCounter = 0;
    }

    lcd_gotoxy(11, 3);
    lcd_putc('H');

    lcd_gotoxy(15, 3);
    lcd_puts("  ");

    lcd_gotoxy(13, 3);
    sprintf(Buffer, "%d", percentHeatCounter * 10);
    lcd_puts(Buffer);
}

void incrementPercentHeatCounter(){
    if (percentHeatCounter < 20) percentHeatCounter++;}


void decrementPercentHeatCounter(){
    if (percentHeatCounter > 0) percentHeatCounter--;}

void incrementPercentCoolCounter(){
    if (percentCoolCounter < 20) percentCoolCounter++;}

void decrementPercentCoolCounter(){
    if (percentCoolCounter > 0) percentCoolCounter--;}

void incrementHS(){
    if (HS < 5) HS++;}

void decrementHS(){
    if (HS > 0) HS--;}



void __interrupt(high_priority) highIsr (void) {

    if(PIR2bits.TMR3IF) {
        RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L);
        PIR2bits.TMR3IF = 0;
        if (operationMode == 2 && (autoMode==0 || ((autoMode==2) && (RT > SP)))) {
            if(RT<(SP-HS)) {
                set_pwm1_percent(0.0);
                if ( timerCounter++ <  5) PORTCbits.RC5 = 1;
                else if ( timerCounter <= 10 ) PORTCbits.RC5 = 0;
                else timerCounter = 0;
            }else {
                PORTCbits.RC5 = 0;
            }
        }
        
        if ((operationMode == 0) || (operationMode == 2 && autoMode == 1) || (operationMode == 2 && autoMode == 2)) {
            if(!(operationMode == 2 && autoMode == 2)) {
                set_pwm1_percent(0.0);
            }
            if ( timerCounter++ <  percentHeatCounter) PORTCbits.RC5 = 1;
            else if ( timerCounter <= 10 ) PORTCbits.RC5 = 0;
            else timerCounter = 0;
        } else if (operationMode == 1) {
            PORTCbits.RC5 = 0;
            set_pwm1_percent(percentCoolCounter * 10.0);
        }
        
        restartTimer3();
        
    } else if(INTCONbits.INT0IF) {
        changeOperationMode();
        INTCONbits.INT0IF = 0;
    }else if(INTCON3bits.INT1IF) {
        if(operationMode == 0) decrementPercentHeatCounter();
        if(operationMode == 1) decrementPercentCoolCounter();
        if(operationMode == 2) decrementHS();
        INTCON3bits.INT1IF = 0;
    }else if(INTCON3bits.INT2IF) {
        if(operationMode == 0) incrementPercentHeatCounter();
        if(operationMode == 1) incrementPercentCoolCounter();
        if(operationMode == 2) incrementHS();
        INTCON3bits.INT2IF = 0;
    }  
} 

void init(void) {
    
    ADCON0 = 0x00 ;
    ADCON1 = 0x0C ;
    TRISA  = 0xFF ;
    TRISB  = 0xFF ;
    TRISC  = 0x80 ;
    TRISD  = 0x00 ;
    TRISE  = 0x00 ;
    PORTD  = 0    ;
    PIE1   = 0    ;
    
    unsigned char dummy;
    dummy = RCREG;
    
    BAUDCONbits.BRG16 = 0;
    TXSTA             = 0;
    SPBRG             = 25;
    TXSTAbits.BRGH    = 1;
    TXSTAbits.TXEN    = 1;
    RCSTA             = 0;
    RCSTAbits.CREN    = 1;
    RCSTAbits.SPEN    = 1;

    T1CONbits.TMR1CS  = 1;
    T1CONbits.TMR1ON  = 1;
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;
    
    T3CONbits.T3CKPS1 = 0;
    T3CONbits.T3CKPS0 = 0;    
    T3CONbits.T3SYNC  = 1;
    T3CONbits.T3CKPS0 = 0;
    T3CONbits.T3CKPS1 = 0;
    T3CONbits.RD16    = 1;

    TMR1H = 0;
    TMR1L = 0;
    TMR3H = 0x40;
    TMR3L = 0x00;

    T3CONbits.TMR3ON = 1;
    T3CONbits.TMR3CS = 0;

    PIE2bits.TMR3IE = 1;
    IPR2bits.TMR3IP = 1;

    RCONbits.IPEN   = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
    
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.T0IE = 1;
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCON3bits.INT1IE  = 1;
    INTCON3bits.INT2IE  = 1;
    
    TRISCbits.RC0 = 1;
    PORTCbits.RC2 = 0;
    PORTCbits.RC5 = 0;
    
    HS = 0;
    SP = 0;
    OT = 0;
    RT = 0.0;
    coolError          = 0;
    heatError          = 0;
    operationMode      = 0;
    autoMode           = 0;
    percentHeatCounter = 0;
    percentCoolCounter = 0;
    timerCounter       = 0;
    
    RCONbits.IPEN     = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.INT0IE = 1;
    
    init_pwm1();
    
}
void setupSerial(void) {
    unsigned char dummy;
    BAUDCONbits.BRG16 = 0;
    TXSTA = 0;
    SPBRG = 25;
    SPBRGH = 0;
    TXSTAbits.BRGH = 1; //baud rate high speed option
    TXSTAbits.TXEN = 1; //	;enable transmission

    RCSTA = 0; // ;SERIAL RECEPTION W/ 8 BITS,
    RCSTAbits.CREN = 1; //;enable reception
    RCSTAbits.SPEN = 1;
    ; //enable serial port
    dummy = RCREG; //, W        ; clear the receiver buffer      
    dummy = RCREG; //,W         ; clear the receiver buffer
    return;
}
void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}
// This function is needed for measuring speed
void initTimers01(void) {
    T0CON = 0;
    //T0CONbits.T0CS = 0;
    //T0CONbits.PSA = 0;
    //T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);

    T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;


    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}
void send_string_no_lib(unsigned char *p) {
    while (*p) {
        send_byte_no_lib(*p); //or use the send_byte_no_lib()
        p++;
    }
}
void send_byte_no_lib(unsigned char c) {
    while (!TXSTAbits.TRMT)//fixed 
    {
        CLRWDT(); //if enabled  
    }
    TXREG = c;
}

unsigned char is_byte_available(void)
{
    if (RCSTAbits.FERR || RCSTAbits.OERR)//check for error
    {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    // wait until byte available
    if (PIR1bits.RCIF) return 1;
    else return 0;
    
}

unsigned char read_byte_no_lib(void) {
    unsigned char c;
    c = RCREG;
    return c;
}


void main(void) {
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    init(); 
    init_pwm1();

    lcd_init();
    init_adc_no_lib();
    send_string_no_lib((unsigned char *) "\r\rReading DAta About the Air conditioner");

    lcd_putc('\f'); // Clears The Display
    unsigned char RecvedChar = 0;
    unsigned char SendToSerial = 0;
    int RPS;
    
    HS                 = 0;
    SP                 = 0;
    OT                 = 0;
    RT                 = 0.0;
    coolError          = 0;
    heatError          = 0;
    operationMode      = 0;
    autoMode           = 0;
    percentHeatCounter = 0;
    percentCoolCounter = 0;
    timerCounter       = 0;
    
//    initTimers01();   // These will be used to measure the speed
     TRISCbits.RC0 = 1; //Timer1 clock
    while (1) {
        CLRWDT();
        
        SP = read_adc_voltage(0) * 20;
        OT = read_adc_voltage(1) * 20;
        RT = read_adc_voltage(2) * 100.0;

        lcd_gotoxy(1, 1);
        sprintf(Buffer, "RT:%4.2fC ", RT);
        lcd_puts(Buffer);
        
        lcd_gotoxy(14, 1);
        lcd_puts("H C");
        
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "SP:%4.2fC ", SP);
        lcd_puts(Buffer);
        
        lcd_gotoxy(14, 2);
        lcd_puts(PORTCbits.RC5 ? "Y" : "N");
        
        lcd_gotoxy(16, 2);
        lcd_puts(PORTCbits.RC2 ? "Y" : "N");
        
        if (operationMode != 2) {
            lcd_gotoxy(1, 3);
            sprintf(Buffer, "OT:%4.2fC %s:%0d%% ", OT,
                    operationMode == 0 ? "H" : "C",
                    operationMode == 0 ? percentHeatCounter * 5 : percentCoolCounter * 5);
            lcd_puts(Buffer);
        } else {
            lcd_gotoxy(1, 3);
            sprintf(Buffer, "OT:%4.2fC", OT);
            lcd_puts(Buffer);
        }
        
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "MD:%s", operationMode == 0 ? "Heat" : operationMode == 1 ? "Cool" : "Auto");
        lcd_puts(Buffer);
        
        lcd_gotoxy(13, 4);
//        if (operationMode == 2)sprintf(Buffer, "HS=%d", HS);
//        else sprintf(Buffer, "    ");
//        lcd_puts(Buffer);
//        
        if (operationMode == 2) autoMode = OT > SUMMER_T ? 0 : OT < WINTER_T ? 1 : 2;
        
        lcd_gotoxy(9, 4);
        if (operationMode == 2)sprintf(Buffer, "%s", autoMode == 0 ? "CL" : autoMode == 1 ? "HT" : "HC");
        else sprintf(Buffer, "  ");
        lcd_puts(Buffer);
        
        if (operationMode == 2) {
            if (autoMode == 0) autoCool();
            else if (autoMode == 1) autoHeat();
            else (SP - RT) > 0 ? autoHeat() : autoCool();
        }
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2
            if (is_byte_available()) { // Read serial, if receive S
                RecvedChar = read_byte_no_lib(); // Then start sending to serial
                if (RecvedChar == 'S') SendToSerial = 1;
                else if (RecvedChar == 'N') SendToSerial = 0;// Stop sending to serialif N is recived
                else {
                    /*No Change */
                }
            }
            if (SendToSerial) {
                // If Sending to Serial ( after receiving S, send values)
                sprintf(Buffer, "SP:%4.2fC  OT:%4.2fC\r\n", SP,OT);
                send_string_no_lib(Buffer);
//                sprintf(Buffer, "OT:%4.2fC\r\n",  OT);
//                send_string_no_lib(Buffer);
                sprintf(Buffer, "RT:%4.2fC\r\n",RT);
                send_string_no_lib(Buffer);
                sprintf(Buffer, "percentHeatCounter:%0d%% percentCoolCounter:%0d%%\r\n",percentHeatCounter*5,percentCoolCounter*5);
                send_string_no_lib(Buffer);
//                sprintf(Buffer, "percentCoolCounter:%0d%%\r\n",percentCoolCounter*5);
//                send_string_no_lib(Buffer);
                sprintf(Buffer, "MD:%s", operationMode == 0 ? "Heat:ON Cool :OFF\r\n" : operationMode == 1 ? "Cool:ON Heat :OFF\r\n" : "Auto\n");
                send_string_no_lib(Buffer);
            }
        }
        
        
        
        
    }
}