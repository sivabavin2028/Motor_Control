#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "BCD.h"
#define _XTAL_FREQ 8000000

// CONFIG
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = ON
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF
#pragma config BOR4V = BOR21V
#pragma config WRT = OFF

// -------- PIN --------
//#define PWM_PIN   RC2
#define MOTOR_IN1 RA6
#define MOTOR_IN2 RA7
#define LIMIT RA5
#define RELAY RA4

#define SS1 RC4
#define SS2 RC7
#define SS3 RC6
#define SS4 RC5

#define BTN_UP   RC0
#define BTN_DOWN RC1
#define BTN_SET  RC3

int constrain(int,int,int);
void Duty(unsigned char );
void delay_ms(unsigned int );
void displayBattery(uint8_t );

typedef enum {
    MODE_STARTUP,
    MODE_IDLE,
    MODE_SET_CURRENT,
    MODE_SET_VOLTAGE,
    MODE_RUN
} system_mode_t;

system_mode_t mode = MODE_STARTUP;

typedef struct{
    float voltage;
    unsigned char display[4];
}voltage_mode_t;

voltage_mode_t modes[3] = {
    {14.5, {SS1_V, SS2_r, SS1_L, SS2_A}},  // VRLA
    {15.5, {SS1_A, SS2_C, SS1_i, SS2_d}},  // ACID
    {16.0, {SS1_t, SS2_r, SS1_b, SS2_U}}   // TRBU
};

// -------- VARIABLES --------
float current_limit = 0;
unsigned char v_index = 0;
float selected_voltage = 14.5;
unsigned char selected_index = 0;
float voltage2;
float ovlo_volt = 0.0f;
volatile unsigned int ovlo_start_tick = 0;

uint8_t c = 0;

volatile unsigned int counter_pwm = 0;
volatile unsigned int timer0_tick = 0;
volatile unsigned int duty = 5;
static unsigned char stage = 0;
 
unsigned char last_set = 0;
unsigned char last_up = 0;
unsigned char last_down = 0;
unsigned char last_volt = 0;

bool outputOk;

volatile unsigned char ts1;
volatile unsigned char ts2;
volatile unsigned char ts3;
volatile unsigned char ts4;

float tar_vol = 0.0f;
float ref_vol = 0.0f;
float adc = 0;

volatile unsigned char direction = 0;

volatile unsigned char limit_mode = 0;
volatile unsigned char limit_step = 0;
static unsigned char last_limit = 1;

volatile unsigned long millis_count = 0;
//volatile unsigned char cv_mode = 0;

#define CC_MODE             0
#define CV_MODE             1
bool SAFE_CON = 1;
bool volt_con = 1;
bool batteryDetected = false;
bool selected_voltage_flag = true;

#define CURRENT_DEADBAND    0.20f
#define CV_DEADBAND         0.05f
#define CV_EXIT_BAND        0.20f

static unsigned char control_mode = CC_MODE;
static unsigned char motor_state = 0;
static unsigned long motor_prev_time = 0;

//extern int SS1_BCD[];
//extern int SS2_BCD[];

#define TOLERANCE 0.3 

// ---------- ADC ----------
void ADC_Init()
{
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;  

    ANSEL = 0x07;
    ANSELH = 0x00;

    ADCON1bits.ADFM = 1;
    ADCON0bits.ADCS = 0b10;
    ADCON0bits.ADON = 1;

    __delay_ms(5);
}

unsigned int readAdc(unsigned char ch)
{
    ADCON0 &= 0b11000011;
    ADCON0 |= (ch << 2);

    __delay_us(10);

    ADON = 1;
    GO_nDONE = 1;
    while(GO_nDONE);

    return ((ADRESH << 8) | ADRESL);
    
}

float readAdcAveraged(unsigned char channel, unsigned char samples)
{
     float sum = 0;
    unsigned char i;

    for(i = 0; i < samples; i++)
    {
        sum += readAdc(channel);
    }

    return (float)(sum / samples);
}

// ---------- GPIO ----------
void gpio_conf()
{
    TRISCbits.TRISC3 = 1;
    TRISAbits.TRISA6 = 0;
    TRISAbits.TRISA7 = 0;
    TRISAbits.TRISA5 = 1;  //LIMIT INPUT 
    TRISAbits.TRISA4 = 0;  //RELAY 
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 0;
    
    TRISB = 0X00;
    
     SS1=1;
     SS2=1;
     SS3=1;
     SS4=1;
    
    
    MOTOR_IN1 = 0;
    MOTOR_IN2 = 0;
    //PWM_PIN = 0;
    RELAY = 0;
    PORTB = 0X00;
}

// ---------- TIMER ----------
void timer_conf()
{

     OPTION_REG = 0b00000100;  // Prescaler 1:256 assign to Timer0

    TMR0 = 0x83;              // preload value

    INTCONbits.TMR0IF = 0;    // clear flag
    INTCONbits.TMR0IE = 1;    // enable Timer0 interrupt

    INTCONbits.GIE = 1;       // global interrupt enable

}


void PWM1_Init(void)
{
    TRISCbits.TRISC2 = 0;

    PR2 = 124;     //124   1khz 1ms time period

    CCP1CONbits.CCP1M = 0b1100;

    T2CONbits.T2CKPS = 0b11;

    TMR2 = 0;

    CCPR1L = 0;
    CCP1CONbits.DC1B = 0;

    PIE1bits.TMR2IE = 0;

    T2CONbits.TMR2ON = 1;
}

//---------------------------------------------------------------------------------

void splitDigits(int val, unsigned char* d_100, unsigned char* d_10, unsigned char* d_1)
{
    if (val < 10)
    {
        *d_100 = SS1_BCD[0];
        *d_10 = SS2_BCD[0];
        *d_1 = SS1_BCD[val % 10];
        return;
    }
    else if (val < 100)
    {
        *d_100 = SS1_BCD[0];
        *d_10 = SS2_BCD[(val / 10) % 10];
        *d_1 = SS1_BCD[val % 10];
        return;
    }
    else
    {
        *d_100 = SS1_BCD[(val / 100) % 10];
        *d_10 = SS2_BCD[(val / 10) % 10];
        *d_1 = SS1_BCD[val % 10];
    }
}


void splitFloat(float val,
                unsigned char *d_1000,
                unsigned char *d_100,
                unsigned char *d_10,
                unsigned char *d_1)
{
    // Scale by 10 so all thresholds/digits are handled from one integer.
    // e.g. 9.67 -> 97, 10.0 -> 100, 12.34 -> 123, 123.45 -> 1234
    int temp = (int)(val * 10.0f + 0.5f);

    if(temp < 100)
    {
        // VALUE < 10   ->  Display:  OFF  OFF  X.  X   (e.g. 9.6)
        int d3 = (temp / 10) % 10;   // units digit
        int d4 = temp % 10;          // tenths digit

        *d_1000 = OFF;
        *d_100  = OFF;
        *d_10   = SS1_BCD[d3] | SS1_DOT;
        *d_1    = SS2_BCD[d4];
    }
    else if(temp == 100)
    {
        // VALUE == 10  ->  Display:  OFF  OFF  1   0   (no decimal point)
        *d_1000 = OFF;
        *d_100  = OFF;
        *d_10   = SS1_BCD[1];
        *d_1    = SS2_BCD[0];
    }
    else if(temp < 1000)
    {
        // 10 < VALUE < 100  ->  Display:  OFF  X  X.  X   (e.g. 12.3)
        int d2 = (temp / 100) % 10;  // tens digit
        int d3 = (temp / 10) % 10;   // units digit
        int d4 = temp % 10;          // tenths digit

        *d_1000 = OFF;
        *d_100  = SS2_BCD[d2];
        *d_10   = SS1_BCD[d3] | SS1_DOT;
        *d_1    = SS2_BCD[d4];
    }
    else
    {
        // VALUE >= 100  ->  Display:  X  X  X.  X   (e.g. 123.4)
        int d1 = (temp / 1000) % 10;
        int d2 = (temp / 100) % 10;
        int d3 = (temp / 10) % 10;
        int d4 = temp % 10;

        *d_1000 = SS1_BCD[d1];
        *d_100  = SS2_BCD[d2];
        *d_10   = SS1_BCD[d3] | SS1_DOT;
        *d_1    = SS2_BCD[d4];
    }
}


//---------------------------------------------------------------------------------------------

void custom_display(unsigned char one, unsigned char two, unsigned char three, unsigned char four)
{
    PORTB = one;
    SS1 = 0;
    __delay_ms(1);  // Changed back to original 1ms
    SS1 = 1;

    PORTB = two;
    SS2 = 0;
    __delay_ms(1);
    SS2 = 1;

    PORTB = three;
    SS3 = 0;
    __delay_ms(1);
    SS3 = 1;

    PORTB = four;
    SS4 = 0;
    __delay_ms(1);
    SS4 = 1;
}


// ---------- ISR ----------
void __interrupt() ISR()
{
 //---------------------------------TIMER0----------------------------------------
    if (TMR0IF)
    {
        static unsigned char s1;
        static unsigned char s2;
        static unsigned char s3;
        static unsigned char s4;

        if (outputOk == 1)
        {
            outputOk = 0;
            s1 = ts1;
            s2 = ts2;
            s3 = ts3;
            s4 = ts4;
        }

        timer0_tick++;
        custom_display(s1, s2, s3, s4);

        TMR0IF = 0;
        TMR0 = 0x83;
    }
}





// ---------- MOTOR ----------
void motor_stop()
{
    MOTOR_IN1 = 0;
    MOTOR_IN2 = 0;
}

void motor_forward()
{
    MOTOR_IN1 = 1;
    MOTOR_IN2 = 0;
    direction = 1;
    stage = 0;
}

void motor_reverse()
{
    MOTOR_IN1 = 0;
    MOTOR_IN2 = 1;
    direction = 2;
    stage = 1;
}

float getVoltage(unsigned char ch)
{
//    unsigned long sum = 0;
//
//    for(int i=0; i<4; i++)
//    {
//        sum += readAdc(ch);
//        __delay_us(25);
//    }
//
//    unsigned int adc = sum / 4;
//
//    return (adc * 24.0f) / 1023.0f;
    adc = readAdcAveraged(ch, 10);
       
       voltage2 = (0.20287f * adc) + 0.044f;   //1.244f  0.444f
       return voltage2;
}

// ---------- MAIN ----------
void main()
{
     OSCCON = 0b01110100;
   // float vtg ;
    
    gpio_conf();
    timer_conf();
    ADC_Init();
    PWM1_Init();
    
 
    ts1 = SS1_G;
    ts2 = SS2_V;
    ts3 = SS1_C;
    ts4 = OFF;
    outputOk = 1;
    delay_ms(500);
    
    
    unsigned char startup_mode = 1;
    static unsigned char prev_limit = 1;
    unsigned char limit_active = 0;

    static unsigned char motor_state = 0;
    static unsigned long motor_prev_time = 0;

    while (1)
    { 
       voltage2 = getVoltage(2);
       
       
    if(batteryDetected == false)
    {
        if((voltage2 >= 0.0f) && (voltage2 < 14.0f))
        {
            c = 1;
            batteryDetected = true;
        }
        else if((voltage2 >= 14.0f) && (voltage2 < 27.0f))
        {
            c = 2;
            batteryDetected = true;
        }
        else if((voltage2 >= 27.0f) && (voltage2 < 40.0f))
        {
            c = 3;
            batteryDetected = true;
        }
        else if((voltage2 >= 40.0f) && (voltage2 < 53.0f))
        {
            c = 4;
            batteryDetected = true;
        }
        else if((voltage2 >= 53.0f) && (voltage2 < 66.0f))
        {
            c = 5;
            batteryDetected = true;
        }
        else if((voltage2 >= 66.0f) && (voltage2 < 79.0f))
        {
            c = 6;
            batteryDetected = true;
        }
        else if((voltage2 >= 79.0f) && (voltage2 < 92.0f))
        {
            c = 7;
            batteryDetected = true;
        }
        else if((voltage2 >= 92.0f) && (voltage2 < 105.0f))
        {
            c = 8;
            batteryDetected = true;
        }
        else if((voltage2 >= 105.0f) && (voltage2 < 118.0f))
        {
            c = 9;
            batteryDetected = true;
        }
        else if((voltage2 >= 118.0f) && (voltage2 < 131.0f))
        {
            c = 10;
            batteryDetected = true;
        }
        else
        {
            c = 255;      // Battery Not Detected
        }
    }
      
        // ================= SET BUTTON (3 STEP CONTROL) =================
    static unsigned char set_state = 0;

    if(BTN_SET == 1 && set_state == 0)
    {
        delay_ms(10);  // Changed back to original 20ms

        if(BTN_SET == 1)
        {
            set_state = 1;

                if(mode == MODE_STARTUP)
                {
                    mode = MODE_SET_CURRENT;
                }
                else if(mode == MODE_SET_CURRENT)
                {
                    mode = MODE_SET_VOLTAGE;
                }
                else if(mode == MODE_SET_VOLTAGE)
                {
//                    selected_index = v_index;
//                    selected_voltage = modes[selected_index].voltage;
//
//                    mode = MODE_RUN;
//
//                    // RESET FLAGS
//                    limit_active = 0;
//                    stage = 0;
                    
                    selected_index = v_index;

                    if(c >= 1 && c <= 10)
                    {
                        selected_voltage = modes[selected_index].voltage * c;
                    }

                    mode = MODE_RUN;

                    // RESET FLAGS
                    limit_active = 0;
                    stage = 0;
                }
        }
    }

    if(BTN_SET == 0)
    {
        set_state = 0;
    }

    // ================= STARTUP SAFETY =================

    if(startup_mode)
    {
        duty = 65;
        Duty(duty);

        if (LIMIT == 0)
        {
            ts1 = SS1_S;
            ts2 = SS2_E;
            ts3 = SS1_t;
            ts4 = OFF;
            outputOk = 1;

            motor_stop();
            delay_ms(50);  // Changed back to original 200ms

            motor_forward();
            delay_ms(25);  // Changed back to original 200ms

            motor_stop();
            startup_mode = 0;
            continue;
        }

        motor_reverse();
        continue;
     
    }

    // ================= MODE: STARTUP DISPLAY =================
    if(mode == MODE_STARTUP)
    {
        static unsigned char toggle = 0;

        voltage2 = getVoltage(2);

        if(voltage2 < 2.0)   // LOW BATTERY
        {
            if(toggle == 0)
            { 
                // OP.BT
                ts1 = SS1_o;
                ts2 = SS4_P;
                ts3 = SS1_b;
                ts4 = SS4_t;
            }
            else
            {
                // LO.BT
                ts1 = SS1_L;
                ts2 = SS2_o;
                ts3 = SS1_b;
                ts4 = SS4_t;
            }

            toggle = !toggle;
            outputOk = 1;
            delay_ms(250);  // Changed back to original 500ms
        }
        else
        {
            // NORMAL STARTUP DISPLAY
            ts1 = SS1_S;
            ts2 = SS2_E;
            ts3 = SS1_t;
            ts4 = OFF;

            outputOk = 1;
        }
    }

    // ================= MODE: CURRENT SET =================
    if(mode == MODE_SET_CURRENT)
    {
        adc = readAdcAveraged(0, 50);
        current_limit = ((adc * 10.0f ) / 814.0f);
        //current_limit *= 2.5f;
        splitFloat(current_limit, &ts1, &ts2, &ts3, &ts4);
        outputOk = 1;
    }

    // ================= VOLTAGE SELECT BUTTONS =================
    if(mode == MODE_SET_VOLTAGE)
    {
        // UP
        if(last_up == 0 && BTN_UP == 1)
        {
            delay_ms(10);  // Changed back to original 20ms
            if (BTN_UP == 1)
            {
                v_index++;
                if (v_index > 2) v_index = 0;
            }
        }
        last_up = BTN_UP;

        // DOWN
        if (last_down == 0 && BTN_DOWN == 1)
        {
            delay_ms(10);  // Changed back to original 20ms
            if (BTN_DOWN == 1)
            {
                if (v_index == 0) 
                    v_index = 2;
                else 
                    v_index--;
            }
        }
        last_down = BTN_DOWN;

        // DISPLAY
        ts1 = modes[v_index].display[0];
        ts2 = modes[v_index].display[1];
        ts3 = modes[v_index].display[2];
        ts4 = modes[v_index].display[3];

        // Store the selected voltage in selected_voltage
        selected_voltage = modes[v_index].voltage ;
        //mode = MODE_RUN;
        outputOk = 1;
        
        continue;
    }

    // ================= MODE: RUN =================
    if (mode == MODE_RUN)
    {
       duty = 50;
       Duty(duty);
        voltage2 = getVoltage(2);
        

        // -------- RELAY CONTROL --------
        static unsigned char relay_state = 0;

        if (voltage2 > 2.0)
        {
            RELAY = 1;
            relay_state = 1;
        }
        else
        {
            RELAY = 0;
            relay_state = 0;
            motor_stop();
            continue;   // STOP if no battery
        }
        
        adc = readAdcAveraged(1, 50);
        ovlo_volt = ((adc * 10.0f) / 814.0f);    // OVER LOAD VOLTAGE

//======================================================
// OVLO - OVER LOAD CONDITION
//======================================================

if(ovlo_volt > 11.0f)
{
    if(ovlo_start_tick == 0)
    {
        // 11.5V cross panna starting time save pannum
        ovlo_start_tick = timer0_tick;
    }

    // 10 seconds = 5000 Timer0 ticks
    if((unsigned int)(timer0_tick - ovlo_start_tick) >= 5000)
    {
        RELAY = 0;
        break;
    }
}
else
{
    // Voltage came back below 11.5V
    // 10 second timing reset
    ovlo_start_tick = 0;
}

//---------------------------------------------------------------------------------
               // -------- LIMIT SWITCH --------
        if (prev_limit == 1 && LIMIT == 0)
        {
            delay_ms(10);  // Reduced from 100ms

            if (LIMIT == 0)
            {
                if (stage == 0 && direction == 1)
                {
                    motor_stop();
                    delay_ms(50);  // Changed back to original 100ms

                    motor_reverse();
                    delay_ms(25);  // Changed back to original 200ms

                    motor_stop();
                    delay_ms(200);  // Changed back to original 500ms

                    motor_reverse();

                    stage = 1;
                    limit_active = 1;
                }
                else if (stage == 1 && direction == 2)
                {
                    motor_stop();
                    delay_ms(50);  // Changed back to original 100ms

                    motor_forward();
                    delay_ms(25);  // Changed back to original 200ms

                    motor_stop();

                    stage = 2;
                    limit_active = 0;
                }
            }
        }
        prev_limit = LIMIT;

   if (LIMIT == 1 && limit_active == 0 && RELAY == 1)
   {
    //======================================================
    // READ CURRENT
    //======================================================

        static float actual_current = 0.0f;
        static float current_error  = 0.0f;

    //======================================================
    // READ CURRENT
    //======================================================

        adc = readAdcAveraged(1, 50);

        actual_current = ((adc * 10.0f) / 814.0f);
        //actual_current = actual_current * 2.0f;

        current_error = current_limit - actual_current;
        
         splitFloat(actual_current, &ts1, &ts2, &ts3, &ts4);
        outputOk = 1;
        delay_ms(700);
        
        
    //======================================================
    // READ VOLTAGE
    //======================================================

        voltage2 = getVoltage(2);
        if(voltage2 >= selected_voltage)
        {
            control_mode = CV_MODE;
        }

    //======================================================
    // CC -> CV TRANSITION
    //======================================================
        if((control_mode == CC_MODE) && (SAFE_CON == 1))
        {
            if(current_error > 2.0f)
            {
                motor_forward();
                delay_ms(160);
                motor_stop();
            }
            else if((current_error <= 2.0f) && (current_error > 1.0f))
            {
                motor_forward();
                delay_ms(140);
                motor_stop();
            }
            else if((current_error <= 1.0f) && (current_error > 0.1f))
            {
                motor_forward();
                delay_ms(30);
                motor_stop();
            }
            else if((current_error <= 0.1f) && (current_error >= -0.1f))
            {
                motor_stop();
                SAFE_CON = 0;
            }
            else if((current_error < -0.1f) && (current_error >= -1.0f))
            {
                motor_reverse();
                delay_ms(30);
                motor_stop();
            }
            else if((current_error < -1.0f) && (current_error >= -2.0f))
            {
                motor_reverse();
                delay_ms(140);
                motor_stop();
            }
            else if(current_error < -2.0f)
            {
                motor_reverse();
                delay_ms(160);
                motor_stop();
            }       
        }

        if(SAFE_CON == 0)
        {
            if((current_error > 0.1f) || (current_error < -0.1f))
            {   
                SAFE_CON = 1;
            }
        }
   }

//======================================================
// CV MODE
//======================================================

        if((control_mode == CV_MODE) && volt_con == 1)
        {
          
        //==================================================
        // CONSTANT VOLTAGE MODE
        //==================================================
        // IMPORTANT:
        // Current is completely ignored here.
        // Only voltage controls the motor.
        //==================================================
          

        // Voltage TOO HIGH
            if(voltage2 >= (selected_voltage + 0.1f))
            {
                // Reduce Variac output
                motor_reverse();
                delay_ms(40);
                motor_stop();
                
            }

        // Voltage TOO LOW
            else if(voltage2 <= (selected_voltage - 0.1f))
            {
                // Increase Variac output
                motor_forward();
                delay_ms(40);
                motor_stop();
               
            }

            // Voltage is within target window
            else
            {
                motor_stop();
                volt_con = 0;
            }
        }
        if(volt_con == 0)
        {
            if((voltage2 > (selected_voltage + 0.1f)) || (voltage2 < (selected_voltage - 0.1f)))
            {
                volt_con = 1;
            }
        }

        splitFloat(voltage2, &ts1, &ts2, &ts3, &ts4);
        outputOk = 1;
        delay_ms(600);

//DISPLAYING THE NUMBER OF BATTERY
        
        displayBattery(c);
        delay_ms(600);
        
        outputOk = 0;
        splitFloat(selected_voltage, &ts1, &ts2, &ts3, &ts4);
        outputOk = 1;
        delay_ms(600);
        
        }
    }
    
    while(1)
    {
       ts1 = SS1_t;
        ts2 = SS2_r;
        ts3 = SS1_i;
        ts4 = SS4_P; 
        outputOk = true; 
    }
}

void Duty(unsigned char duty_percent)
{
    unsigned int duty_count;

    if(duty_percent > 100)
        duty_percent = 100;

    // 10-bit PWM duty calculation
    duty_count = ((unsigned long)duty_percent *
                  4UL *
                  (PR2 + 1)) / 100UL;

    // Upper 8 bits
    CCPR1L = duty_count >> 2;

    // Lower 2 bits
    CCP1CONbits.DC1B = duty_count & 0x03;
}

void delay_ms(unsigned int delay)
{
    unsigned int start_tick;
    unsigned int required_tick;

    required_tick = delay / 2;      // Timer0 interrupt = 2ms

    start_tick = timer0_tick;

    while((unsigned int)(timer0_tick - start_tick) < required_tick)
    {
        // Wait here
    }
}

void displayBattery(uint8_t batteryNo)
{
    outputOk = false;

    ts1 = SS1_b;      // First digit = B
    ts2 = SS4_t;      // Second digit = t

    if(batteryNo >= 1 && batteryNo <= 9)
    {
        ts3 = SS1_BCD[batteryNo];
        ts4 = OFF;
    }
    else if(batteryNo == 10)
    {
        ts3 = SS1_BCD[1];
        ts4 = SS2_BCD[0];
    }
    else
    {
        ts1 = OFF;
        ts2 = OFF;
        ts3 = OFF;
        ts4 = OFF;
    }

    outputOk = true;
}