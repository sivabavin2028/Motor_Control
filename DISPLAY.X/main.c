#include <xc.h>
#include <stdbool.h>
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
#define PWM_PIN   RC3
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
#define BTN_SET  RC2

int constrain(int,int,int);

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
    {16.5, {SS1_t, SS2_r, SS1_b, SS2_U}}   // TRBU
};

// -------- VARIABLES --------
float current_limit = 0;
unsigned char v_index = 0;
float selected_voltage = 14.5;
unsigned char selected_index = 0;
float voltage2;

volatile unsigned int counter_pwm = 0;
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

volatile unsigned char direction = 0;

volatile unsigned char limit_mode = 0;
volatile unsigned char limit_step = 0;
static unsigned char last_limit = 1;

extern int SS1_BCD[];
extern int SS2_BCD[];

#define TOLERANCE 0.8 

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

// ---------- GPIO ----------
void gpio_conf()
{
    TRISCbits.TRISC3 = 0;
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
    TRISCbits.TRISC2 = 1;
    
    TRISB = 0X00;
    
     SS1=1;
     SS2=1;
     SS3=1;
     SS4=1;
    
    
    MOTOR_IN1 = 0;
    MOTOR_IN2 = 0;
    PWM_PIN = 0;
    RELAY = 0;
    PORTB = 0X00;
}

// ---------- TIMER ----------
void timer_conf()
{
    T1CON = 0x01;

    TMR1H = 0xF0;
    TMR1L = 0x60;
    
    TMR1IF = 0;
    TMR1IE = 1;
    PEIE = 1;
    GIE = 1;
    
    //-------------------------
     OPTION_REG = 0b00000100;  // Prescaler 1:256 assign to Timer0

    TMR0 = 0x83;              // preload value

    INTCONbits.TMR0IF = 0;    // clear flag
    INTCONbits.TMR0IE = 1;    // enable Timer0 interrupt

    INTCONbits.GIE = 1;       // global interrupt enable
}

//---------------------------------------------------------------------------------

void splitDigits(int val, unsigned char* d_100, unsigned char* d_10, unsigned char* d_1)
{
if(val<10){
  *d_100 = SS1_BCD[0];
  *d_10 = SS2_BCD[0];
  *d_1 = SS1_BCD[val%10];
  return;
}
else if(val>=10 && val <100){
  *d_100 = SS1_BCD[0];
  *d_10 = SS2_BCD[(val/10)%10];
  *d_1 = SS1_BCD[val%10];
  return;
}
else
{
  *d_100 = SS1_BCD[(val/100)%10];
  *d_10 = SS2_BCD[(val/10)%10];
  *d_1 = SS1_BCD[val%10];
}
}

int constrain(int data,int min,int max)
{
    if(data<min)
        return min;
    if(data>max)
        return max;
    return data;
}

void splitFloat(float val, unsigned char* d_1000, unsigned char* d_100, unsigned char* d_10, unsigned char* d_1)
{
  if (val < 10)
    {
      int v_one = (int)val % 10;
      int mv_hun = (int)((val - (int)val) * 10) % 10;
      int mv_ten = (int)((val - (int)val) * 100) % 10;
      *d_1000  = SS1_BCD[0];
      *d_100 = SS2_BCD[v_one] | SS2_DOT;
      *d_10 = SS1_BCD[mv_hun];
      *d_1= SS2_BCD[mv_ten];
    }
  else if (val >= 10)
  {
    int v_ten = ((int)val / 10) % 10;
    int v_one = (int)val % 10;

    int mv_hun = (int)((val - (int)val) * 10) % 10;
    int mv_ten = (int)((val - (int)val) * 100) % 10;
    *d_1000 = SS1_BCD[v_ten];
    *d_100 = SS2_BCD[v_one] | SS2_DOT;
    *d_10 = SS1_BCD[mv_hun];
    *d_1 = SS2_BCD[mv_ten]; 
  }

}

//---------------------------------------------------------------------------------------------

void custom_display(unsigned char one, unsigned char two, unsigned char three, unsigned char four)
{
  PORTB = one;
  SS1 = 0;
  __delay_ms(1);
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
//-------------------------------TIMER1---------------------------------
    if(TMR1IF)
    {
     
        TMR1IF = 0;

        TMR1H = 0xF0;
        TMR1L = 0x60;

        counter_pwm++;
        if(counter_pwm >= 100) counter_pwm = 0;

        if(counter_pwm < duty)
            PWM_PIN = 1;
        else
            PWM_PIN = 0;
    }
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
    custom_display(s1, s2, s3, s4);

    TMR0IF = 0;
    TMR0 = 0X83;
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
    unsigned long sum = 0;

    for(int i=0; i<4; i++)
    {
        sum += readAdc(ch);
        __delay_us(50);
    }

    unsigned int adc = sum / 4;

    return (adc * 24.0f) / 1023.0f;
}

float getAmps(unsigned char ch)
{
    unsigned long sum = 0;

    for(int i=0; i<4; i++)
    {
        sum += readAdc(ch);
        __delay_us(50);
    }

    unsigned int adc = sum / 4;

    return (adc * 10.0f) / 1023.0f;
}


// ---------- MAIN ----------
void main()
{
   // float vtg ;
    
    gpio_conf();
    timer_conf();
    ADC_Init();
 
    ts1 = SS1_G;
    ts2 = SS2_V;
    ts3 = SS1_C;
    ts4 = OFF;
    outputOk = 1;
    __delay_ms(200);
    
    /*vtg = 12.05;
    splitFloat(vtg, &ts1, &ts2, &ts3, &ts4);
    outputOk = true;
    __delay_ms(200);*/
    
    unsigned char startup_mode = 1;
    static unsigned char prev_limit = 1;
    unsigned char limit_active = 0;
    unsigned char system_done = 0;
   
  while(1)
{
    // ================= SET BUTTON (3 STEP CONTROL) =================
    static unsigned char set_state = 0;

    if(BTN_SET == 1 && set_state == 0)
    {
        __delay_ms(20);

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
                    selected_index = v_index;
                    selected_voltage = modes[selected_index].voltage;

                    mode = MODE_RUN;

                    // RESET FLAGS
                    system_done = 0;
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
        duty = 5;
        motor_reverse();

        if(LIMIT == 0)
        {
            motor_stop();
            __delay_ms(200);

            motor_forward();
            __delay_ms(200);

            motor_stop();

            startup_mode = 0;
        }

        continue;
    }

    // ================= MODE: STARTUP DISPLAY =================
    if(mode == MODE_STARTUP)
    {
        static unsigned char toggle = 0;

        voltage2 = getVoltage(2);

        if(voltage2 < 2.0)   // ? LOW BATTERY
        {
            if(toggle == 0)
            {
                // OP.BT
                ts1 = SS1_o;
                ts2 = SS2_P;
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
            __delay_ms(500);
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
        current_limit = getAmps(0);
        splitFloat(current_limit, &ts1, &ts2, &ts3, &ts4);
        outputOk = 1;
    }

    // ================= VOLTAGE SELECT BUTTONS =================
    if(mode == MODE_SET_VOLTAGE)
    {
        // UP
        if(last_up == 0 && BTN_UP == 1)
        {
            __delay_ms(20);
            if(BTN_UP == 1)
            {
                v_index++;
                if(v_index > 2) v_index = 0;
            }
        }
        last_up = BTN_UP;

        // DOWN
        if(last_down == 0 && BTN_DOWN == 1)
        {
            __delay_ms(20);
            if(BTN_DOWN == 1)
            {
                if(v_index == 0) 
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

        outputOk = 1;
    }

    
if(system_done == 1)
{
    motor_stop();
    continue;
}
    // ================= MODE: RUN =================
    if(mode == MODE_RUN)
    {
        duty = 3;

        voltage2 = getVoltage(2);

        // -------- RELAY CONTROL --------
        static unsigned char relay_state = 0;

        if(getVoltage(2) > 3.0)
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
        //RELAY = 1;

        // -------- VOLTAGE CUTOFF --------
        if(voltage2 >= selected_voltage)
        {
            motor_stop();
            //continue;
            system_done = 1; 
        }

        // -------- LIMIT SWITCH --------
        if(prev_limit == 1 && LIMIT == 0)
        {
            __delay_ms(20);

            if(LIMIT == 0)
            {
                if(stage == 0 && direction == 1)
                {
                    motor_stop();
                    __delay_ms(100);

                    motor_reverse();
                    __delay_ms(200);

                    motor_stop();
                    __delay_ms(500);

                    motor_reverse();

                    stage = 1;
                    limit_active = 1;
                }
                else if(stage == 1 && direction == 2)
                {
                    motor_stop();
                    __delay_ms(100);

                    motor_forward();
                    __delay_ms(200);

                    motor_stop();

                    stage = 2;
                    system_done = 1;
                    limit_active = 0;
                }
            }
        }
        prev_limit = LIMIT;

        // -------- MOTOR CONTROL --------
        if(LIMIT == 1 && limit_active == 0 && system_done == 0 && RELAY == 1)
        {
           
            tar_vol = getAmps(0); 
            ref_vol = getAmps(1); 
            float error = (float)tar_vol - (float)ref_vol; 
           static unsigned char stopped = 0;

            if(error < TOLERANCE && error > -TOLERANCE)
            {
                motor_stop();
                stopped = 1;
            }
            else if(stopped == 0)
            {
                if(error > 0)
                    motor_forward();
                else
                    motor_reverse();
            }
            
        }
        
        voltage2 = getVoltage(2);
        splitFloat(voltage2 ,&ts1 ,&ts2 ,&ts3 ,&ts4);
        outputOk = 1;
        
        // -------- FINAL STOP --------
        if(system_done == 1)
        {
            motor_stop();
        }
    }
}
}







