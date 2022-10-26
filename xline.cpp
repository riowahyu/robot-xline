//
//  xline.cpp
//  
//
//  Created by rodik wahyu indrawan on 21/06/2016.
//  rodikwahyuindrawan@gmail.com
//

#include "xline.h"
#include "Dynamixel_Serial.h"
#include "Wire.h"
#include "PWM.h"
#include "Servo.h"
#include "Arduino.h"

int tnos=0;
int FMOTOR =0, FG=0;
int KI = 5;
char line_color = black;
char sensor_direction = front;
char flagdirsensor = front;
int ID_DXL = 4;
int ID_DXR = 5;
int FLAG_WIDE_LINE = 2;
char kp = 30;
int dkp = 10, dsetdelay=0,dsetbreak=30,dsetturn=5;
int dturnon = 5,dturnoff = 2;
char data_mirror = off;

void motor_type(int tp)
{
    FMOTOR = tp;
    if(FMOTOR == MRB) KI = 20;
    else if (FMOTOR == MGR) KI = 5;
    else if (FMOTOR == MDX) KI = 5;
}
void mirror(char dt)
{
    data_mirror = dt;
}
int convertpwm(int din)
{
    if (din > 20)din = 20;
    else if (din < -20)din = -20;
    
    return map(din, -20, 20, -255, 255);
}
int xconvertpwm(int din)
{
    if (din > 20)din = 20;
    else if (din < -20)din = -20;
    
    return map(din, -20, 20, -500, 500);
}
//---------------------------------------------------------------------------------

#define ID_PS2 21

char pb_up    = 0;
char pb_down  = 0;
char pb_left  = 0;
char pb_right = 0;
char pb_select = 0;
char pb_l1 = 0;
char pb_l2 = 0;
char pb_l3 = 0;

char pb_triangle = 0;
char pb_cross = 0;
char pb_circle = 0;
char pb_square = 0;
char pb_start = 0;
char pb_r1 = 0;
char pb_r2 = 0;
char pb_r3 = 0;

int  analog_lx = 0;
int  analog_ly = 0;
int  analog_rx = 0;
int  analog_ry = 0;

int speedturn = 0;

float sudut_r = 0;
float radial = 0;
float throttle = 0;
void read_psx() {
    Wire.requestFrom(ID_PS2, 6);    // request 6 bytes from slave device #8
    char count = 0;
    int i2c_rx_ps2[6];//dleft,dright,leftjoyx,leftjoyy,rightjoyx,rightjoyy
    
    while (Wire.available()) { // slave may send less than requested
        i2c_rx_ps2[count] = Wire.read(); // receive a byte as character
        count++;
    }
    pb_select = bitRead(i2c_rx_ps2[0], 0);
    pb_right  = bitRead(i2c_rx_ps2[0], 1);
    pb_left   = bitRead(i2c_rx_ps2[0], 2);
    pb_down   = bitRead(i2c_rx_ps2[0], 3);
    pb_up     = bitRead(i2c_rx_ps2[0], 4);
    pb_l1     = bitRead(i2c_rx_ps2[0], 5);
    pb_l2     = bitRead(i2c_rx_ps2[0], 6);
    pb_l3     = bitRead(i2c_rx_ps2[0], 7);
    
    pb_start    = bitRead(i2c_rx_ps2[1], 0);
    pb_square   = bitRead(i2c_rx_ps2[1], 1);
    pb_circle   = bitRead(i2c_rx_ps2[1], 2);
    pb_cross    = bitRead(i2c_rx_ps2[1], 3);
    pb_triangle = bitRead(i2c_rx_ps2[1], 4);
    pb_r1       = bitRead(i2c_rx_ps2[1], 5);
    pb_r2       = bitRead(i2c_rx_ps2[1], 6);
    pb_r3       = bitRead(i2c_rx_ps2[1], 7);
    
    
    
    analog_lx   = 128 - i2c_rx_ps2[2];
    analog_ly   = 128 - i2c_rx_ps2[3];
    
    
    analog_rx   = 128 - i2c_rx_ps2[4];
    analog_ry   = 127 - i2c_rx_ps2[5];
    
    if (analog_rx <= 15 && analog_rx >= -15) analog_rx = 0;
    if (analog_ry <= 15 && analog_ry >= -15) analog_ry = 0;

    if (analog_lx <= 15 && analog_lx >= -15) analog_lx = 0;
    if (analog_ly <= 15 && analog_ly >= -15) analog_ly = 0;
    
}
float dnos=0;
void navigate_maneuver_5(int dL, int dR)
{
    read_psx();
    
    if(pb_l1 && !pb_l2) { dmotor(0,0);sensor(front);  buzzer(2,100,100);}
    else if(!pb_l1 && pb_l2) {dmotor(0,0);sensor(rear);buzzer(2,100,100);}
    
    
    if(pb_r1)dnos = dnos + 0.1;
    else     dnos =0;
    
    if(dnos >= 5) dnos = 5;
    
    dL = dL + dnos;
    dR = dR + dnos;
    
    
    if( pb_up && !pb_down && !pb_left && !pb_right){dmotor(dL,dR);}
    else if(!pb_up && pb_down && !pb_left && !pb_right){dmotor(-dL,-dR);}
    else if(!pb_up && !pb_down && pb_left && !pb_right){dmotor(-dL,dR);}
    else if(!pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,-dR);}
    
    else if( pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,0);}
    else if( pb_up && !pb_down && pb_left && !pb_right){dmotor(0,dR);}
    
    else if( !pb_up &&  pb_down && pb_left && !pb_right){dmotor(-dL,0);}
    else if( !pb_up && pb_down && !pb_left &&  pb_right){dmotor(0,-dR);}
    
     else if (analog_ly != 0 || analog_ry != 0)
     {
     if (analog_ly <= -20)  dL = analog_ly - (dnos * 12);
     else if (analog_ly >= 20)                dL = analog_ly + (dnos * 12);
     else dL=0;
     
     if (analog_ry <= -20)  dR = analog_ry - (dnos * 12);
     else if (analog_ry >= 20)                dR = analog_ry + (dnos * 12);
     else dR=0;
     d_motor(dL,dR);
     
     }
    else dmotor(0,0);
    
    
    
    
}


void navigate_maneuver_4(int dL, int dR)
{
    read_psx();
    
    if(pb_l1 && !pb_l2) { dmotor(0,0);sensor(front);  buzzer(2,100,100);}
    else if(!pb_l1 && pb_l2) {dmotor(0,0);sensor(rear);buzzer(2,100,100);}

    
    
    if( pb_up && !pb_down && !pb_left && !pb_right){dmotor(dL,dR);}
    else if(!pb_up && pb_down && !pb_left && !pb_right){dmotor(-dL,-dR);}
    else if(!pb_up && !pb_down && pb_left && !pb_right){dmotor(-dL,dR);}
    else if(!pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,-dR);}
    
    else if( pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,0);}
    else if( pb_up && !pb_down && pb_left && !pb_right){dmotor(0,dR);}
    
    else if( !pb_up &&  pb_down && pb_left && !pb_right){dmotor(-dL,0);}
    else if( !pb_up && pb_down && !pb_left &&  pb_right){dmotor(0,-dR);}
    
    else dmotor(0,0);
    
    
    
    
}

void navigate_maneuver_2(int dL, int dR)
{
    read_psx();
    
    if(pb_r1)dnos = dnos + 0.1;
    else     dnos =0;
    
    if(dnos >= 5) dnos = 5;
    
    dL = dL + dnos;
    dR = dR + dnos;
    
    
    if( pb_up && !pb_down && !pb_left && !pb_right){dmotor(dL,dR);}
    else if(!pb_up && pb_down && !pb_left && !pb_right){dmotor(-dL,-dR);}
    else if(!pb_up && !pb_down && pb_left && !pb_right){dmotor(-dL,dR);}
    else if(!pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,-dR);}
    
    else if( pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,0);}
    else if( pb_up && !pb_down && pb_left && !pb_right){dmotor(0,dR);}
    
    else if( !pb_up &&  pb_down && pb_left && !pb_right){dmotor(-dL,0);}
    else if( !pb_up && pb_down && !pb_left &&  pb_right){dmotor(0,-dR);}
    
    else if (analog_ly != 0 || analog_ry != 0)
    {
        if (analog_ly <= -20)  dL = analog_ly - (dnos * 12);
        else if (analog_ly >= 20)                dL = analog_ly + (dnos * 12);
        else dL=0;
        
        if (analog_ry <= -20)  dR = analog_ry - (dnos * 12);
        else if (analog_ry >= 20)                dR = analog_ry + (dnos * 12);
        else dR=0;
        d_motor(dL,dR);
        
    }
    else dmotor(0,0);
    
    
    
    
}

void navigate_maneuver_1(int dL, int dR)
{
    read_psx();
    
    if( pb_up && !pb_down && !pb_left && !pb_right){dmotor(dL,dR);}
    else if(!pb_up && pb_down && !pb_left && !pb_right){dmotor(-dL,-dR);}
    else if(!pb_up && !pb_down && pb_left && !pb_right){dmotor(-dL,dR);}
    else if(!pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,-dR);}
    
    else if( pb_up && !pb_down && !pb_left && pb_right){dmotor(dL,0);}
    else if( pb_up && !pb_down && pb_left && !pb_right){dmotor(0,dR);}
    
    else if( !pb_up &&  pb_down && pb_left && !pb_right){dmotor(-dL,0);}
    else if( !pb_up && pb_down && !pb_left &&  pb_right){dmotor(0,-dR);}
    
    else dmotor(0,0);


}
void sequencial_command_1(int dLF, int dRF, int dTF,
                          int dLR, int dRR,int dTR,
                          int dLB, int dRB,int dTB,
                          int dLL, int dRL,int dTL)
{
    read_psx();
    
    if( pb_up && !pb_down && !pb_left && !pb_right)     {buzzer(1,50,50);tmotor(dLF,dRF,dTF);buzzer(2,50,50);}
    else if(!pb_up && pb_down && !pb_left && !pb_right) {buzzer(1,50,50);tmotor(dLB,dRB,dTB);buzzer(2,50,50);}
    else if(!pb_up && !pb_down && pb_left && !pb_right) {buzzer(1,50,50);tmotor(dLL,dRL,dTL);buzzer(2,50,50);}
    else if(!pb_up && !pb_down && !pb_left && pb_right) {buzzer(1,50,50);tmotor(dLR,dRR,dTR);buzzer(2,50,50);}
    
    
}
#define scf 1
#define scr 2
#define scb 3
#define scl 4
#define scstart 5
#define screset 6

int di=0;
int sc_dt[51]={0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0};
void sequencial_command_2(int dLF, int dRF, int dTF,
                          int dLR, int dRR,int dTR,
                          int dLB, int dRB,int dTB,
                          int dLL, int dRL,int dTL)
{
   
    int pb_all = 0;
    int sc_dd = 0;
    while(!pb_all)
    {
        read_psx();
       
        pb_all = ( (pb_circle << 5) | (pb_cross << 4) |(pb_up << 3) | (pb_right << 2) | (pb_down << 1) | (pb_left << 0));
        switch(pb_all)
        {
            case 0b000001 : sc_dd = scl; break;
            case 0b000010 : sc_dd = scb; break;
            case 0b000100 : sc_dd = scr; break;
            case 0b001000 : sc_dd = scf; break;
            case 0b010000 : sc_dd = screset; break;
            case 0b100000 : sc_dd = scstart; break;
        }
        
    }
    while(pb_all){
        read_psx();
        pb_all = ( (pb_circle << 5) | (pb_cross << 4) | (pb_up << 3) | (pb_right << 2) | (pb_down << 1) | (pb_left << 0));
    }
 //   Serial.print(di);Serial.print(" = ");
    
    buzzer(1,50,50);
    if(sc_dd == scstart)
    {
        buzzer(2,300,300);
        for(int i=0;i <= di;i++)
        {
            switch (sc_dt[i])
            {
                case scf: tmotor(dLF, dRF, dTF); break;
                case scr: tmotor(dLR, dRR, dTR); break;
                case scb: tmotor(dLB, dRB, dTB); break;
                case scl: tmotor(dLL, dRL, dTL); break;
            }
            buzzer(2,300,300);
            //Serial.print(sc_dt[i]);
            //Serial.print(" run ");
        }
    }
    else if( sc_dd == screset)
    {
        for(int i=0;i<=50;i++)
        {
            sc_dt[i] = 0;
        }
        di=0;
        buzzer(3,50,50);
    }
    else
    {
        sc_dt[di] = sc_dd;
        di++;
        while(di >= 50 ){buzzer(1,50,100);}

    }
 /*   for(int i=0;i<=50;i++)
    {
        Serial.print(sc_dt[i]);
        Serial.print(" ");
    }
    
    Serial.println("-------------");
   */
    
}
void linemaze_command(int dLM, int dLS,int dLBR,
                      int dRM, int dRS,int dRBR,
                      int dLL, int dLR,int dLB,
                      int dRL, int dRR,int dRB)
{
    
    
    int pb_all = 0;
    int sc_dd = 0;
    while(!pb_all)
    {
        read_psx();
        
        pb_all = ( (pb_circle << 5) | (pb_cross << 4) |(pb_up << 3) | (pb_right << 2) | (pb_triangle << 1) | (pb_left << 0));
        switch(pb_all)
        {
            case 0b000001 : sc_dd = scl; break;
            case 0b000010 : sc_dd = scb; break; //triangle
            case 0b000100 : sc_dd = scr; break;
            case 0b001000 : sc_dd = scf; break;
            case 0b010000 : sc_dd = screset; break;
            case 0b100000 : sc_dd = scstart; break;
        }
        
    }
    while(pb_all){
        read_psx();
        pb_all = ( (pb_circle << 5) | (pb_cross << 4) | (pb_up << 3) | (pb_right << 2) | (pb_triangle << 1) | (pb_left << 0));
    }
    //   Serial.print(di);Serial.print(" = ");
    
    buzzer(1,50,50);
    if(sc_dd == scstart)
    {
        buzzer(2,300,300);
        for(int i=0;i <= di;i++)
        {
            switch (sc_dt[i])
            {
                case scf: line(dLM, dLS, dLBR); break;
                case scr: right5(dRL, dRR, dRB); break;
                case scb: line(dRM, dRS, dRBR); break;//triangle
                case scl: left4(dLL, dLR, dLB); break;
            }
            buzzer(2,300,300);
            delay(500);
            //Serial.print(sc_dt[i]);
            //Serial.print(" run ");
        }
    }
    else if( sc_dd == screset)
    {
        for(int i=0;i<=50;i++)
        {
            sc_dt[i] = 0;
        }
        di=0;
        buzzer(3,50,50);
    }
    else
    {
        sc_dt[di] = sc_dd;
        di++;
        while(di >= 50 ){buzzer(1,50,100);}
        
    }
    /*   for(int i=0;i<=50;i++)
     {
     Serial.print(sc_dt[i]);
     Serial.print(" ");
     }
     
     Serial.println("-------------");
     */
    
}

void display_psx()
{
    
    /* Serial.print(i2c_rx[0]);Serial.print(" ");
     Serial.print(i2c_rx[1]);Serial.print(" ");
     Serial.print(i2c_rx[2]);Serial.print(" ");
     Serial.print(i2c_rx[3]);Serial.print(" ");
     Serial.print(i2c_rx[4]);Serial.print(" ");
     Serial.print(i2c_rx[5]);Serial.print("----");
     
     */
    Serial.print(pb_select, DEC); Serial.print(" ");
    Serial.print(pb_right, DEC); Serial.print(" ");
    Serial.print(pb_left, DEC); Serial.print(" ");
    Serial.print(pb_down, DEC); Serial.print(" ");
    Serial.print(pb_up, DEC); Serial.print(" ");
    Serial.print(pb_l1, DEC); Serial.print(" ");
    Serial.print(pb_l2, DEC); Serial.print(" ");
    Serial.print(pb_l3, DEC); Serial.print("---");
    Serial.print(pb_start, DEC); Serial.print(" ");
    Serial.print(pb_square, DEC); Serial.print(" ");
    Serial.print(pb_circle, DEC); Serial.print(" ");
    Serial.print(pb_cross, DEC); Serial.print(" ");
    Serial.print(pb_triangle, DEC); Serial.print(" ");
    Serial.print(pb_r1, DEC); Serial.print(" ");
    Serial.print(pb_r2, DEC); Serial.print(" ");
    Serial.print(pb_r3, DEC); Serial.print(" ---  ");
    Serial.print(analog_lx); Serial.print(" ");
    Serial.print(analog_ly); Serial.print(" ");
    Serial.print(analog_rx); Serial.print(" ");
    Serial.print(analog_ry); Serial.print(" --- ");
    //Serial.print(sudut_l);Serial.print(" ");
    // Serial.print(sudut_r);Serial.print(" ");
    // Serial.print(radial);Serial.print(" ");
    Serial.println(" ");
    
}
//-----------------------------------------------

int port_motor_left_a = 3;
int port_motor_left_b = 11;
int port_motor_right_a = 5;
int port_motor_right_b = 6;

void motor_init() {
    pinMode(port_motor_left_a, OUTPUT);
    pinMode(port_motor_left_b, OUTPUT);
    pinMode(port_motor_right_a, OUTPUT);
    pinMode(port_motor_right_b, OUTPUT);
    
  //  pinMode(5, OUTPUT);
  //  pinMode(6, OUTPUT);
    
    TCCR0A = 0; //reset the register
    TCCR0B = 0; //reset tthe register
    TCCR0A = 0b11110011; // fast pwm mode
    TCCR0B = 0b00000011; // prescaler 64
    OCR0A = 0; //duty cycle for pin 6
    OCR0B = 0; //duty cycle for pin 5
    
  //  pinMode(3, OUTPUT);
  //  pinMode(11, OUTPUT);
    
    TCCR2A = 0; //reset the register
    TCCR2B = 0; //reset tthe register
    TCCR2A = 0b11110011; // fast pwm mode
    TCCR2B = 0b00000011; // prescaler 64
    OCR2A = 0; //duty cycle for pin 11
    OCR2B = 0; //duty cycle for pin 3
    
  //------------------
    //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
 //   TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
  //  TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
    
    
    
    //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
   // TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
   // TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
    //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
}
void motor( int dl, int dr, int dtime){
    dl = convertpwm(dl); dr = convertpwm(dr);
    d_motor(dl,dr);
    delay(dtime);
    d_motor(0,0);
}
void tmotor( int dl, int dr, int dtime){
    dl = convertpwm(dl); dr = convertpwm(dr);
    d_motor(dl,dr);
    delay(dtime);
    d_motor(0,0);
}
void dmotor( int dl, int dr){
    dl = convertpwm(dl); dr = convertpwm(dr);
    d_motor(dl,dr);
}



void d_motor(int dl, int dr)
{
   if(sensor_direction == front)
   {
        if (dl < 0)
        {
            OCR2A = -dl; //duty cycle for pin 11
            OCR2B = 0; //duty cycle for pin 3
        }
        else if (dl > 0)
        {
            OCR2A = 0; //duty cycle for pin 11
            OCR2B = dl; //duty cycle for pin 3
        }
        else
        {
            OCR2A = 0; //duty cycle for pin 11
            OCR2B = 0; //duty cycle for pin 3
        }
       
        if (dr < 0)
        {
            OCR0A = -dr; //duty cycle for pin 6
            OCR0B = 0; //duty cycle for pin 5
        }
        else if (dr > 0)
        {
            OCR0A = 0; //duty cycle for pin 6
            OCR0B = dr; //duty cycle for pin 5
        }
        else
        {
            OCR0A = 0; //duty cycle for pin 6
            OCR0B = 0; //duty cycle for pin 5
        }
   }
    else
    {
        if (dl < 0)
        {
            OCR0A =  0; //duty cycle for pin 11
            OCR0B = -dl; //duty cycle for pin 3
        }
        else if (dl > 0)
        {
            OCR0A = dl; //duty cycle for pin 11
            OCR0B = 0; //duty cycle for pin 3
        }
        else
        {
            OCR0A = 0; //duty cycle for pin 11
            OCR0B = 0; //duty cycle for pin 3
        }
        
        if (dr < 0)
        {
            OCR2A = 0; //duty cycle for pin 6
            OCR2B = -dr; //duty cycle for pin 5
        }
        else if (dr > 0)
        {
            OCR2A = dr; //duty cycle for pin 6
            OCR2B = 0; //duty cycle for pin 5
        }
        else
        {
            OCR2A = 0; //duty cycle for pin 6
            OCR2B = 0; //duty cycle for pin 5
        }
    }
    
    /*
    
    if (dl < 0)
    {
        digitalWrite(port_motor_left_b, LOW);
        analogWrite(port_motor_left_a, abs(dl));
        
    }
    else if (dl > 0)
    {
        digitalWrite(port_motor_left_a, LOW);
        analogWrite(port_motor_left_b, abs(dl));
        
    }
    else
    {
        
        digitalWrite(port_motor_left_a, LOW);
        digitalWrite(port_motor_left_b, LOW);
        
    }

    if (dr < 0)
    {
        digitalWrite(port_motor_right_b, LOW);
        analogWrite(port_motor_right_a, abs(dr));
        
    }
    else if (dr > 0)
    {
        digitalWrite(port_motor_right_a, LOW);
        analogWrite(port_motor_right_b, abs(dr));
        
    }
    else
    {
        digitalWrite(port_motor_right_a, LOW);
        digitalWrite(port_motor_right_b, LOW);
    }
    */
    /*
     digitalWrite(port_motor_left_a, LOW);
     digitalWrite(port_motor_left_b, LOW);
     if (dl < 0)       analogWrite(port_motor_left_a, abs(dl));
     else if (dl > 0)  analogWrite(port_motor_left_b, abs(dl));
     digitalWrite(port_motor_right_a, LOW);
     digitalWrite(port_motor_right_b, LOW);
     if (dr < 0)       analogWrite(port_motor_right_a, abs(dr));
     else if (dr > 0)  analogWrite(port_motor_right_b, abs(dr));
     */
}

void motor_break_left()
{
   // digitalWrite(port_motor_left_a, LOW);
   // digitalWrite(port_motor_left_b, LOW);
}
void motor_break_right()
{
  //  digitalWrite(port_motor_right_a, LOW);
   // digitalWrite(port_motor_right_b, LOW);
}
//-----------------------------------------------




// sl -> sensor line
char sl_front_1()
{
   // return sl_front_2();
    
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
    digitalWrite(port_channel_c, HIGH);
    digitalWrite(port_channel_b, LOW);
    digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    
}
char sl_front_2()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black)return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black)return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_front_3()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black)return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black)return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_front_4()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_front_5()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_front_6()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_front_7()
{
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_front_8()
{
 //  return sl_front_7();
 
    if (sensor_direction == front) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
   
}

char sl_rear_1()
{
   // return sl_rear_2();
    
    if (sensor_direction == rear) {
    digitalWrite(port_channel_c, LOW);
    digitalWrite(port_channel_b, HIGH);
    digitalWrite(port_channel_a, LOW);
    if (line_color == black) return digitalRead(pin_sensor_line_a);
    else                     return !digitalRead(pin_sensor_line_a);
}
else                          {
    digitalWrite(port_channel_c, HIGH);
    digitalWrite(port_channel_b, LOW);
    digitalWrite(port_channel_a, HIGH);
    if (line_color == black) return digitalRead(pin_sensor_line_b);
    else                     return !digitalRead(pin_sensor_line_b);
}
    
}
char sl_rear_2()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_rear_3()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_rear_4()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
}
char sl_rear_5()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_rear_6()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_rear_7()
{
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, LOW);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
}
char sl_rear_8()
{
    //return sl_rear_7();
    
    if (sensor_direction == rear) {
        digitalWrite(port_channel_c, LOW);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, LOW);
        if (line_color == black) return digitalRead(pin_sensor_line_b);
        else                     return !digitalRead(pin_sensor_line_b);
    }
    else                          {
        digitalWrite(port_channel_c, HIGH);
        digitalWrite(port_channel_b, HIGH);
        digitalWrite(port_channel_a, HIGH);
        if (line_color == black) return digitalRead(pin_sensor_line_a);
        else                     return !digitalRead(pin_sensor_line_a);
    }
    
}
void stop_end(){
    d_motor(0,0);
    digitalWrite(port_enable_sensor,LOW);
    dxmotor(0,0);
    while(1);
}

//--------------------------------------------
char sl_front[8];
int sl_front_all = 0;
void read_sl_front()
{
    if (line_color == black) {
        if (sensor_direction == front) {
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[1] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[2] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[0] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[3] = digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[6] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[4] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[7] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[5] = digitalRead(pin_sensor_line_b);
        }
        else {
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[4] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[5] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[6] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[7] = digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[3] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[0] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[2] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[1] = digitalRead(pin_sensor_line_b);
        }
    }
    else {
        if (sensor_direction == front) {
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[1] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[2] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[0] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[3] = !digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[6] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[4] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[7] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[5] = !digitalRead(pin_sensor_line_b);
        }
        else {
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[4] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[5] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[6] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[7] = !digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_front[3] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_front[0] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_front[2] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_front[1] = !digitalRead(pin_sensor_line_b);
        }
    }
    // edit kron
    //---------------------------
  //  sl_front[7] = sl_front[6];
   // sl_front[0] = sl_front[1];
    //---------------------------

    sl_front_all = (sl_front[7] << 0) | (sl_front[6] << 1) | (sl_front[5] << 2) | (sl_front[4] << 3) | (sl_front[3] << 4) | (sl_front[2] << 5) | (sl_front[1] << 6) | (sl_front[0] << 7);
}

char sl_rear[8];
int sl_rear_all = 0;
void read_sl_rear()
{
    if (line_color == black) {
        if (sensor_direction == rear) {
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[4] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[5] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[6] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[7] = digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[3] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[0] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[2] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[1] = digitalRead(pin_sensor_line_b);

        }
        else {
            
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[1] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[2] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[0] = digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[3] = digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[6] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[4] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[7] = digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[5] = digitalRead(pin_sensor_line_b);
        }
    }
    else {
        if (sensor_direction == rear) {
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[4] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[5] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[6] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[7] = !digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[3] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[0] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[2] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, HIGH); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[1] = !digitalRead(pin_sensor_line_b);
        }
        else {
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[1] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[2] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[0] = !digitalRead(pin_sensor_line_a);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[3] = !digitalRead(pin_sensor_line_a);
            
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, LOW);  sl_rear[6] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, LOW);  digitalWrite(port_channel_a, HIGH); sl_rear[4] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, LOW);  sl_rear[7] = !digitalRead(pin_sensor_line_b);
            digitalWrite(port_channel_c, LOW); digitalWrite(port_channel_b, HIGH); digitalWrite(port_channel_a, HIGH); sl_rear[5] = !digitalRead(pin_sensor_line_b);
            
            
        }
    }
    
    // edit kron
    //---------------------------
   // sl_rear[7] = sl_rear[6];
  //  sl_rear[0] = sl_rear[1];
    //---------------------------
        
    sl_rear_all = (sl_rear[7] << 0) | (sl_rear[6] << 1) | (sl_rear[5] << 2) | (sl_rear[4] << 3) | (sl_rear[3] << 4) | (sl_rear[2] << 5) | (sl_rear[1] << 6) | (sl_rear[0] << 7);
}


//--------------------------------------------

#define dxl_id 0x01
#define dxl_controlpin 7
#define dxl_baudrate 1000000
void dxl_init(int did, int mode, int cwlimit,int ccwlimit)
{
    Dynamixel.begin(dxl_baudrate, dxl_controlpin);
    Dynamixel.setMode(did, mode, cwlimit, ccwlimit);
    
}
void dxlservo(int did, int dpos, int dspeed)
{
    Dynamixel.servo(did,dpos,dspeed);delay(1);
    Dynamixel.servo(did,dpos,dspeed);delay(1);
    Dynamixel.servo(did,dpos,dspeed);delay(1);
    Dynamixel.servo(did,dpos,dspeed);delay(1);
}
void dxlsetid(int did)
{
    Dynamixel.begin(dxl_baudrate, dxl_controlpin);
    Dynamixel.setID(BROADCAST_ID,did);
}
void dxlwheel(int did, int dspeed)
{
    if(dspeed < 0 )      Dynamixel.wheel(did,LEFT,-dspeed);
    else if(dspeed > 0 ) Dynamixel.wheel(did,RIGHT,dspeed);
    else                 Dynamixel.wheel(did,RIGHT,0);
    
}
void xmotor(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    dxmotor(dl,dr);
    delay(dtime);
    dxmotor(0,0);
}
void dxmotor(int dl, int dr)
{
  //  for(int i=0;i<2;i++)
    {   dxlwheel(ID_DXL, -dl);delay(1);
        dxlwheel(ID_DXR, dr);delay(1);
    }
}
//--------------------------------------------

void init_srf05(){
    
}
void init_gp2d12(){
    
}
void init_gp2d120(){
    
}

//-----------------------------------------------

char flagtracemode = 1;
void tracemode(char ddir)
{
    flagtracemode = ddir;
}

void ss()
{
    if(flagdirsensor == front)          flagdirsensor = rear;
    else if (flagdirsensor == rear)     flagdirsensor = front;
    
    sensor(flagdirsensor);
    
}
void sensor(char dir)
{
    if (dir == front) {
        sensor_direction = front;
        port_motor_left_a = 3;
        port_motor_left_b = 11;
        port_motor_right_a = 5;
        port_motor_right_b = 6;
        ID_DXL = 4;
        ID_DXR = 5;
        flagdirsensor = front;
    }
    else if (dir == rear) {
        sensor_direction = rear;
        port_motor_left_a = 6;
        port_motor_left_b = 5;
        port_motor_right_a = 11;
        port_motor_right_b = 3;
        
        ID_DXL = 5;
        ID_DXR = 4;
        flagdirsensor = rear;
    }
}
void linecolor(char col)
{
    if (col == black)        line_color = black;
    else if (col == white)   line_color = white;
}
//--------------------------------------------
#define port_buzzer 13

void buzzer(int num, int ton, int toff)
{
    for (int i = 0; i < num ; i++)
    {
        digitalWrite(port_buzzer, HIGH);
        delay(ton);
        digitalWrite(port_buzzer, LOW);
        delay(toff);
    }
    
}
//--------------------------------------------

//--------------------------------------------
int map_pwm(int din)
{
    if (din > 20)din = 20; else if (din < -20)din = -20;
    return map(din, -20, 20, -255, 255);
}
//---------------------------------------------

void drive_line(char dkd,char dsd,char dsb )
{   dkp = dkd;
    dsetdelay = dsd;
    dsetbreak= dsb;
}
void xdrive_line(char dkd,char dsd )
{   dkp = dkd;
    dsetdelay = dsd;
}

void linewide(int dw)
{
    FLAG_WIDE_LINE = dw;
    
}

float Steering = 0;
int tspeed =100;
void line_scan_front(int dspeed)
{
    signed int dErr;
    static signed char Err, LastErr;
    static float iErr;
    int uL, uR,kd = 50;
    static char X;
    
    read_sl_front();
    
    if (sl_front[7] == 1 )X = 1; if (sl_front[0] == 1 )X = 0;
    if(sl_front[3] == 1 || sl_front[4] == 1)
    {
        tnos+=3;
        
        if (sl_front[3] && sl_front[4] ) {iErr=0;Err = 0; tspeed = 250;}
        else if (sl_front[3]  ){Err = -1; tspeed = 240;}
        else {Err = 1; tspeed = 240;}
    }
    else if(sl_front[2] == 1 || sl_front[5] == 1) { tnos=0;tspeed = 200;if (sl_front[2]) Err = -3; else Err = 3;}
    else if(sl_front[1] == 1 || sl_front[6] == 1) { tnos=0;tspeed = 180;if (sl_front[1]) Err = -4; else Err = 4;}
    else if(sl_front[0] == 1 || sl_front[7] == 1) { tnos=0;tspeed = 160;if (sl_front[0]) Err = -5; else Err = 5;}
    else
    {
        tnos=0;
        d_motor(0, 0);delayMicroseconds(10);
        d_motor(-dspeed, -dspeed);
        delayMicroseconds(dsetbreak);
        tspeed =250;
        if (!X) {
            Err = -35;
            d_motor(0, uR);
        }
        else    {
            Err =  35;
            d_motor(uL, 0);
        }
    }
    
    if(flagtracemode == ll)  {
        if((sl_front[1] == 1) || (sl_front[2] == 1))
        {   Err = -7;
        }
    }
    else if(flagtracemode == rr ) {
        if((sl_front[5] == 1) || (sl_front[6] == 1))
        {   Err = 7;
            
        }
    }
    if(dspeed > tspeed) dspeed = tspeed;
    //dspeed += (tnos);
    dErr = Err - LastErr;
    LastErr = Err;
    iErr += Err;
    
    if( iErr > 255)  iErr =255;
    else if( iErr < -255)  iErr =-255;
    
    if(dspeed >= 150 ){kd = 90;}
    
    
    Steering = (float) Err * dkp + (float)dErr*kd  + (float)iErr/KI ;//5
    
    if (Steering > 500  )    Steering = 500;
    else if (Steering < -500) Steering = -500;
    uL = (int)(dspeed + Steering);
    uR = (int)(dspeed - Steering);
    
    if(uL > 255) uL =255;
    else if(uL < -255) uL =-255;
    
    if(uR > 255) uR =255;
    else if(uR < -255) uR =-255;
    d_motor(uL, uR);
   
}

void centerfront()
{
    int i=0;
    while(i<2)
    {
    read_sl_front();
    switch (sl_front_all)
    {
        case 0b10000000: d_motor(-80,80); break;
        case 0b11000000: d_motor(-70,70); break;
        case 0b01000000: d_motor(-70,70); break;
        case 0b01100000: d_motor(-60,60); break;
        case 0b00100000: d_motor(-60,60); break;
        case 0b00110000: d_motor(-50,50); break;
        case 0b00010000: d_motor(-50,50); i++;break;
        case 0b00011000: d_motor(0,0); i++;break; ////
        case 0b00001000: d_motor(50,-50); i++; break;
        case 0b00001100: d_motor(50,-50); break;
        case 0b00000100: d_motor(60,-60); break;
        case 0b00000110: d_motor(60,-60); break;
        case 0b00000010: d_motor(70,-70);break;
        case 0b00000011: d_motor(70,-70);break;
        case 0b00000001: d_motor(80,-80); break;
        default :        d_motor(80,-80); break;
   
    }
    }
    d_motor(0,0);
}
void line(int dmode, int dspeed, int dbreaktime){
    if(FMOTOR == MRB )
    {
        robo_line(dmode,dspeed,dbreaktime);
    }
    else if (FMOTOR == MGR )
    {
        dspeed = convertpwm(dspeed);
        int cn =10;
        read_sl_front();
        switch (dmode)
        {
            case ff:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case fl:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case fl3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case fl4:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2] && !sl_front[3]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2] || !sl_front[3])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2] && sl_front[3]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
                
            case fr:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[6] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case fr3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case fr4:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[4] || sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[4] && !sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[4] || !sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[4] && sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[4] || sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
                }
                break;
            case sf:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sl:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sl3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sr:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[6] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sr3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
        }
        
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   }
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    }
        // else {buzzer(1, 10, 0);}//d_motor(0, 0);}
        buzzer(1, 5, 0);
        flagtracemode = cc;
    }
}
void robo_line(int dmode, int dspeed, int dbreaktime){
   dspeed = convertpwm(dspeed);
    int cn =10, FG=0;
    read_sl_front();
    switch (dmode)
    {
        case ff:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case fl:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] ){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] ){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case fl3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case fl4:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2] || !sl_front[3]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
            
        case fr:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case fr3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case fr4:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[4] ||!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front(dspeed);};
            }
            break;
        case sf:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            break;
        case sl:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1]){line_scan_front(dspeed);};
                
            }
            break;
        case sl3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){line_scan_front(dspeed);};
                
            }
            break;
        case sr:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            break;
        case sr3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};
                
            }
            break;
    }
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   }
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    }
   // else {buzzer(1, 10, 0);}//d_motor(0, 0);}
    buzzer(1, 5, 0);
    flagtracemode = cc;
    
}
void line_scan_front_p(int dspeed)
{
    signed int dErr;
    static signed char Err, LastErr;
    static float iErr;
    int uL, uR,kd = 50;
    static char X;
    
    read_sl_front();
    
    if (sl_front[7] == 1 )X = 1; if (sl_front[0] == 1 )X = 0;
    if(sl_front[3] == 1 || sl_front[4] == 1)
    {
        tnos+=3;
        
        if (sl_front[3] && sl_front[4] ) {iErr=0;Err = 0; tspeed = 250;}
        else if (sl_front[3]  ){Err = -1; tspeed = 240;}
        else {Err = 1; tspeed = 240;}
    }
    else if(sl_front[2] == 1 || sl_front[5] == 1) { tnos=0;tspeed = 200;if (sl_front[2]) Err = -3; else Err = 3;}
    else if(sl_front[1] == 1 || sl_front[6] == 1) { tnos=0;tspeed = 180;if (sl_front[1]) Err = -4; else Err = 4;}
    else if(sl_front[0] == 1 || sl_front[7] == 1) { tnos=0;tspeed = 160;if (sl_front[0]) Err = -5; else Err = 5;}
    else
    {
        tnos=0;
      /*  d_motor(0, 0);delayMicroseconds(10);
        d_motor(-dspeed, -dspeed);
        delayMicroseconds(dsetbreak);
        tspeed =250;
        if (!X) {
            Err = -35;
            d_motor(0, uR);
        }
        else    {
            Err =  35;
            d_motor(uL, 0);
        }
       */
    }
    
    if(flagtracemode == ll)  {
        if((sl_front[1] == 1) || (sl_front[2] == 1))
        {   Err = -7;
        }
    }
    else if(flagtracemode == rr ) {
        if((sl_front[5] == 1) || (sl_front[6] == 1))
        {   Err = 7;
            
        }
    }
    if(dspeed > tspeed) dspeed = tspeed;
    //dspeed += (tnos);
    dErr = Err - LastErr;
    LastErr = Err;
    iErr += Err;
    
    if( iErr > 255)  iErr =255;
    else if( iErr < -255)  iErr =-255;
    
    if(dspeed >= 150 ){kd = 90;}
    
    
    Steering = (float) Err * dkp + (float)dErr*kd  + (float)iErr/KI ;//5
    
    if (Steering > 500  )    Steering = 500;
    else if (Steering < -500) Steering = -500;
    uL = (int)(dspeed + Steering);
    uR = (int)(dspeed - Steering);
    
    if(uL > 255) uL =255;
    else if(uL < -255) uL =-255;
    
    if(uR > 255) uR =255;
    else if(uR < -255) uR =-255;
    d_motor(uL, uR);
    
}
void robo_linep(int dmode, int dspeed, int dbreaktime){
    dspeed = convertpwm(dspeed);
    int cn =10, FG=0;
    read_sl_front();
    switch (dmode)
    {
        case ff:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case fl:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] ){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] ){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case fl3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case fl4:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2] || !sl_front[3]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
            
        case fr:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case fr3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case fr4:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[4] ||!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
            }
            break;
        case sl:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1]){line_scan_front_p(dspeed);};
                
            }
            break;
        case sl3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){line_scan_front_p(dspeed);};
                
            }
            break;
        case sr:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            break;
        case sr3:
            for (int i = 0; i < 10; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            for (int i = 0; i < cn; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front_p(dspeed);};
                
            }
            break;
    }
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   }
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    }
    // else {buzzer(1, 10, 0);}//d_motor(0, 0);}
    buzzer(1, 5, 0);
    flagtracemode = cc;
    
}

void linep(int dmode, int dspeed, int dbreaktime){
    if(FMOTOR == MRB )
    {
        robo_linep(dmode,dspeed,dbreaktime);
    }
    else if (FMOTOR == MGR )
    {
        dspeed = convertpwm(dspeed);
        int cn =10;
        read_sl_front();
        switch (dmode)
        {
            case ff:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case fl:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case fl3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case fl4:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2] && !sl_front[3]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2] || !sl_front[3])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2] && sl_front[3]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
                
            case fr:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[6] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case fr3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case fr4:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[4] || sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[4] && !sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[4] || !sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[4] && sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                for (int i = 0; i < 10; i++)
                {
                    read_sl_front();
                    while(sl_front[4] || sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();d_motor(dspeed, dspeed);}//{line_scan_front_p(dspeed);};
                }
                break;
            case sl:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sl3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1] || sl_front[2]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[0] && !sl_front[1] && !sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1] || !sl_front[2])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[0] && sl_front[1] && sl_front[2]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sr:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[6] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[6] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
            case sr3:
                for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[5] || sl_front[6] || sl_front[7]){
                        line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(!sl_front[5] && !sl_front[6] && !sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                        
                    };
                    
                }
                for (int i = 0; i < cn ; i++)
                {
                    read_sl_front();
                    while(!sl_front[5] || !sl_front[6] || !sl_front[7])
                    {   line_scan_front_p(dspeed);
                        for (int i = 0; i < 80 ; i++){
                            read_sl_front();
                            if(sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                            else FG=0;
                            
                        }
                        if(FG)break;
                    };
                    
                }
                break;
        }
        
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   }
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    }
        // else {buzzer(1, 10, 0);}//d_motor(0, 0);}
        buzzer(1, 5, 0);
        flagtracemode = cc;
    }
}


void xline_scan_front(int dspeed)
{
    signed int dErr;
    static signed char Err, LastErr;
    static float iErr;
    int uL, uR,kd = 30;
    static char X;
    
    read_sl_front();
    
    /*
    if (sl_front[7] == 1 )X = 1; if (sl_front[0] == 1 )X = 0;
    if(sl_front[3] == 1 || sl_front[4] == 1)
    {
        tnos+=3;
        
        if (sl_front[3] && sl_front[4] ) {iErr=0;Err = 0; tspeed = 550;}
        else if (sl_front[3]  ){Err = -1; tspeed = 530;}
        else {Err = 1; tspeed = 530;}
    }
    else if(sl_front[2] == 1 || sl_front[5] == 1) { tnos=0;tspeed = 500;if (sl_front[2]) Err = -3; else Err = 3;}
    else if(sl_front[1] == 1 || sl_front[6] == 1) { tnos=0;tspeed = 400;if (sl_front[1]) Err = -4; else Err = 4;}
    else if(sl_front[0] == 1 || sl_front[7] == 1) { tnos=0;tspeed = 350;if (sl_front[0]) Err = -5; else Err = 5;}
    else
    {
        tnos=0;
        //dxmotor(0, 0);//delayMicroseconds(3);
        dxmotor(-dspeed/4, -dspeed/4);
        delayMicroseconds(dsetbreak);
        tspeed =500;
        if (!X) {
            Err = -50;
             //dxmotor(0, uR);
        }
        else    {
            Err =  50;
             //dxmotor(uL, 0);
        }
    }
    */
    
    if (sl_front[5] == 1 )X = 1; if (sl_front[2] == 1 )X = 0;
    if(sl_front[3] == 1 || sl_front[4] == 1)
    {
        tnos+=3;
        
        if (sl_front[3] && sl_front[4] ) {iErr=0;Err = 0; tspeed = 550;}
        else if (sl_front[3]  ){Err = -1; tspeed = 530;}
        else {Err = 1; tspeed = 530;}
    }
    else if(sl_front[2] == 1 || sl_front[5] == 1) { tnos=0;tspeed = 500;if (sl_front[2]) Err = -3; else Err = 3;}
   // else if(sl_front[1] == 1 || sl_front[6] == 1) { tnos=0;tspeed = 400;if (sl_front[1]) Err = -4; else Err = 4;}
   // else if(sl_front[0] == 1 || sl_front[7] == 1) { tnos=0;tspeed = 350;if (sl_front[0]) Err = -5; else Err = 5;}
    else
    {
        tnos=0;
        //dxmotor(0, 0);//delayMicroseconds(3);
        dxmotor(-dspeed/4, -dspeed/4);
        delayMicroseconds(dsetbreak);
        tspeed =500;
        if (!X) {
            Err = -50;
             //dxmotor(0, uR);
        }
        else    {
            Err =  50;
             //dxmotor(uL, 0);
        }
    }
    if(flagtracemode == ll)  {
        if((sl_front[1] == 1) || (sl_front[2] == 1))
        {   Err = -7;
        }
    }
    else if(flagtracemode == rr ) {
        if((sl_front[5] == 1) || (sl_front[6] == 1))
        {   Err = 7;
            
        }
    }
    if(dspeed > tspeed) dspeed = tspeed;
    //dspeed += (tnos);
    dErr = Err - LastErr;
    LastErr = Err;
    iErr += Err;
    
    if( iErr > 500)  iErr =500;
    else if( iErr < -500)  iErr =-500;
    
    if(dspeed >= 450 ){kd = 80;}
    
    
    Steering = (float) Err * dkp + (float)dErr*kd  + (float)iErr/7 ;//5
    
    if (Steering > 1000  )    Steering = 1000;
    else if (Steering < -1000) Steering = -1000;
    uL = (int)(dspeed + Steering);
    uR = (int)(dspeed - Steering);
    
    if(uL > 600) uL =600;
    else if(uL < -600) uL =-600;
    
    if(uR > 600) uR =600;
    else if(uR < -600) uR =-600;
    
    
    dxmotor(uL, uR);
    
}
void xline_scan_front3(int dspeed)
{
    
    int dErr;
    static signed int Err, LastErr;
    static signed int iErr;
    int uL, uR;
    static char X;
    
    read_sl_front();
    if (sl_front[7] == 1 )X = 1; if (sl_front[0] == 1 )X = 0;
    if(sl_front[3] == 1 || sl_front[4] == 1)
    {
        tnos++;
        if (sl_front[3] && sl_front[4] ) {Err = 0; tspeed = 650;}
        else if (sl_front[3]  ){Err = -1; tspeed = 600;}
        else {Err = 1; tspeed = 600;}
    }
    else
    {   tnos=0;
        switch (sl_front_all)
        {
            case 0b10000000: Err = -15; tspeed = 290; break;
            case 0b11000000: Err = -13; tspeed = 310; break;
            case 0b11100000: Err = -10; tspeed = 330; break;
            case 0b01100000: Err = -9; tspeed = 350; break;
            case 0b01110000: Err = -5; tspeed = 380;break;
            case 0b00110000: Err = -4; tspeed = 400; break;
            case 0b00111000: Err = -3; tspeed = 500; break;
                // case 0b00010000: Err = -1; tspeed = 500;break;
                // case 0b00011000: Err = 0;  tspeed = 550; break; ////
                // case 0b00001000: Err = 1;  tspeed = 500; break;
            case 0b00011100: Err = 3;  tspeed = 500; break;
            case 0b00001100: Err = 4;  tspeed = 400; break;
            case 0b00001110: Err = 5;  tspeed = 380; break;
            case 0b00000110: Err = 9;  tspeed = 350; break;
            case 0b00000111: Err = 10;  tspeed = 330; break;
            case 0b00000011: Err = 13;  tspeed = 310; break;
            case 0b00000001: Err = 15;  tspeed = 290; break;
                
            case 0b00000000:
                //        dxmotor(0, 0);delayMicroseconds(10);
                //       dxmotor(-dspeed, -dspeed);
                //      delayMicroseconds(10);
                tspeed =500;
                if (!X) {
                    Err = -50;
                    dxmotor(0, uR);
                }
                else    {
                    Err =  50;
                    dxmotor(uL, 0);
                }
                break;
            default : Err = LastErr; break;
        }
    }
    
    if(flagtracemode == ll)  {
        if((sl_front[1] == 1) || (sl_front[2] == 1))
        {   Err = -7;
        }
    }
    else if(flagtracemode == rr ) {
        if((sl_front[5] == 1) || (sl_front[6] == 1))
        {   Err = 7;
            
        }
    }
    
    //   if(dsetdelay){motor_break_left(); motor_break_right();delayMicroseconds(dsetdelay);}
    if(dspeed > tspeed) dspeed = tspeed;
   //  dspeed += tnos;
    dErr = Err - LastErr;
    LastErr = Err;
    iErr +=iErr;// = Err + LastErr;
    Steering = (float) Err * dkp + (float)dErr * dsetdelay + (float)iErr;
    if (Steering > 1000  )    Steering = 1000;
    else if (Steering < -1000) Steering = -1000;
    uL = (int)(dspeed + Steering);
    uR = (int)(dspeed - Steering);
    
    if(uL > 600) uL =600;
    else if(uL < -600) uL =-600;
    
    if(uR > 600) uR =600;
    else if(uR < -600) uR =-600;
    
    
    dxmotor(uL, uR);

    
}
void xcenterfront()
{
    int i=0;
    while(i<2)
    {
        read_sl_front();
        switch (sl_front_all)
        {
            case 0b10000000: dxmotor(-80,80); break;
            case 0b11000000: dxmotor(-70,70); break;
            case 0b01000000: dxmotor(-70,70); break;
            case 0b01100000: dxmotor(-60,60); break;
            case 0b00100000: dxmotor(-60,60); break;
            case 0b00110000: dxmotor(-50,50); break;
            case 0b00010000: dxmotor(-50,50); i++;break;
            case 0b00011000: dxmotor(0,0); i++;break; ////
            case 0b00001000: dxmotor(50,-50); i++; break;
            case 0b00001100: dxmotor(50,-50); break;
            case 0b00000100: dxmotor(60,-60); break;
            case 0b00000110: dxmotor(60,-60); break;
            case 0b00000010: dxmotor(70,-70);break;
            case 0b00000011: dxmotor(70,-70);break;
            case 0b00000001: dxmotor(80,-80); break;
            default :        dxmotor(80,-80); break;
                
        }
    }
    dxmotor(0,0);
}
void sline_mgr(int dmode, int dspeed, int dbreaktime){
    dspeed = convertpwm(dspeed);
    int counts = 20;
    read_sl_front();
    switch(dmode)
    {
        case s1 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[0]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[0]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                            };
                    }
                    break;
        case s2 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[1]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[1]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s3 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[2]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[2]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s4 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[3]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[3]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s5 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[4]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[4]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s6 :   for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[5]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[5]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s7 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[6]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[6]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;
        case s8 :  for (int i = 0; i < counts; i++)
                    {
                        read_sl_front();
                        while(!sl_front[7]){
                            line_scan_front(dspeed);
                            for (int i = 0; i < 80 ; i++){
                                read_sl_front();
                                if(sl_front[7]){FG=1;break;}
                                else FG=0;
                            }
                            if(FG)break;
                        };
                    }
                    break;

        case s1+s2 :    for (int i = 0; i < counts; i++)
                        {
                            read_sl_front();
                            while(!sl_front[0] || !sl_front[1]){
                                line_scan_front(dspeed);
                                for (int i = 0; i < 80 ; i++){
                                    read_sl_front();
                                    if(sl_front[0] && sl_front[1]){FG=1;break;}
                                    else FG=0;
                                }
                                if(FG)break;
                            };
                        }
                        break;
        case s2+s3 :     for (int i = 0; i < counts; i++)
                        {
                            read_sl_front();
                            while(!sl_front[1] || !sl_front[2]){
                                line_scan_front(dspeed);
                                for (int i = 0; i < 80 ; i++){
                                    read_sl_front();
                                    if(sl_front[1] && sl_front[2]){FG=1;break;}
                                    else FG=0;
                                }
                                if(FG)break;
                            };
                        }
                        break;
        case s1+s2+s3 :  for (int i = 0; i < counts; i++)
                            {
                                read_sl_front();
                                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){
                                    line_scan_front(dspeed);
                                    for (int i = 0; i < 80 ; i++){
                                        read_sl_front();
                                        if(sl_front[0] && sl_front[1] && sl_front[2]){FG=1;break;}
                                        else FG=0;
                                    }
                                    if(FG)break;
                                };
                            }
                                break;
            
        case s7+s8 :     for (int i = 0; i < counts; i++)
                        {
                            read_sl_front();
                            while(!sl_front[6] || !sl_front[7]){
                                line_scan_front(dspeed);
                                for (int i = 0; i < 80 ; i++){
                                    read_sl_front();
                                    if(sl_front[6] && sl_front[7]){FG=1;break;}
                                    else FG=0;
                                }
                                if(FG)break;
                            };
                        }
                            break;
        case s6+s7 :     for (int i = 0; i < counts; i++)
                        {
                            read_sl_front();
                            while(!sl_front[5] || !sl_front[6]){
                                line_scan_front(dspeed);
                                for (int i = 0; i < 80 ; i++){
                                    read_sl_front();
                                    if(sl_front[5] && sl_front[6]){FG=1;break;}
                                    else FG=0;
                                }
                                if(FG)break;
                            };
                        }
                            break;
        case s6+s7+s8 :  for (int i = 0; i < counts; i++)
                        {
                            read_sl_front();
                            while(!sl_front[5] || !sl_front[6] || !sl_front[7]){
                                line_scan_front(dspeed);
                                for (int i = 0; i < 80 ; i++){
                                    read_sl_front();
                                    if(sl_front[5] && sl_front[6] && sl_front[7]){FG=1;break;}
                                    else FG=0;
                                }
                                if(FG)break;
                            };
                        }
                            break;
    }
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0); }
    flagtracemode = cc;
    
}
void xsline(int dmode, int dspeed, int dbreaktime){
    dspeed = xconvertpwm(dspeed);
    int counts = 20;
    read_sl_front();
    switch(dmode)
    {
        case s1 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0]){xline_scan_front(dspeed);};}break;
        case s2 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[1]){xline_scan_front(dspeed);};}break;
        case s3 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[2]){xline_scan_front(dspeed);};}break;
        case s4 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[3]){xline_scan_front(dspeed);};}break;
        case s5 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[4]){xline_scan_front(dspeed);};}break;
        case s6 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5]){xline_scan_front(dspeed);};}break;
        case s7 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[6]){xline_scan_front(dspeed);};}break;
        case s8 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[7]){xline_scan_front(dspeed);};}break;
            
        case s1+s2 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0] || !sl_front[1]){xline_scan_front(dspeed);};}break;
        case s2+s3 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[1] || !sl_front[2]){xline_scan_front(dspeed);};}break;
        case s1+s2+s3 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0] || !sl_front[1] || !sl_front[2]){xline_scan_front(dspeed);};}break;
        
        case s7+s8 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};}break;
        case s6+s7 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5] || !sl_front[6]){xline_scan_front(dspeed);};}break;
        case s6+s7+s8 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5] || !sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};}break;
    }
    if (dbreaktime < 0)       {dxmotor(dspeed, dspeed);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {dxmotor(-dspeed, -dspeed);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0); }
    flagtracemode = cc;
    
}

void sline(int dmode, int dspeed, int dbreaktime){
    if (FMOTOR == MGR)
    {
        sline_mgr(dmode,dspeed,dbreaktime);
    }
    else if (FMOTOR == MDX)
    {
        xsline(dmode,dspeed,dbreaktime);
    }
    else
    {
        dspeed = convertpwm(dspeed);
        int counts = 20;
        read_sl_front();
        switch(dmode)
        {
            case s1 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0]){line_scan_front(dspeed);};}break;
            case s2 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[1]){line_scan_front(dspeed);};}break;
            case s3 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[2]){line_scan_front(dspeed);};}break;
            case s4 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[3]){line_scan_front(dspeed);};}break;
            case s5 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[4]){line_scan_front(dspeed);};}break;
            case s6 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5]){line_scan_front(dspeed);};}break;
            case s7 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[6]){line_scan_front(dspeed);};}break;
            case s8 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[7]){line_scan_front(dspeed);};}break;
                
            case s1+s2 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0] || !sl_front[1]){line_scan_front(dspeed);};}break;
            case s2+s3 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[1] || !sl_front[2]){line_scan_front(dspeed);};}break;
            case s1+s2+s3 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[0] || !sl_front[1] || !sl_front[2]){line_scan_front(dspeed);};}break;
                
            case s7+s8 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};}break;
            case s6+s7 :     for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5] || !sl_front[6]){line_scan_front(dspeed);};}break;
            case s6+s7+s8 :  for (int i = 0; i < counts; i++){read_sl_front();while(!sl_front[5] || !sl_front[6] || !sl_front[7]){line_scan_front(dspeed);};}break;
        }
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
        else {buzzer(1, 10, 0); }
        flagtracemode = cc;
    }
}
void xline(int dmode, int dspeed, int dbreaktime){
    dspeed = xconvertpwm(dspeed);
    int counts = 20;
    read_sl_front();
    switch (dmode)
    {
        case ff:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[7]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
        case fl:
            for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){xline_scan_front(dspeed);};
                    
                }
            for (int i = 0; i < counts; i++)
                {
                    read_sl_front();
                    while(!sl_front[0] || !sl_front[1]){xline_scan_front(dspeed);};
                    
                }
            for (int i = 0; i < 100; i++)
                {
                    read_sl_front();
                    while(sl_front[0] || sl_front[1]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
                }
            break;
        case fl3:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
        case fl4:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2] || !sl_front[3]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2] || sl_front[3]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
            
        case fr:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
        case fr3:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
        case fr4:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[4] ||!sl_front[5] || !sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[4] ||sl_front[5] || sl_front[6] || sl_front[7]){read_sl_front();dxmotor(dspeed, dspeed);};//{xline_scan_front(dspeed);};
            }
            break;
        case sl:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1]){xline_scan_front(dspeed);};
                
            }
            break;
        case sl3:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[0] || sl_front[1] || sl_front[2]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[0] || !sl_front[1] || !sl_front[2]){xline_scan_front(dspeed);};
                
            }
            break;
        case sr:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[6] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            break;
        case sr3:
            for (int i = 0; i < 100; i++)
            {
                read_sl_front();
                while(sl_front[5] || sl_front[6] || sl_front[7]){xline_scan_front(dspeed);};
                
            }
            for (int i = 0; i < counts; i++)
            {
                read_sl_front();
                while(!sl_front[5] || !sl_front[6] || !sl_front[7]){xline_scan_front(dspeed);};
                
            }
            break;
    }
    
   if (dbreaktime < 0)       {dxmotor(dspeed, dspeed);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {dxmotor(0, 0); dxmotor(-dspeed, -dspeed);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0); }
    
    buzzer(1, 10, 0);
    flagtracemode = cc;
    
}

#define DS1 8
#define DS2 9
#define DS3 10

Servo servoa;
Servo servob;
Servo servoc;
// twelve servo objects can be created on most boards

//-----------------------------------------------
#define SERVO_PIN3 10
#define SERVO_PIN4 9
#define SERVO_PIN5 8


void servo_init()
{
    servoa.attach(SERVO_PIN3);
    servob.attach(SERVO_PIN4);
    servoc.attach(SERVO_PIN5);
}
int temp_s3 = 90;
void servo3(int dpos,int dspeed)
{
    servoa.write(temp_s3);
    char f1 = 1;
    while (f1 )
    {
        if (temp_s3 == dpos )f1 = 0;
        else {
            if (temp_s3 < dpos) {
                servoa.write(temp_s3);  temp_s3++;
            }
            else {
                servoa.write(temp_s3);  temp_s3--;
            }
        }
        delay(dspeed);
    }
    temp_s3 = dpos;
    servoa.write(dpos);
}

int temp_s4 = 90;
void servo4(int dpos,int dspeed)
{
    servob.write(temp_s4);
    char f1 = 1;
    while (f1 )
    {
        if (temp_s4 == dpos )f1 = 0;
        else {
            if (temp_s4 < dpos) {
                servob.write(temp_s4);  temp_s4++;
            }
            else {
                servob.write(temp_s4);  temp_s4--;
            }
        }
        delay(dspeed);
    }
    temp_s4 = dpos;
    servob.write(dpos);
}

int temp_s5 = 90;
void servo5(int dpos,int dspeed)
{
    servoc.write(temp_s5);
    char f1 = 1;
    while (f1 )
    {
        if (temp_s5 == dpos )f1 = 0;
        else {
            if (temp_s5 < dpos) {
                servoc.write(temp_s5);  temp_s5++;
            }
            else {
                servoc.write(temp_s5);  temp_s5--;
            }
        }
        delay(dspeed);
    }
    temp_s5 = dpos;
    servoc.write(dpos);
}


//-----------------------------------------------
int pos = 0;    // variable to store the servo position


int temp_servo3 = 50;
int temp_servol = 50;
int temp_servor = 50;

int servo_up   = 30;
int speed_up  = 0;

int servo_down1 = 80;
int speed_down1 = 0;

int servo_down2 = 75;
int speed_down2 = 0;

int servo_down3 = 70;
int speed_down3 = 0;


int servo_left_open   = 90;
int speed_left_open  = 0;

int servo_left_close  = 40;
int speed_left_close  = 0;

int servo_right_open  = 15;
int speed_right_open  = 0;

int servo_right_close   = 65;
int speed_right_close  = 0;


void set_up(int d_up, int dsp)
{
    servo_up  = d_up; speed_up = dsp;
}
void set_down1(int d_down, int dsp)
{
    servo_down1   = d_down; speed_down1 = dsp;
}
void set_down2(int d_down, int dsp)
{
    servo_down2   = d_down; speed_down2 = dsp;
}
void set_down3(int d_down, int dsp)
{
    servo_down3   = d_down; speed_down3 = dsp;
}
void set_open(int d_left, int spdL, int d_right, int spdR)
{
    servo_left_open = d_left;   speed_left_open = spdL;
    servo_right_open  = d_right;  speed_right_open = spdR;
}
void set_close(int d_left, int spdL, int d_right, int spdR)
{
    servo_left_close  = d_left; speed_left_close = spdL;
    servo_right_close = d_right;  speed_right_close = spdR;
}

void dservo(int id, int d_start, int d_target, int d_speed, int d_time)
{
    doServo(id, d_start);
    if (d_start < d_target)
    {
        for ( int ii = d_start ; ii <= d_target; ii++)
        {
            doServo(id, ii);
            delay(d_speed);
        }
    }
    else
    {
        for ( int ii = d_start ; ii >= d_target; ii--)
        {
            doServo(id, ii);
            delay(d_speed);
        }
    }
    doServo(id, d_target);
    delay(d_time);
}

void dservo2(int id1, int d_start1, int d_target1, int id2, int d_start2, int d_target2, int d_speed, int d_time)
{
    doServo(id1, d_start1); doServo(id2, d_start2);
    int temp1 = d_start1;
    int temp2 = d_start2;
    char f1 = 1, f2 = 1;
    while (f1 || f2)
    {
        if (temp1 == d_target1 )f1 = 0;
        else {
            if (d_start1 < d_target1) {
                doServo(id1, temp1);  temp1++;
            }
            else {
                doServo(id1, temp1);  temp1--;
            }
        }
        if (temp2 == d_target2 )f2 = 0;
        else {
            if (d_start2 < d_target2) {
                doServo(id2, temp2);  temp2++;
            }
            else {
                doServo(id2, temp2);  temp2--;
            }
        }
        delay(d_speed);
    }
    doServo(id1, d_target1); doServo(id2, d_target2);
    delay(d_time);
}
void up()
{
    dservo(DS3, temp_servo3, servo_up, speed_up, 300);
    temp_servo3 = servo_up;
}
void down1()
{
    dservo(DS3, temp_servo3, servo_down1, speed_down1, 300);
    temp_servo3 = servo_down1;
}
void down2()
{
    dservo(DS3, temp_servo3, servo_down2, speed_down2, 300);
    temp_servo3 = servo_down2;
}
void down3()
{
    dservo(DS3, temp_servo3, servo_down3, speed_down3, 300);
    temp_servo3 = servo_down3;
}
void dclose()
{
    dservo2(DS1, temp_servol, servo_left_close, DS2, temp_servor, servo_right_close, speed_left_close, 400);
    temp_servol = servo_left_close;
    temp_servor = servo_right_close;
}
void dopen()
{
    dservo2(DS1, temp_servol, servo_left_open, DS2, temp_servor, servo_right_open,  speed_left_open, 400);
    temp_servol = servo_left_open;
    temp_servor = servo_right_open;
}

void doServo(int id, int dpos)
{
    switch (id)
    {
        case DS1 : servoa.write(dpos); break;
        case DS2 : servob.write(dpos); break;
        case DS3 : servoc.write(dpos); break;
            
    }
}

//servo3 -> pin 10
//servo4 -> pin 9
//servo5 -> pin 8

void transporter_init()
{
    servoa.attach(DS1);
    servob.attach(DS2);
    servoc.attach(DS3);
}

void take(int dspeed, int dbreaktime)
{
    sensor(rear);
    dopen();
    centerfront();
    down1();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    
            for (int i = 0; i < 100; i++)
                while (((sl_front_all & 0b00000001) == 0b00000001) && ((sl_front_all & 0b10000000) == 0b10000000)) {
                    line_scan_front(dspeed);
                };
            for (int i = 0; i < 5; i++)
                while (((sl_front_all & 0b00000001) != 0b00000001) && ((sl_front_all & 0b10000000) != 0b10000000)) {
                    line_scan_front(dspeed);
                };
            for (int i = 0; i < 10500; i++)
                while (((sl_front_all & 0b00000001) == 0b00000001) && ((sl_front_all & 0b10000000) == 0b10000000))  {
                    line_scan_front(dspeed);
                };
    
    
    d_motor(0, 0);
    dclose(); delay(300);
    up();
    if(dspeed < 70)dspeed =70;
    if (dbreaktime < 0)       {
        dbreaktime = -dbreaktime;
        for (int i = 0; i < dbreaktime; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                default :           line_scan_front(dspeed);break;
                    
            }

           
        }
        d_motor(0, 0);   buzzer(1, 10, 0);
    }
    else if(dbreaktime > 0)   {for (int i = 0; i < dbreaktime; i++){d_motor(-dspeed,-dspeed); delay(1);d_motor(0, 0);}   d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
  
}
void put(int dspeed,int  dist, int layer)
{
    dist = dist * 10;
    sensor(rear);
    centerfront();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    read_sl_rear();
    
    for (int i = 0; i < 100; i++)
        while (((sl_rear_all & 0b00000111) == 0b00000111) && ((sl_rear_all & 0b11100000) == 0b11100000) && dist>=0) {
            line_scan_front(dspeed);read_sl_rear(); dist--;

        };
    for (int i = 0; i < 5; i++)
        while (((sl_rear_all & 0b00000111) != 0b00000111) && ((sl_rear_all & 0b10000000) != 0b11100000) && dist>=0) {
            line_scan_front(dspeed);read_sl_rear(); dist--;

        };
    d_motor(0, 0); delay(20);
    
    if(dist>=1)
    {
       sensor(front);
        read_sl_front();
        
        for (int i = 0; i < 100; i++)
            while (((sl_front_all & 0b00000111) == 0b00000111) && ((sl_front_all & 0b11100000) == 0b11100000)) {
                line_scan_front(dspeed);
            };
        for (int i = 0; i < 5000; i++)
            while (((sl_front_all & 0b00000111) != 0b00000111) && ((sl_front_all & 0b11100000) != 0b11100000)) {
                line_scan_front(dspeed);
            };
        d_motor(50, 50);delay(300);

        for (int i = 0; i < 5000; i++)
            while (((sl_front_all & 0b00000111) == 0b00000111) && ((sl_front_all & 0b11100000) == 0b11100000))  {
                line_scan_front(dspeed);
            };
    }
    motor(0,0,0);
    switch (layer)
    {
        case 2 : down2();break;
        case 3 : down3();break;
        default : down1();break;
    }
    delay(300);
    dopen(); delay(400);
  
    {
        sensor(front);
        d_motor(0, 0);
        if(dspeed < 70)dspeed =70;
        for (int i = 0; i < 700; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;

                default :           line_scan_front(dspeed);break;
                    
            }
            
            
        }
        d_motor(0, 0);   buzzer(1, 10, 0);

       
 
    }
    up();
    sensor(rear);
    d_motor(0, 0);delay(30);
    
}
void putz(int dspeed,int  dist, int layer)
{
    dist = dist * 10;
    sensor(rear);
    centerfront();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    read_sl_rear();
    
    for (int i = 0; i < 100; i++)
        while (((sl_rear_all & 0b00000111) == 0b00000111) && ((sl_rear_all & 0b11100000) == 0b11100000) && dist>=0) {
            line_scan_front(dspeed);read_sl_rear(); dist--;
            
        };
    for (int i = 0; i < 5; i++)
        while (((sl_rear_all & 0b00000111) != 0b00000111) && ((sl_rear_all & 0b10000000) != 0b11100000) && dist>=0) {
            line_scan_front(dspeed);read_sl_rear(); dist--;
            
        };
    d_motor(0, 0); delay(20);
    
    if(dist>=1)
    {
        sensor(front);
        read_sl_front();
        
        for (int i = 0; i < 100; i++)
            while (((sl_front_all & 0b00000111) == 0b00000111) && ((sl_front_all & 0b11100000) == 0b11100000)) {
                line_scan_front(dspeed);
            };
        for (int i = 0; i < 5000; i++)
            while (((sl_front_all & 0b00000111) != 0b00000111) && ((sl_front_all & 0b11100000) != 0b11100000)) {
                line_scan_front(dspeed);
            };
        d_motor(50, 50);delay(300);
        
        for (int i = 0; i < 5000; i++)
            while (((sl_front_all & 0b00000111) == 0b00000111) && ((sl_front_all & 0b11100000) == 0b11100000))  {
                line_scan_front(dspeed);
            };
    }
    motor(0,0,0);
    switch (layer)
    {
        case 2 : down2();break;
        case 3 : down3();break;
        default : down1();break;
    }
    delay(300);
    dopen(); delay(400);
    
    {
        sensor(front);
        d_motor(0, 0);
        if(dspeed < 70)dspeed =70;
        for (int i = 0; i < 700; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    
                default :           line_scan_front(dspeed);break;
                    
            }
            
            
        }
        d_motor(0, 0);   buzzer(1, 10, 0);
        
        
        
    }
   // up();
    sensor(rear);
    d_motor(0, 0);delay(30);
    
}




// setting position
int  dsetopen = 260;
int  dspopen = 400;
int  dsetclose = 450;
int  dspclose = 400;
int  dsetup  =  100;
int  dspup = 350;
int  dsetdown1  = 500;
int  dspdown1 = 500;
int  dsetdown2 = 450;
int  dspdown2 = 500;
//

void set_xclose(int dpos, int dspeed){ dsetclose = dpos; dspclose = dspeed;}
void set_xopen(int dpos, int dspeed) { dsetopen  = dpos; dspopen  = dspeed;}
void set_xup(int dpos, int dspeed)   { dsetup    = dpos; dspup    = dspeed;}
void set_xdown1(int dpos, int dspeed){ dsetdown1 = dpos; dspdown1 = dspeed;}
void set_xdown2(int dpos, int dspeed){ dsetdown2 = dpos; dspdown2 = dspeed;}
void xclose()
{
  dxlservo(ID2, dsetclose, dspclose);
  delay(300);
}
void xopen()
{
  dxlservo(ID2, dsetopen, dspopen);
  delay(300);
}
void xdown1(int dtime)
{
  dxlservo(ID1, dsetdown1, dspdown1);
  delay(dtime);
}
void xdown2(int dtime)
{
  dxlservo(ID1, dsetdown2, dspdown2);
  delay(dtime);
}
void xdown1s()
{
  dxlservo(ID1, dsetdown1, 100);
  delay(1000);
}
void xdown2s()
{
  dxlservo(ID1, dsetdown2, 100);
  delay(1000);
}
void xup(int dtime)
{
  dxlservo(ID1, dsetup, dspup);
  delay(dtime);
}

void xtake(int dspeed, int dbreaktime)
{
   // sensor(rear);
    xopen();
    centerfront();
    xdown1(300);delay(500);
    dspeed = convertpwm(dspeed);
    read_sl_front();
    
            for (int i = 0; i < 100; i++)
                while (((sl_front_all & 0b00000001) == 0b00000001) && ((sl_front_all & 0b10000000) == 0b10000000)) {
                    line_scan_front(dspeed);
                };
            for (int i = 0; i < 5; i++)
                while (((sl_front_all & 0b00000001) != 0b00000001) && ((sl_front_all & 0b10000000) != 0b10000000)) {
                    line_scan_front(dspeed);
                };
            for (int i = 0; i < 10500; i++)
                while (((sl_front_all & 0b00000001) == 0b00000001) && ((sl_front_all & 0b10000000) == 0b10000000))  {
                    line_scan_front(dspeed);
                };
    
    
    d_motor(0, 0);
    xclose(); delay(300);
    xup(300);
    if(dspeed < 70)dspeed =70;
    if (dbreaktime < 0)       {
        dbreaktime = -dbreaktime;
        for (int i = 0; i < dbreaktime; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                default :           line_scan_front(dspeed);break;
                    
            }

           
        }
        d_motor(0, 0);   buzzer(1, 10, 0);
    }
    else if(dbreaktime > 0)   {for (int i = 0; i < dbreaktime; i++){d_motor(-dspeed,-dspeed); delay(1);d_motor(0, 0);}   d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
  
}
void xput(int dspeed,int  dist, int layer, int dbreaktime)
{
    dist = dist * 10;
    centerfront();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    read_sl_rear();
    for (int i = 0; i < 100; i++)
        while (dist>=0) {
            line_scan_front(dspeed);dist--;
            
        };
    for (int i = 0; i < 5; i++)
        while ( dist>=0) {
            line_scan_front(dspeed); dist--;
            
        };
    d_motor(0, 0); delay(20);
    
    switch (layer)
    {
        case 2 : xdown2(300);break;
        default : xdown1(300);break;
    }
    delay(300);
    xopen(); delay(400);

    if(dspeed < 70)dspeed =70;
    if (dbreaktime < 0)       {
        dbreaktime = -dbreaktime;
        for (int i = 0; i < dbreaktime; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                default :           line_scan_front(dspeed);break;
                    
            }
        }
        d_motor(0, 0);
    }
    else if(dbreaktime > 0)   {for (int i = 0; i < dbreaktime; i++){d_motor(-dspeed,-dspeed); delay(1);d_motor(0, 0);}   d_motor(0, 0);   }
    xup(300);
    d_motor(0, 0);delay(30);
    buzzer(1, 10, 0);
}

void xputs(int dspeed,int  dist, int layer, int dbreaktime)
{
    dist = dist * 10;
    centerfront();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    read_sl_rear();
    for (int i = 0; i < 100; i++)
        while (dist>=0) {
            line_scan_front(dspeed);dist--;
            
        };
    for (int i = 0; i < 5; i++)
        while ( dist>=0) {
            line_scan_front(dspeed); dist--;
            
        };
    d_motor(0, 0); delay(20);
    
    switch (layer)
    {
        case 2 : xdown2s();break;
        default : xdown1s();break;
    }
    delay(1000);
    xopen(); delay(400);
    
    if(dspeed < 70)dspeed =70;
        if (dbreaktime < 0)       {
            dbreaktime = -dbreaktime;
            for (int i = 0; i < dbreaktime; i++)
            {
                read_sl_front();
                switch (sl_front_all)
                {
                    case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                    default :           line_scan_front(dspeed);break;
                        
                }
            }
            d_motor(0, 0);
        }
        else if(dbreaktime > 0)   {for (int i = 0; i < dbreaktime; i++){d_motor(-dspeed,-dspeed); delay(1);d_motor(0, 0);}   d_motor(0, 0);   }
    xup(300);
    d_motor(0, 0);delay(30);
    buzzer(1, 10, 0);
}
void xputz(int dspeed,int  dist, int layer, int dbreaktime)
{
    dist = dist * 10;
    centerfront();
    dspeed = convertpwm(dspeed);
    read_sl_front();
    read_sl_rear();
    for (int i = 0; i < 100; i++)
        while (dist>=0) {
            line_scan_front(dspeed);dist--;
            
        };
    for (int i = 0; i < 5; i++)
        while ( dist>=0) {
            line_scan_front(dspeed); dist--;
            
        };
    d_motor(0, 0); delay(20);
    
    switch (layer)
    {
        case 2 : xdown2(300);break;
        default : xdown1(300);break;
    }
    delay(300);
    xopen(); delay(400);
    
    if(dspeed < 70)dspeed =70;
    if (dbreaktime < 0)       {
        dbreaktime = -dbreaktime;
        for (int i = 0; i < dbreaktime; i++)
        {
            read_sl_front();
            switch (sl_front_all)
            {
                case 0b00000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000001:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b00000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b11000011:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                case 0b10000000:  d_motor(dspeed, dspeed);delay(1);d_motor(0, 0);delay(1);i+=2; break;
                default :           line_scan_front(dspeed);break;
                    
            }
        }
        d_motor(0, 0);
    }
    else if(dbreaktime > 0)   {for (int i = 0; i < dbreaktime; i++){d_motor(-dspeed,-dspeed); delay(1);d_motor(0, 0);}   d_motor(0, 0);   }
 //   xup(300);
    d_motor(0, 0);delay(30);
    buzzer(1, 10, 0);
}
//-------------------------------------------------

//-------------------------------------------------
void linejump(int dl, int dr)
{
    ss();
    dl = convertpwm(dl); dr = convertpwm(dr);
    
    int dspeed = (dl+dr)/2;
    read_sl_rear();
    while(1)
    {
        read_sl_rear();
        if(!sl_rear[6] && !sl_rear[1])d_motor(-dspeed,-dspeed);
        else if( sl_rear[6] && !sl_rear[1])d_motor(-dspeed,0);
        else if(!sl_rear[6] &&  sl_rear[1])d_motor(0,-dspeed);
        else  {d_motor(0,0);break;}
        
    }
    buzzer(1, 10, 0);
    ss();
     read_sl_front();
    for (int i = 0; i < 5000; i++)
        while (((sl_front_all & 0b11100111) != 0b11100111)) {
            d_motor(dl,dr);     read_sl_front();

        };
    d_motor(0,0);
    read_sl_front();
    for (int i = 0; i < 5000; i++)
        while (((sl_front_all & 0b11100111) == 0b11100111)) {
            d_motor(dl,dr);     read_sl_front();
            
        };

    buzzer(1, 10, 0);
    
    linedelay(10,15,0);

}
void linedelay_p(int dspeed, int dtime, int dbreaktime)
{
    dspeed = convertpwm(dspeed);
    do {
        for (int i = 0; i < 50; i++)
            line_scan_front_p(dspeed);
        if(!digitalRead(8))break;
    } while (dtime--);
    
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
    flagtracemode = cc;
}
void linedelay_mgr(int dspeed, int dtime, int dbreaktime)
{
    dspeed = convertpwm(dspeed);
    do {
        //for (int i = 0; i < 5; i++)
        {
            line_scan_front(dspeed);
            for (int i = 0; i < 80 ; i++){
                read_sl_front();
            }
        }
    } while (dtime--);
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
    flagtracemode = cc;
}
void xlinedelay(int dspeed, int dtime, int dbreaktime)
{
    dspeed = xconvertpwm(dspeed);
    do {
        for (int i = 0; i < 10; i++)
            xline_scan_front(dspeed);
        
    } while (dtime--);
    
    
    if (dbreaktime < 0)       {dxmotor(dspeed, dspeed);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {dxmotor(-dspeed, -dspeed);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);}
    flagtracemode = cc;
}
void linedelay(int dspeed, int dtime, int dbreaktime)
{
    if (FMOTOR == MGR)
    {
        linedelay_mgr(dspeed, dtime,dbreaktime);
    }
    else if (FMOTOR == MDX)
    {
        xlinedelay(dspeed, dtime,dbreaktime);
    }
    else
    {
        dspeed = convertpwm(dspeed);
        do {
            for (int i = 0; i < 50; i++)
                line_scan_front(dspeed);
            
        } while (dtime--);
        
        
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
        else {buzzer(1, 10, 0);d_motor(0, 0);}
        flagtracemode = cc;
    }
}

void linedelay_l(int dspeed, int dtime, int dbreaktime)
{
    dspeed = convertpwm(dspeed);
    do {
        for (int i = 0; i < 50; i++)
            line_scan_front(dspeed);
        if(!digitalRead(8))break;
    } while (dtime--);
    
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
    flagtracemode = cc;
}
int igno = 10;
void xfindline(int dl, int dr, int dbreaktime)
{
    dl = xconvertpwm(dl);
    dr = xconvertpwm(dr);
    
    /* read_sl_front();
     for (int i = 0; i < dbreaktime; i++)
     while (sl_front_all) {
     read_sl_front();dxmotor(dl, dr);
     };
     */
    read_sl_front();
    for (int i = 0; i < igno; i++)
        while (!sl_front[2] && !sl_front[3] && !sl_front[4] && !sl_front[5])//!sl_front_all)
             {
            read_sl_front();dxmotor(dl, dr);
        };
    
    /*  if (dbreaktime < 0)       {dxmotor(dl, dr);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
     else if(dbreaktime > 0)   {dxmotor(-dl, -dr);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
     else {buzzer(1, 10, 0);}
     */
    dxmotor(0, 0);
    flagtracemode = cc;
}
void findline_mgr(int dl, int dr, int dbreaktime)
{
    
    dl = convertpwm(dl);
    dr = convertpwm(dr);
    
    int dspeed =( dl + dr )/2;
    read_sl_front();
  /*  for (int i = 0; i < 10000; i++)
        while (sl_front_all) {
            read_sl_front();motor_break_left(); motor_break_right();d_motor(dl, dr);
        };*/
    for (int i = 0; i < igno; i++)
        while (!sl_front_all) {
            read_sl_front();d_motor(dl, dr);
        };
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
    flagtracemode = cc;
}
void findline(int dl, int dr, int dbreaktime)
{
    

    
    if (FMOTOR == MGR)
    {
        findline_mgr(dl, dr, dbreaktime);
    }
    else if (FMOTOR == MDX)
    {
        xfindline(dl, dr, dbreaktime);
    }
    else
    {
        dl = convertpwm(dl);
        dr = convertpwm(dr);
        
        int dspeed =( dl + dr )/2;
        read_sl_front();
      /*  for (int i = 0; i < 10000; i++)
            while (sl_front_all) {
                read_sl_front();motor_break_left(); motor_break_right();d_motor(dl, dr);
            };*/
        for (int i = 0; i < igno; i++)
            while (!sl_front_all) {
                read_sl_front();d_motor(dl, dr);
            };
        
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
        else {buzzer(1, 10, 0);d_motor(0, 0);}
        flagtracemode = cc;
    }
}

void xlostline(int dspeed, int dbreaktime)
{
 
    dspeed = xconvertpwm(dspeed);
    
    read_sl_front();
    /*  for (int i = 0; i < 4; i++)
     while (sl_front_all ) {
     xline_scan_front(dspeed);
     };*/
    read_sl_front();
    for (int i = 0; i < igno; i++)
        while (sl_front[2] || sl_front[3] || sl_front[4] ||sl_front[5])//sl_front_all)
        {
            xline_scan_front(dspeed);
        };
    
    //if (dbreaktime < 0)       {dxmotor(dspeed, dspeed);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
    //else if(dbreaktime > 0)   {dxmotor(-dspeed, -dspeed);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
    //else {buzzer(1, 10, 0);}
    dxmotor(0, 0);
    buzzer(1, 10, 0);
    flagtracemode = cc;
}
/*
#define PUTIH 0
#define MERAH   1
#define COKLAT  2
#define KUNING  3
#define BIRU    4
#define HIJAU   5

void xcolor(int dspeed, int dwarna)
{
 
    dspeed = xconvertpwm(dspeed);
    int warna = PUTIH;
      if(digitalRead(8) == 0 &&  digitalRead(9) == 0 && digitalRead(10) == 1)warna= MERAH; // merah
      else if(digitalRead(8) == 0 &&  digitalRead(9) == 1 && digitalRead(10) == 0)warna= COKLAT; // coklat
      else if(digitalRead(8) == 0 &&  digitalRead(9) == 1 && digitalRead(10) == 1)warna= KUNING; // kuning
      else if(digitalRead(8) == 1 &&  digitalRead(9) == 0 && digitalRead(10) == 0)warna= BIRU; // biru
      else if(digitalRead(8) == 1 &&  digitalRead(9) == 0 && digitalRead(10) == 1)warna= HIJAU; // hijau
      else   warna= PUTIH;
    
    read_sl_front();
   
    read_sl_front();
    for (int i = 0; i < igno; i++)
        while (dwarna != warna )//sl_front_all)
        {
            xline_scan_front(dspeed);
            if(digitalRead(8) == 0 &&  digitalRead(9) == 0 && digitalRead(10) == 1)warna= MERAH; // merah
            else if(digitalRead(8) == 0 &&  digitalRead(9) == 1 && digitalRead(10) == 0)warna= COKLAT; // coklat
            else if(digitalRead(8) == 0 &&  digitalRead(9) == 1 && digitalRead(10) == 1)warna= KUNING; // kuning
            else if(digitalRead(8) == 1 &&  digitalRead(9) == 0 && digitalRead(10) == 0)warna= BIRU; // biru
            else if(digitalRead(8) == 1 &&  digitalRead(9) == 0 && digitalRead(10) == 1)warna= HIJAU; // hijau
            else   warna= PUTIH;
        };
    
    //if (dbreaktime < 0)       {dxmotor(dspeed, dspeed);     delay(-dbreaktime);dxmotor(0, 0);   buzzer(1, 10, 0);}
    //else if(dbreaktime > 0)   {dxmotor(-dspeed, -dspeed);   delay(dbreaktime);dxmotor(0, 0);    buzzer(1, 10, 0);}
    //else {buzzer(1, 10, 0);}
    dxmotor(0, 0);
    buzzer(1, 10, 0);
    flagtracemode = cc;
}
*/
void lostline_mgr(int dspeed, int dbreaktime)
{
    dspeed = convertpwm(dspeed);
    read_sl_front();
    /*   for (int i = 0; i < 1000; i++)
     while (!sl_front_all ) {
     line_scan_front(dspeed);
     };
     */ for (int i = 0; i < igno; i++)
         while (sl_front_all) {
             line_scan_front(dspeed);
             for (int i = 0; i < 80 ; i++){
                 read_sl_front();
             }
         };
    
    if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
    else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
    else {buzzer(1, 10, 0);d_motor(0, 0);}
    flagtracemode = cc;
}
void lostline(int dspeed, int dbreaktime)
{
    if(igno < 1) igno = 1;

    if (FMOTOR == MGR)
    {
        lostline_mgr(dspeed, dbreaktime);
    }
    else if (FMOTOR == MDX)
    {
        xlostline( dspeed, dbreaktime);
    }
    else
    {
        dspeed = convertpwm(dspeed);
        read_sl_front();
     /*   for (int i = 0; i < 1000; i++)
            while (!sl_front_all ) {
                line_scan_front(dspeed);
            };
       */ for (int i = 0; i < igno; i++)
            while (sl_front_all) {
                line_scan_front(dspeed);
            };
        
        if (dbreaktime < 0)       {d_motor(dspeed, dspeed);     delay(-dbreaktime);d_motor(0, 0);   buzzer(1, 10, 0);}
        else if(dbreaktime > 0)   {d_motor(-dspeed, -dspeed);   delay(dbreaktime);d_motor(0, 0);    buzzer(1, 10, 0);}
        else {buzzer(1, 10, 0);d_motor(0, 0);}
        flagtracemode = cc;
    }
}
void drive_turn(int don, int doff)
{
    dturnon = don;
    dturnoff = doff;
}

void left_start1(int dl, int dr)
{
    //dl = convertpwm(dl);dr = convertpwm(dr);
    read_sl_front(); d_motor(dl, dr);
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);

    } while (sl_front_1());
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (!sl_front_1());
    d_motor(-dl, 0);
    d_motor(0, 0);
}
void left_start4(int dl, int dr)
{
    read_sl_front(); d_motor(dl, dr);
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (sl_front_4());
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (!sl_front_4());
    d_motor(-dl, 0);
    d_motor(0, 0);
}
void left(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else 			  left3(dl, dr, dtime);
}
void left1(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right8(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_1());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_1());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left2(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right7(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_2());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_2());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left3(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right6(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_3());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_3());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left4(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right5(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_4());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_4());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left5(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right4(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_5());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_5());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left6(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right3(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_6());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_6());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left7(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right2(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        left_start4(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_7());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_7());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void left8(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        right1(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        left_start1(dl, dr);
        left_start4(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_8());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_8());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}

void right_start8(int dl, int dr)
{
    read_sl_front(); d_motor(dl, dr);
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (sl_front_8());
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (!sl_front_8());
    d_motor(0, -dr);
    d_motor(0, 0);
}
void right_start5(int dl, int dr)
{
    read_sl_front(); d_motor(dl, dr);
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (sl_front_5());
    do {
        d_motor(dl, dr);                            delayMicroseconds(dturnon);
        motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
    } while (!sl_front_5());
    d_motor(0, -dr);
    d_motor(0, 0);
}
void right(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        right6(dl, dr, dtime);
    }
}
void right8(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left1(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_8());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_8());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right7(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left2(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_7());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_7());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right6(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left3(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_6());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_6());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right5(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left4(-dl, -dr, dtime);
        data_mirror = on ;
    }
    else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_5());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_5());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right4(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left5(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_4());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_4());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right3(int dl, int dr, int dtime)
{
    if(data_mirror == on)
    { data_mirror = off ;
        left6(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_3());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_3());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right2(int dl, int dr, int dtime)
{
    if(data_mirror == on) 
    { data_mirror = off ; 
        left7(-dl, -dr, dtime);
        data_mirror = on ;
    }  else 
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        right_start5(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_2());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_2());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}
void right1(int dl, int dr, int dtime)
{
    if(data_mirror == on) 
    { data_mirror = off ; 
        left8(-dl, -dr, dtime);
        data_mirror = on ;
    }  else
    {
        dl = convertpwm(dl); dr = convertpwm(dr);
        right_start8(dl, dr);
        right_start5(dl, dr);
        read_sl_front(); d_motor(dl, dr);
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (sl_front_1());
        do {
            d_motor(dl, dr);                            delayMicroseconds(dturnon);
            motor_break_left(); motor_break_right();    delayMicroseconds(dturnoff);
        } while (!sl_front_1());
        d_motor(-dl, -dr); delay(dtime);
        d_motor(0, 0); buzzer(1, 10, 0);
    }
}//---------------------------------------------
void xleft_start1(int dl, int dr)
{
  //  dl = xconvertpwm(dl);dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_1());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_1());
    dxmotor(-dl, 0);
    dxmotor(0, 0);
}
void xleft_start4(int dl, int dr)
{
   // dl = xconvertpwm(dl);dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_4());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_4());
    dxmotor(-dl, 0);
    dxmotor(0, 0);
}
void xleft(int dl, int dr, int dtime)
{
    xleft3(dl, dr, dtime);
}
void xleft1(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_1());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_1());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft2(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_2());
    */do {
        dxmotor(dl, dr);
    } while (!sl_front_2());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft3(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_3());
    */do {
        dxmotor(dl, dr);
    } while (!sl_front_3());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft4(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_4());
    */do {
        dxmotor(dl, dr);
    } while (!sl_front_4());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft5(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_5());
    */do {
        dxmotor(dl, dr);
    } while (!sl_front_5());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft6(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
 /*   do {
        dxmotor(dl, dr);
    } while (sl_front_6());
   */ do {
        dxmotor(dl, dr);
    } while (!sl_front_6());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft7(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    xleft_start4(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_7());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_7());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}
void xleft8(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xleft_start1(dl, dr);
    xleft_start4(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_8());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_8());
    
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
}

void xright_start8(int dl, int dr)
{
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_8());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_8());
    dxmotor(0, -dr); 
    dxmotor(0, 0);
}
void xright_start5(int dl, int dr)
{
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_5());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_5());
    dxmotor(0, -dr);
    dxmotor(0, 0);
}
void xright(int dl, int dr, int dtime)
{
    xright6(dl, dr, dtime);
}
void xright1(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start5(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_1());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_1());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright2(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start5(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_2());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_2());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright3(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start8(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
   /* do {
        dxmotor(dl, dr);
    } while (sl_front_3());
    */do {
        dxmotor(dl, dr);
    } while (!sl_front_3());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright4(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start8(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_4());
   */ do {
        dxmotor(dl, dr);
    } while (!sl_front_4());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright5(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start8(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
  /*  do {
        dxmotor(dl, dr);
    } while (sl_front_5());
   */ do {
        dxmotor(dl, dr);
    } while (!sl_front_5());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright6(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    xright_start8(dl, dr);
    read_sl_front(); dxmotor(dl, dr);
/*    do {
        dxmotor(dl, dr);
    } while (sl_front_6());
  */  do {
        dxmotor(dl, dr);
    } while (!sl_front_6());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);

}
void xright7(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
        read_sl_front(); dxmotor(dl, dr);
 /*   do {
        dxmotor(dl, dr);
    } while (sl_front_7());
   */ do {
        dxmotor(dl, dr);
    } while (!sl_front_7());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void xright8(int dl, int dr, int dtime)
{
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);
    do {
        dxmotor(dl, dr);
    } while (sl_front_8());
    do {
        dxmotor(dl, dr);
    } while (!sl_front_8());
    if(dtime <= 0) dtime=0;
    dxmotor(-dl, -dr); delay(dtime);
    dxmotor(0, 0); buzzer(1, 10, 0);
    
}
void sturn_mgr(int dl, int dr, int sens,int dbreaktime)
{
    dl = convertpwm(dl); dr = convertpwm(dr);
    read_sl_front(); d_motor(dl, dr);
    
    switch(sens)
    {
        case s1:do {d_motor(dl, dr);} while (!sl_front_1());break;
        case s2:do {d_motor(dl, dr);} while (!sl_front_2());break;
        case s3:do {d_motor(dl, dr);} while (!sl_front_3());break;
        case s4:do {d_motor(dl, dr);} while (!sl_front_4());break;
        case s5:do {d_motor(dl, dr);} while (!sl_front_5());break;
        case s6:do {d_motor(dl, dr);} while (!sl_front_6());break;
        case s7:do {d_motor(dl, dr);} while (!sl_front_7());break;
        case s8:do {d_motor(dl, dr);} while (!sl_front_8());break;
    }
    if(dl<0 && dr >0) d_motor(dl, -dr);
    else if(dl>0 && dr <0) d_motor(-dl, dr);
    else if(dl>0 && dr >0) d_motor(-dl, -dr);
    else if(dl<0 && dr <0) d_motor( dl, dr);
    delay(abs(dbreaktime));
    d_motor(0, 0); buzzer(1, 10, 0);
}
void sturn(int dl, int dr, int sens,int dbreaktime)
{
  if (FMOTOR == MDX)
  {
    dl = xconvertpwm(dl); dr = xconvertpwm(dr);
    read_sl_front(); dxmotor(dl, dr);

    switch(sens)
    {
        case s1:do {dxmotor(dl, dr);} while (!sl_front_1());break;
        case s2:do {dxmotor(dl, dr);} while (!sl_front_2());break;
        case s3:do {dxmotor(dl, dr);} while (!sl_front_3());break;
        case s4:do {dxmotor(dl, dr);} while (!sl_front_4());break;
        case s5:do {dxmotor(dl, dr);} while (!sl_front_5());break;
        case s6:do {dxmotor(dl, dr);} while (!sl_front_6());break;
        case s7:do {dxmotor(dl, dr);} while (!sl_front_7());break;
        case s8:do {dxmotor(dl, dr);} while (!sl_front_8());break;
    }
    if(dl<0 && dr >0) dxmotor(dl, -dr);
    else if(dl>0 && dr <0) dxmotor(-dl, dr);
    else if(dl>0 && dr >0) dxmotor(-dl, -dr);
    else if(dl<0 && dr <0) dxmotor( dl, dr);
    delay(abs(dbreaktime));
    dxmotor(0, 0); buzzer(1, 10, 0);
  }
  else if (FMOTOR == MGR)
  {
      sturn_mgr(dl,dr,sens,dbreaktime);
  }
}

char flagdx= 0;
#define pin_pushbutton 12
void dnext(){
    if(flagdx)dxmotor(0, 0);
    dmotor(0,0);
    while (digitalRead(pin_pushbutton)); buzzer(1, 80, 100);
    while (!digitalRead(pin_pushbutton));
    delay(300);
}

#define pin_pushbutton1 8
void dnext1(){
    pinMode(8, INPUT_PULLUP);
    while (digitalRead(pin_pushbutton1)); buzzer(1, 80, 100);
    while (!digitalRead(pin_pushbutton1));
    delay(300);
}

void xline_init(){
    pinMode(port_enable_sensor,OUTPUT);
    digitalWrite(port_enable_sensor,LOW);
    
    motor_init();
    motor(0,0,1);
    
    pinMode(port_buzzer, OUTPUT);
    pinMode(pin_pushbutton, INPUT_PULLUP);

    
    pinMode(port_channel_a, OUTPUT);
    pinMode(port_channel_b, OUTPUT);
    pinMode(port_channel_c, OUTPUT);
    
    
    pinMode(pin_sensor_line_a, INPUT);
    pinMode(pin_sensor_line_b, INPUT);
   
    sensor(front);
    buzzer(3,50,50);
    dnext();
    digitalWrite(port_enable_sensor,HIGH);
    dnext();
    
}

void dxline_init(){
    flagdx = 1;
    pinMode(port_enable_sensor,OUTPUT);
    digitalWrite(port_enable_sensor,LOW);
    delay(1000);
    
    dxl_init(ID_DXL, WHEEL , 0 , 0 ); delay(5);
    dxl_init(ID_DXR, WHEEL , 0 , 0 ); delay(5);
    dxl_init(ID1, SERVO, 1, 1024);delay(5);
   // dxl_init(ID2, SERVO, 1, 1024);delay(5);
   // dxl_init(ID3, SERVO, 1, 1024);delay(5);
    xmotor(0, 0, 2);
    xmotor(0, 0, 2);
    
    
    pinMode(port_buzzer, OUTPUT);
    pinMode(pin_pushbutton, INPUT_PULLUP);
    
    
    pinMode(port_channel_a, OUTPUT);
    pinMode(port_channel_b, OUTPUT);
    pinMode(port_channel_c, OUTPUT);
    
    
    pinMode(pin_sensor_line_a, INPUT);
    pinMode(pin_sensor_line_b, INPUT);
    
    sensor(front);
    buzzer(3,50,50);
    dnext();
    digitalWrite(port_enable_sensor,HIGH);
    dnext();
    
}
void xline_remote_init(){
    pinMode(port_enable_sensor,OUTPUT);
    digitalWrite(port_enable_sensor,LOW);
    
    motor_init();
    motor(0,0,1);
    
    pinMode(port_buzzer, OUTPUT);
    pinMode(pin_pushbutton, INPUT_PULLUP);
    
    
    pinMode(port_channel_a, OUTPUT);
    pinMode(port_channel_b, OUTPUT);
    pinMode(port_channel_c, OUTPUT);
    
    
    pinMode(pin_sensor_line_a, INPUT);
    pinMode(pin_sensor_line_b, INPUT);
    
    Wire.begin();
    Serial.begin(9600);
    
    sensor(front);
    buzzer(3,50,50);
    dnext();
    digitalWrite(port_enable_sensor,LOW);
    
}

//---------------------------------------------


