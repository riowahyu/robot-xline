//
//  xline.hpp
//  
//
//  Created by rodik wahyu indrawan on 21/06/2016.
//  rodikwahyuindrawan@gmail.com
//

#ifndef xline_h
#define xline_h

#include <stdio.h>
#include "Dynamixel_Serial.h"
#include "Wire.h"
#include "PWM.h"
#include "Servo.h"
#include "Arduino.h"


#define pin_sensor_line_a 4
#define pin_sensor_line_b 2
#define port_channel_a A1
#define port_channel_b A2
#define port_channel_c A0
#define port_enable_sensor A3

void line_follow_set_speed(int);
void line_follower();


#define ID1 1
#define ID2 2
#define ID3 3

#define PD1 A7
#define PD2 A6
#define PD3 10
#define PD4 9
#define PD5 8

#define MRB 1
#define MGR 2
#define MDX 3

/*
 // setting position 
#define dsetopen  260
#define dsetclose 450
#define dsetup    100
#define dsetdown1 500
#define dsetdown2 450
*/


#define ff 1
#define fl 2
#define fr 3
#define fl3 4
#define fr3 5
#define fl4 6
#define fr4 7
#define sl 8
#define sl3 9
#define sr 10
#define sr3 11
#define sf 12


#define on 1
#define off 2

#define ll 2
#define LL 2
#define cc 1

#define rr 3
#define RR 3
#define CC 1

#define s1 1
#define s2 3
#define s3 5
#define s4 11
#define s5 13
#define s6 17
#define s7 19
#define s8 23

#define FF 1
#define FL 2
#define FR 3
#define FL3 4
#define FR3 5
#define FL4 6
#define FR4 7
#define SL 8
#define SL3 9
#define SR 10
#define SR3 11

#define black 1
#define white 2
#define front 1
#define rear 2

#define BLACK 1
#define WHITE 2
#define FRONT 1
#define REAR 2

#define next dnext();
#define next1 dnext1();
#define xcf xcenterfront();

void motor_type(int);
void tracemode(char);

void set_xclose(int dpos, int dspeed);
void set_xopen(int dpos, int dspeed);
void set_xup(int dpos, int dspeed);
void set_xdown1(int dpos, int dspeed);
void set_xdown2(int dpos, int dspeed);

#define SLF1 sl_front_1()
#define SLF2 sl_front_2()
#define SLF3 sl_front_3()
#define SLF4 sl_front_4()
#define SLF5 sl_front_5()
#define SLF6 sl_front_6()
#define SLF7 sl_front_7()
#define SLF8 sl_front_8()

#define SLR1 sl_rear_1()
#define SLR2 sl_rear_2()
#define SLR3 sl_rear_3()
#define SLR4 sl_rear_4()
#define SLR5 sl_rear_5()
#define SLR6 sl_rear_6()
#define SLR7 sl_rear_7()
#define SLR8 sl_rear_8()


char sl_front_1();
char sl_front_2();
char sl_front_3();
char sl_front_4();
char sl_front_5();
char sl_front_6();
char sl_front_7();
char sl_front_8();

char sl_rear_1();
char sl_rear_2();
char sl_rear_3();
char sl_rear_4();
char sl_rear_5();
char sl_rear_6();
char sl_rear_7();
char sl_rear_8();


void xline_init();
void dxline_init();
void read_sl_front();
void dxlsetid(int);
void dxl_init(int, int , int ,int );
void dxlservo(int, int, int);
void dxlwheel(int , int );

void drive_line(char dkd,char dsd,char dsb ); //kp,delay stop trigger,break mundur
void drive_turn(int,int ); //don,doff

void xdrive_line(char dkd,char dsd);

void init_srf05();
void init_gp2d12();
void init_gp2d120();

void servo_init();
void servo3(int, int);
void servo4(int, int);
void servo5(int, int);


void ss();
void sensor(char);
void linecolor(char);
void buzzer(int, int, int);

void motor_init();
void motor(int , int , int);
void tmotor(int , int , int);
void dmotor(int , int);
void d_motor(int , int );

void xmotor(int , int , int);
void dxmotor(int , int);
void line_scan_front(int);

void line_scan_front_p(int);
void robo_line(int, int, int);
void linewide(int);
void xline_scan_front(int);
void xline_scan_front3(int);
void xcenterfront();
void xline(int, int, int);
void centerfront();
void line(int, int, int);
void linep(int, int, int);
void linedelay(int, int, int);
void xlinedelay(int, int, int);
void linedelay_p(int, int, int);
void linedelay_l(int, int, int);
void findline(int, int, int);
void xfindline(int, int, int);
void lostline(int, int);
void xlostline(int, int);


void stop_end();

void linejump(int , int );

void transporter_init();
void take(int dspeed, int dbreaktime);
void up();
void down1();
void down2();
void down3();
void dopen();
void dclose();
void doServo(int id, int dpos);
void dservo(int id, int d_start, int d_target, int d_speed, int d_time);
void dservo2(int id1, int d_start1, int d_target1, int id2, int d_start2, int d_target2, int d_speed, int d_time);
void put(int speed,int dist, int layer);

void putz(int speed,int dist, int layer);

void xup(int);
void xdown1(int);
void xdown2(int);
void xopen();
void xclose();
void xtake(int dspeed, int dbreaktime);
void xput(int speed,int dist, int layer,int dbreaktime);
void xputz(int speed,int dist, int layer, int dbreaktime);
void xputs(int speed,int dist, int layer, int dbreaktime);

void set_up(int, int);
void set_down1(int, int);
void set_down2(int, int);
void set_down3(int, int);
void set_open(int, int, int, int);
void set_close(int, int, int, int);

void xleft(int, int, int);
void xleft1(int, int, int);
void xleft2(int, int, int);
void xleft3(int, int, int);
void xleft4(int, int, int);
void xleft5(int, int, int);
void xleft6(int, int, int);
void xleft7(int, int, int);
void xleft8(int, int, int);

void xright(int, int, int);
void xright1(int, int, int);
void xright2(int, int, int);
void xright3(int, int, int);
void xright4(int, int, int);
void xright5(int, int, int);
void xright6(int, int, int);
void xright7(int, int, int);
void xright8(int, int, int);

void left(int, int, int);
void left1(int, int, int);
void left2(int, int, int);
void left3(int, int, int);
void left4(int, int, int);
void left5(int, int, int);
void left6(int, int, int);
void left7(int, int, int);
void left8(int, int, int);
void right(int, int, int);
void right1(int, int, int);
void right2(int, int, int);
void right3(int, int, int);
void right4(int, int, int);
void right5(int, int, int);
void right6(int, int, int);
void right7(int, int, int);
void right8(int, int, int);

void sturn(int, int, int,int );
void sline(int , int , int );
void dnext();

void xline_remote_init();
void navigate_maneuver_5(int , int );
void navigate_maneuver_4(int , int );
void navigate_maneuver_2(int , int );
void navigate_maneuver_1(int , int );
void sequencial_command_1(int dLF, int dRF, int dTF,
                          int dLR, int dRR,int dTR,
                          int dLB, int dRB,int dTB,
                          int dLL, int dRL,int dTL);
void sequencial_command_2(int dLF, int dRF, int dTF,
                          int dLR, int dRR,int dTR,
                          int dLB, int dRB,int dTB,
                          int dLL, int dRL,int dTL);
void linemaze_command(int dLM, int dLS,int dLBR,
                      int dRM, int dRS,int dRBR,
                      int dLL, int dLR,int dLB,
                      int dRL, int dRR,int dRB);
void read_psx();
void display_psx();

#endif /* xline_hpp */
