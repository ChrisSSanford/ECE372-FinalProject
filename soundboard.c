/**************************************************************************************************/

/*
 * File: soundboard.c
 * Team: Lambda^3
 * Members: Chris Houseman
 *          Randy Martinez
 *          Rachel Powers
 *          Chris Sanford
 *
 * Date: October 2, 2014
 *
 * Description: Code that defines LCD functions
 *
 */

// ******************************************************************************************* //
// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include "soundboard.h"
#include "lcd.h"    //NEED4FP
// ******************************************************************************************* //
#define SB_RESET  LATBbits.LATB4        //soundboard pin 1 is connected to starter board pin 11
#define SB_CLOCK  LATAbits.LATA4        //soundboard pin 7 is connected to starter board pin 12
#define SB_DATA   LATBbits.LATB8        //soundboard pin 10 is connected to starter board pin 17
#define SB_BUSY   LATBbits.LATB9        //soundboard pin 15 is connected to starter board pin 18

#define SB_TRIS_RESET  TRISBbits.TRISB4
#define SB_TRIS_CLOCK  TRISAbits.TRISA4
#define SB_TRIS_DATA   TRISBbits.TRISB8
#define SB_TRIS_BUSY   TRISBbits.TRISB9
// ******************************************************************************************* //
//#define SB_RESET  LATBbits.LATB15        //soundboard pin 1 is connected to starter board pin 11
//#define SB_CLOCK  LATBbits.LATB14        //soundboard pin 7 is connected to starter board pin 12
//#define SB_DATA   LATBbits.LATB12        //soundboard pin 10 is connected to starter board pin 17
//#define SB_BUSY   LATBbits.LATB13        //soundboard pin 15 is connected to starter board pin 18
//
//#define SB_TRIS_RESET  TRISBbits.TRISB15
//#define SB_TRIS_CLOCK  TRISBbits.TRISB14
//#define SB_TRIS_DATA   TRISBbits.TRISB12
//#define SB_TRIS_BUSY   TRISBbits.TRISB13
// ******************************************************************************************* //
volatile unsigned int PLAY_PAUSE = 0xFFFE;
volatile unsigned int STOP = 0xFFFF;
volatile unsigned int VOLUME_MIN = 0xFFF0;
volatile unsigned int VOLUME_MAX = 0xFFF7;
// ******************************************************************************************* //
void Delayms(unsigned int msDelay) {

    T1CONbits.TON = 0;  // Turn timer 2 off
    T1CONbits.TCS=0; // sets up to use internal clock
    T1CONbits.TGATE = 0;
    IFS0bits.T1IF=0;  // reset timer 2 interrupt flag
    TMR1=0;           // resets timer 2 to 0

    T1CONbits.TCKPS1 = 1;   //prescale of 256
    T1CONbits.TCKPS0 = 1;
    PR1 =(unsigned int) (56*msDelay);  // (1ms)(14745600/8)-(1) = 0.8432

    T1CONbits.TON = 1; // start timer 2
    while (IFS0bits.T1IF == 0); // delay until the timer finishes

    T1CONbits.TON = 0; // Turn timer 2 off

/*****************************************************/
}
void SBInitialize(void) {
     SB_TRIS_RESET = 0;	// Reset is output
     SB_TRIS_CLOCK = 0;	// Clock is output
     SB_TRIS_DATA  = 0;	// Data is output
     SB_TRIS_BUSY  = 1;	// Busy is input

     SB_CLOCK=1;
     SB_DATA=1;
     SB_RESET=1;

}
//**************************************************************************** //
void SBReset(){
  SB_CLOCK = 0;
  SB_RESET = 1;
  //Reset pulse.
  SB_RESET = 0;
  Delayms(5);
  SB_RESET = 1;
  //Reset idle to start bit.
  SB_CLOCK=1;
  Delayms(300);
}
//**************************************************************************** //
void SBPlayVoice(int voiceNumber){
  SBSendCommand(voiceNumber);
  while(PORTBbits.RB9==1);
}
//**************************************************************************** //
void SBAsyncPlayVoice(int voiceNumber){
  SBSendCommand(voiceNumber);
}
//**************************************************************************** //
void SBStopVoice(){
  SBSendCommand(STOP);
}
//**************************************************************************** //
void SBPauseVoice() {
  SBSendCommand(PLAY_PAUSE);
}

void SBMute(){
  SBSendCommand(VOLUME_MIN);
}

void SBUnmute(){
  SBSendCommand(VOLUME_MAX);
}

//**************************************************************************** //
void SBSendCommand(unsigned int command){
    unsigned int mask = 0x8000;
    int i = 0;
    SB_CLOCK=0;
    Delayms(2);
    for (i=16; i > 0; --i) {
        SB_CLOCK=0;
        DelayUs(100);
        if ((command & mask)==0x8000) {
            SB_DATA=1;
        }
        else {
            SB_DATA=0;
        }
        command <<= 1;
        DelayUs(100);
        //DelayUs(200);
        SB_CLOCK=1;
        DelayUs(200);
        //DelayUs(200);
//        if (mask>0x0001) {
//            Delayms(2);
//        }
    }
    SB_CLOCK=1;
    SB_DATA=1;
    Delayms(20);
}