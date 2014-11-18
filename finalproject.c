/**************************************************************************************************/

/*
 * File: finalproject.c
 * Team: Lambda^3
 * Members: Chris Houseman
 *          Randy Martinez
 *          Rachel Powers
 *          Chris Sanford
 *
 * Date: November 14, 2014
 *
 * Description: Code that allows robot to follow line
 *
 */

// ******************************************************************************************* //
// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"
#include <stdlib.h>

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //

// variables that are accessed in multiple functions

volatile int state = 0;
volatile int prevState=0;
volatile int leftGood = 1;
volatile int centerGood = 1;
volatile int rightGood = 1;
volatile int calibration = 0;
volatile int thresh1 = 0;
volatile int thresh2 = 0;
volatile int thresh3 = 0;

// ******************************************************************************************* //

void checkSensors() {
    //check right
    int AD_value = 0;
    AD1CHS = 0;         // positive input is AN0
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value > thresh3) {
        rightGood=1;
    }
    else {
        rightGood = 0;
    }

    //check center
    AD_value=0;
    AD1CHS = 1;         // positive input is AN0
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value < thresh2) {
        centerGood=1;
    }
    else {
        centerGood = 0;
    }
    AD_value = 0;
    TRISBbits.TRISB3 = 1;
    AD1PCFGbits.PCFG5 = 0;
    AD1CHS = 5;         // positive input is AN0
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value > thresh1) {
        leftGood=1;
    }
    else {
        leftGood = 0;
    }
}


/*
 * 
 */
int main(void) {

 /**********************************************/
    //variables that store what percentage the motors are turned on to (Motor Control)
    double oldADVal=-1;


/**********************************************/
    //use timer for PWM (Motor Control)
    T3CONbits.TCS = 0; // sets up to use internal clock 
    T3CONbits.TGATE = 0; 
    T3CONbits.TON = 0;  // Turn timer 3 off
    TMR3 = 0;           // resets timer 3 to 0

    T3CONbits.TCKPS = 3; // set a prescaler of 8 for timer 2
    PR3 = 575;


/*****************************************************/
    
    //output compare stuff (motor control)
    OC1CONbits.OCM = 6; // Initialize OCx pin low, compare event forces OCx pin high,
    OC1CONbits.OCTSEL = 1; // using timer 3

    OC1R = OC1RS = PR3;
    OC1RS=PR3/2; //TODO: Take out for battery

    OC2CONbits.OCM = 6; // Initialize OCx pin low, compare event forces OCx pin high,
    OC2CONbits.OCTSEL = 1; // using timer 3

    OC2R = OC2RS = PR3;
    OC2RS=PR3/2;

    //Ports used for output to H-bridge
    //PWM outputs
    RPOR0bits.RP0R = 18;    //10010 - OC1 (Output Compare 1)
    RPOR1bits.RP2R = 19;    //10011 - OC2 (Output Compare 2)
/*****************************************************/
    //pins that control which direction the motors turn (motor control)
    TRISBbits.TRISB11 = 0;
    LATBbits.LATB11 = 1;
    CNPU1bits.CN15PUE = 1;

    TRISBbits.TRISB10 = 0;
    LATBbits.LATB10 = 1;
    CNPU2bits.CN16PUE = 1;

//Sensor (inputs)
    //sensor left 3
    TRISBbits.TRISB3 = 1;
    AD1PCFGbits.PCFG5 = 0;

    //sensor center 2
    TRISAbits.TRISA1 = 1;
    AD1PCFGbits.PCFG1 = 0;

    //sensor right 1
    TRISAbits.TRISA0 = 1;
    AD1PCFGbits.PCFG0 = 0;

/*****************************************************/
    //determines which state the car is in
// Configure TRIS register bits for switch 1 input
	TRISBbits.TRISB5 = 1;

// Configure CN register bits to enable change notifications for switch input.
	CNEN2bits.CN27IE = 1;
        IFS1bits.CNIF = 0;
        IEC1bits.CNIE = 1;
/*****************************************************/

    int ADC_value;      // variable to store the binary value in the ADC buffer
    char value[8];      //  character array to store the values to be printed to the LCD
    //double AD_value;    // variable to store the calculated value of the voltage

    LCDInitialize();  // initialize the LCD display

    //AD1PCFG &= 0xFFDF; // Pin 7, AN5, where the POT is connected, IO6, is set to analog mode, AD module samples pin voltage
    AD1CON2 = 0x0;       // Always uses MUX A input multiplexer settings, configured as one 16-word buffer, interrupts at the completion of conversion for each sample/convert sequence, use the channel selected by the CH0SA bits as the MUX A input
    AD1CON3 = 0x0101;      //set the A/D conversion clock period to be 2*Tcy, set the Auto-Sample Time bits to be 1 T_AD, A/D conversion clock derived from system clock
    AD1CON1 = 0x20E4;   // A/D sample auto-start mode set for sampling begins immediately after last conversion completes, SAMP bit is automatically set, Conversion trigger source set to internal counter (auto-convert), data output format is integer, stop in idle mode set to discontinue module operation when device enters idle mode
    AD1CHS = 5;         // positive input is AN0
    AD1CSSL = 0;        // low reference set to 0

    AD1CON1bits.ADON = 1; // A/D operating mode set to A/D converter module is operating
    IFS0bits.AD1IF = 0;   // clear the A/D 1 interrupt flag
    T3CONbits.TON = 1;  // Turn timer 3 on - VVVEEERRRYYY important for comparisons

    while(1)
    {
        while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
        IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
        ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
        sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
        if(oldADVal!=ADC_value){
            LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
            LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
            oldADVal=ADC_value;
        }
        //AD_value = (ADC_value * 3.3)/1024;  // converts the binary value of the voltage to the analog value by multiplying by the maximum voltage and dividing by 2^n = 2^10, then stores it in AD_value

// Motor switching
        switch(state){

            //State 0: Idle Sate-wait for button press
            case 0:
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 0");
                LATBbits.LATB10=0;
                LATBbits.LATB11=0;
                break;
            //State 1: Drive forward
            case 1:
                //AD1CHS = 0;
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 1");
                OC1RS = PR3;
                OC2RS = PR3;
                LATBbits.LATB10=1; 
                LATBbits.LATB11=0;
                checkSensors();
                while ((rightGood==1)&&(centerGood==1)&&(leftGood==1)){
                    checkSensors();
                }
                state=2;
                break;
            //State 2: Drive backwards
            case 2:
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 2");
                //AD1CHS = 1;         // input is center sensor
                if ((rightGood==1)&&(centerGood==0)&&(leftGood==0)) { //turn left
                    state = 3;
                }
                else if ((rightGood==1)&&(centerGood==0)&&(leftGood==1)){ //also turn right
                    state = 3;
                }
                else if ((rightGood==0)&&(centerGood==0)&&(leftGood==1)) { //turn right
                    state = 4;
                }
                else {
                    state = 1;
                }
                break;
            case 3: //car off track
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 3");
                while (centerGood!=1){
                    checkSensors();
                    OC2RS = PR3/2;
                    OC1RS = 0;
                }
                state = 1;
                break;

            case 4: //car off track
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 4");
                while (centerGood!=1){
                    checkSensors();
                    OC2RS = 0;
                    OC1RS = PR3/2;
                }
                state = 1;
                break;
            case 20:    //calibration
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("Set Lim");
                LCDMoveCursor(1,0);
                LCDPrintString("Press 1");
                if (calibration==1) {
                   LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                   LCDPrintString("S1 Blk");
                   AD1CHS = 5;         // positive input is AN0
                   while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                   IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                   ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                   sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                   if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 2){
                    thresh1=ADC_value;      //darkval
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("S1 Lt");
                    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                    ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                    sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                    if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 3) {
                    thresh1=(thresh1+ADC_value)/2;      //average of light and dark
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("S2 Blk");
                    AD1CHS = 1;         // positive input is AN0
                    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                    ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                    sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                    if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 4) {
                    thresh2=ADC_value;
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("S2 Lt");
                    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                    ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                    sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                    if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 5) {
                    thresh2=(thresh2+ADC_value)/2;      //average of light and dark
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("S3 Blk");
                    AD1CHS = 0;         // positive input is AN0
                    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                    ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                    sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                    if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 6) {
                    thresh3=ADC_value;
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("S3 Lt");
                    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
                    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
                    ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
                    sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
                    if(oldADVal!=ADC_value){
                        LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                        LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                        oldADVal=ADC_value;
                        }
                    }
                if (calibration == 7) {
                    thresh3=(thresh3+ADC_value)/2;
                    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("Set Lim");
                    LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                    LCDPrintString("Done");
                    DelayUs(1000);
                    LCDInitialize();
                    state = 1;
                }
                break;
        }

    }
return 0;
}


void __attribute__((interrupt,auto_psv)) _CNInterrupt(void)
{
    // Clear CN interrupt flag to allow another CN interrupt to occur.

    while (PORTBbits.RB5==0);
    IFS1bits.CNIF = 0;
    if((state == 0)&&(calibration==7)) {
        state = 1;
    }
    else if ((state == 0)&&(calibration == 0)) {
        calibration=1;
        state = 20; //calibration state is last case
    }
    else if (state == 20) {
        calibration=calibration+1;
    }
        
}

