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

volatile int state = 0;                 //changes which case the switch function goes to
volatile int prevState=0;               //records last state (may not need for this)
volatile int leftGood = 1;              //flag is set when left sensor is within threshold
volatile int centerGood = 1;            //flag is set when center sensor is within threshold
volatile int rightGood = 1;             //flag is set when right sensor is within threshold
volatile int calibration = 0;           //keeps track of calibration count
volatile int thresh1 = 275;               //left sensor threshold -was 500
volatile int thresh2 = 1000;               //center sensor threshold
volatile int thresh3 = 475;               //right sensor threshold - was 800

// ******************************************************************************************* //

void checkSensors() {
    //check right
    int AD_value = 0;       //variable to store A/D buffer val
    AD1CHS = 0;         // positive input is AN0 (right sensor)
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value > thresh3) {   //if the right sensor is above the threshold value
        rightGood=1;            //set rightGood flag high
    }
    else {                      //if AD_value is not within threshold
        rightGood = 0;          //set rightGood flag low
    }

    //check center
    AD_value=0;             //reset AD_value to zero
    AD1CHS = 1;         // positive input is AN1 (center sensor)
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value < thresh2) {   //if the center sensor is below the threshold value
        centerGood=1;           //set the centerGood flag high
    }
    else {                      //if the center sensor is above threshold
        centerGood = 0;         //set ceterGood flag low
    }
    //check left
    AD_value = 0;           //reset AD_value
    AD1CHS = 3;         // positive input is AN5
    while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
    IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
    AD_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
    if (AD_value > thresh1) {   //if left threshold is above threshold
        leftGood=1;             //set leftGood flag high
    }
    else {                      //if left threshold is below threshold
        leftGood = 0;           //set leftGood flag low
    }
}

void calibrateSensors() {
    int ADC_value = 0;
    double oldADVal=-1;
    char value[8];
    //calibration
    LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
    LCDPrintString("Set Lim ");
    LCDMoveCursor(1,0);
    LCDPrintString("Press 1 ");
     while (calibration!=2);
     //calibrate sensor 1's dark value
       LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
       LCDPrintString("S1 Track");
       AD1CHS = 3;         // positive input is AN0
       while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
       IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
       ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
       sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
            if(oldADVal!=ADC_value){
               LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
               LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
               oldADVal=ADC_value;
               }
                   
       while (calibration != 3){
           ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
           sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
           if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
       }
       thresh1=ADC_value;      //darkval
       //calibrate sensor 1's light value
       LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
       LCDPrintString("S1 Floor");
       while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
       IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
       ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
       sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
           if(oldADVal!=ADC_value){
                LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
                oldADVal=ADC_value;
                }
                
        while(calibration != 4){
            ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
            sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
            if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
        }
        thresh1=(thresh1+ADC_value)/2;      //average of light and dark
        //calibrate sensor 2's dark value
        LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
        LCDPrintString("S2 Track");
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
        
        while (calibration != 5){
            ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
            sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
            if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
        }
        thresh2=ADC_value;
        //calibrate sensor 2's light value
        LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
        LCDPrintString("S2 Floor");
        while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
        IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
        ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
        sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
        while(calibration != 6){
            ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
            sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
            if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
        }
        thresh2=(thresh2+ADC_value)/2;      //average of light and dark
        //calibrate sensor 3's dark value
        LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
        LCDPrintString("S3 Track");
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
        while(calibration != 7) {
            ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
            sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
            if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
        }
        thresh3=ADC_value;
        //calibrate sensor 3's light value
        LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
        LCDPrintString("S3 Floor");
        while(!IFS0bits.AD1IF);  // wait while the A/D 1 interrupt flag is low
        IFS0bits.AD1IF = 0;     // clear the A/D 1 interrupt flag
        ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
        sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
        if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
         while(calibration != 8) {
             ADC_value = ADC1BUF0;   // stores the current value in the A/D 1 buffer in the ADC_value variable
             sprintf(value, "%6d", ADC_value); // formats value in ADC_value as a 6 character string and stores in in the value character array
             if(oldADVal!=ADC_value){
              LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
              LCDPrintString(value);              // sends value to the LCD print function to display it on the LCD screen
              oldADVal=ADC_value;
              }
         }
         thresh3=(thresh3+ADC_value)/2;
         //report calibration completion to user
         LCDClear();
         LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
         LCDPrintString("Set Lim");
         LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
         LCDPrintString("Done");
         DelayUs(10000000);
         LCDInitialize();
         state = 1;
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
    TRISBbits.TRISB1 = 1;
    AD1PCFGbits.PCFG3 = 0;

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
    AD1CHS = 1;         // positive input is AN0
    AD1CSSL = 0;        // low reference set to 0

    AD1CON1bits.ADON = 1; // A/D operating mode set to A/D converter module is operating
    IFS0bits.AD1IF = 0;   // clear the A/D 1 interrupt flag
    T3CONbits.TON = 1;  // Turn timer 3 on - VVVEEERRRYYY important for comparisons

    while(1)
    {
// Motor switching
        switch(state){

            //State 0: Idle Sate-wait for button press
            case 0:
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 0");
                LATBbits.LATB10=0;
                LATBbits.LATB11=0;
                while(calibration<1);
//                if (calibration < 8){
//                    calibrateSensors();
//                }
                LCDClear();   // (ch) added to see if it is always going back to 'glitch' state
                state=1;
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
                LCDClear();   // (ch) added to see if it is always going back to 'glitch' state
                state=2;
                break;
            //State 2: check problems
            case 2:
                LCDMoveCursor(0,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 2");
                OC1RS = 0;
                OC2RS = 0;
                //AD1CHS = 1;         // input is center sensor
                if ((leftGood==0)&&(centerGood==0)&&(rightGood==0)) {
//                   LCDMoveCursor(1,0);
//                   LCDPrintString("LB CB RB");
                    state = 0;
                }
                else if ((leftGood==0)&&(centerGood==0)&&(rightGood==1)) { //veer left
                    state = 3;
                }
                else if ((leftGood==0)&&(centerGood==1)&&(rightGood==0)) { //end of track
                    LCDMoveCursor(1,0);
                    LCDPrintString("LB CG RB"); // printed this
                    state = 0;
                }
                else if ((leftGood==0)&&(centerGood==1)&&(rightGood==1)) { //intersection
                    state = 6;
                }
                else if ((leftGood==1)&&(centerGood==0)&&(rightGood==0)) { //veer right
                    state = 4;
                }
                else if ((leftGood==1)&&(centerGood==0)&&(rightGood==1)) { //weird glitch
//                   LCDMoveCursor(1,0);
//                   LCDPrintString("LG CB RG");
                    state == 0;
                }
                else if ((leftGood==1)&&(centerGood==1)&&(rightGood==0)) {  //intersection
                    state == 5;
                }
                else if ((leftGood==1)&&(centerGood==1)&&(rightGood==1)) {  //should not get here
                    state == 1;
                }
                else {                                                      //should not get here
//                   LCDMoveCursor(1,0);
//                   LCDPrintString("ELSE    ");
                    state = 0;
                }
                break;

            case 3: //car off track
                LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 3");
                while ((centerGood!=1)&&(leftGood!=1)){
                    OC1RS = 0;
                    OC2RS = PR3*.667;
                    checkSensors();
                }
                state = 1;
                break;

            case 4: //car off track
                LCDMoveCursor(1,0);                 // moves the cursor on the LCD to the home position
                LCDPrintString("State 4");
                while (centerGood!=1){
                    OC1RS = PR3*.667;
                    OC2RS = 0;
                    checkSensors();
                }
                state = 1;
                break;
            case 5: //turn right
                LCDMoveCursor(1,0);
                LCDPrintString("State 5");
                while (rightGood!=1) {
                    OC1RS=0;
                    OC2RS=PR3*.667;
                    checkSensors();
                }
                state = 1;
                break;

            case 6: //turn left
                LCDMoveCursor(1,0);
                LCDPrintString("State 6");
                while (leftGood!=1) {
                    OC1RS=PR3*.667;
                    OC2RS=0;
                    checkSensors();
                }
                state = 1;
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
    if(state == 0) {
        calibration=calibration+1;
    }
    if (state ==6) {
        state=0;
    }
        
}

