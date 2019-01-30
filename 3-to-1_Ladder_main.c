//###########################################################################
//
// FILE:    HRPWM_PrdUpDown_SFO_V8.c
//
// TITLE:   F2837xD Device HRPWM SFO V8 High-Resolution Period (Up-Down Count)
//          example
//
//###########################################################################
//changed linker file!!!
//C:\ti\controlSUITE\device_support\F2837xD\v210\F2837xD_common\cmd\2837xD_RAM_lnk_cpu1.cmd


//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"
#include "SFO_V8.h"
#include "ringbuffer.h"
#include "string.h"
//#include "stdio.h"
#include "stdlib.h"
#include "F2837xD_gpio.h"
#include <sgen.h>

//
// Defines

//Serial commands
//#define

#define SYSCLK 200e6
#define EPWMCLK 100e6

#define F_PWM 200e3
#define PWM_CH            9        // # of PWM channels


//249 => 200kHz; 124 => 400kHz
#define PWM_PERIOD_TICKS  (EPWMCLK/(2 * F_PWM) -1)      //required -1!

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define RX_PACKAGE_SIZE 20

#define DEBUG_PIN1 161
#define DEBUG_PIN2 163

float32 T_PWM = 1/F_PWM;

ringbuffer_t TXBufferStruct;
ringbuffer_t RXBufferStruct;

Uint16 TXBuffer[TX_BUFFER_SIZE];
Uint16 RXBuffer[RX_BUFFER_SIZE];
Uint16 RxPackage[RX_PACKAGE_SIZE];
Uint16 RxCommand[24];
Uint16 RxData[24];

//Uint16 countReceive = 0;
//Uint16 testReceive[100];
//Uint16 countSend = 0;
//Uint16 testSend[100];


//
// Globals
//
int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO(0) function.


Uint16 adcStartDelayClk = 50;
Uint16 updateDB = 0;
Uint16 deadBand = 10;
Uint16 status;
Uint16 newCommandRdy = 0;
Uint16 i = 0;
Uint16 rampUp = 0;
Uint16 rampDown = 0;


//Uout control loop values
float32 KP_PHI = 0.05;
float32 KI_PHI = 6;
float32 ADC_UC2_GAIN = 0.003659;
float32 ADC_UC2_OFFSET = -0.017504;

float32 eSumPhi = 0;
float32 uOutTarget = 4.0;
float32 ePhi = 0;
float32 phi = 0;
float32 uOut = 0;
float32 upperPhiLimit = 0;


//C2 control loop values
float32 KP_DPHI = 0.01;
float32 KI_DPHI = 0.5;
float32 ADC_UOUT_GAIN = 0.003656;
float32 ADC_UOUT_OFFSET =0.002825;

float32 eSumdPhi = 0;
float32 uC2Target = 4.0;
float32 edPhi = 0;
float32 dPhi = 0;
float32 uC2 = 0;
float32 upperLowerdPhiLimit = 0;

float32 antiWindupMax = 5000;
float32 DACgain = 1024;
float32 DACoffset = 2048;

float32 phiTemp = 0;
float32 dPhiTemp = 0;

float32 utemp = 0;
Uint16 delayTemp = 0;


// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
   &EPwm6Regs, &EPwm7Regs, &EPwm8Regs};


// Function Prototypes
void HRPWM1_Config(int);
void HRPWM2_Config(int);
void HRPWM3_Config(int);
void HRPWM4_Config(int);
void HRPWM5_Config(int);            //for ADC SOC
void DAC_A_Config(void);
void reverseString(uint16_t *str, uint16_t len);
uint16_t intToStr(uint16_t x, uint16_t str[], uint16_t d);
void floatToArray(float32 n, Uint16 *res, Uint16 afterpoint);
void HRPWMupdatePhases(float32 phaseShift1_3, float32 phaseShift3_4);
void HRPWMupdateDB(int deadBand);
void ConfigureADC(void);
interrupt void adca0_isr(void);
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);

void error(void);



SGENTI_1 sgen=SGENTI_1_DEFAULTS;

//
// Main
//
void main(void){
    // Initialize System Control for Control and Analog Subsystems
    // Enable Peripheral Clocks
    InitSysCtrl();

    //init debug pin
    InitGpio();
    GPIO_SetupPinMux(DEBUG_PIN1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG_PIN1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(DEBUG_PIN2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG_PIN2, GPIO_OUTPUT, GPIO_PUSHPULL);


    // EPWM1A and EPWM1B through PWM1-3
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

    DINT; // Disable CPU interrupts
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Map ISR functions
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ADCB1_INT = &adca0_isr;        //function for ADC-B interrupt 1
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  //ISR for RX interrupt
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;  //ISR for TX interrupt
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // configure Serial
    scia_fifo_init();

    // Configure the ADC and power it up
    ConfigureADC();

    // configuarion of the DAC
    DAC_A_Config();



    // enable PIE interrupts
//  PieCtrlRegs.PIEIER1.bit.INTx1  = 1;     //enable interrupt for ADC A

    //enable ADC interrupt after voltage startup
    //if from beginning on => controller will be go to max phase shift
    //max phaseshift at startup will result in high current
    //can partial compensated with dPhi
    PieCtrlRegs.PIEIER1.bit.INTx2  = 1;     //enable interrupt for ADC B

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // PIE Group 9, INT1, TX interrupt
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // PIE Group 9, INT2, RX interrupt

    IER |= M_INT9;                          // Enable CPU INT group 9 (RX/TX)
    IER |= M_INT1;                          // Enable CPU INT group 1 (ADC)



    // Enable global Interrupts and higher priority real-time debug events
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM




   status = SFO_INCOMPLETE;


    // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
    // HRMSTEP must be populated with a scale factor value prior to enabling
    // high resolution period control.
    //
    while(status == SFO_INCOMPLETE){ // Call until complete
        status = SFO();
        if (status == SFO_ERROR){
            error();    // SFO function returns 2 if an error occurs & # of MEP
        }               // steps/coarse step exceeds maximum of 255.
    }


    EALLOW;
    InputXbarRegs.INPUT5SELECT = 0xA;   //Input5 is normally connected to GPIO0 which is PWM1 output-> every PWM cycle reset
                                        //use not as input
                                        //GPIO10 -> INPUT5 -> EXTSYNCIN1 -> Sync singal for PWM modules (S. 1737)

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the EPWM
    EDIS;
    //PWM frequence EPWMCLK = 100MHz (maximum)
    HRPWM1_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM2_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM3_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM4_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM5_Config(PWM_PERIOD_TICKS);   // ADC SOC
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;      // Enable TBCLK within the EPWM
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;          // Synchronize high resolution phase to start HR period
    EDIS;

    //init RX/TX buffer
    TXBufferStruct.buffer = TXBuffer;
    RXBufferStruct.buffer = RXBuffer;
    TXBufferStruct.size = TX_BUFFER_SIZE;
    RXBufferStruct.size = RX_BUFFER_SIZE;
    ringbuffer_reset(&TXBufferStruct);
    ringbuffer_reset(&RXBufferStruct);




//    Uint16 a = 0;
//    for(a=0; a<100; a++)
//    {
//        testSend[a] = '\0';
//    }



    sgen.offset=sgen.offset = 0 * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF (-1 to 1);
    sgen.gain=sgen.gain = 1 * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF (0 to 1)
    sgen.freq=5369; /* freq = (Required Freq/Max Freq)*2^15 */
    /* = (50/305.17)*2^15 = 5369 */
    sgen.step_max=1000; /* Max Freq= (step_max * sampling freq)/65536 */
    /* Max Freq = (1000*20k)/65536 = 305.17 */



    for(;;){

        GPIO_WritePin(DEBUG_PIN2, 1);
//        DELAY_US(1000);

        //for start up

        //check RX buffer for new command
        if(!ringbuffer_empty(&RXBufferStruct)){
            ringbuffer_get(&RXBufferStruct, &RxPackage[i]); //copy character


//            testReceive[countReceive] = RxPackage[i];
//            countReceive += 1;


            if(RxPackage[i] == 0x000D){                      //enter command is received => command complett
                Uint16 backspacePos = 0;
                Uint16 j = 0;
                for(j = 0; j < sizeof(RxData); j++) RxData[j] = 0;    //clear buffer
                for(j = 0; j < sizeof(RxCommand); j++) RxCommand[j] = 0;    //clear buffer

                for(j = 0; j < i; j++){
                    if(RxPackage[j] == 0x0020){              //find backspace in RxPackage
                        backspacePos = j;
                    }
                    if(backspacePos > 0){
                        RxData[j-backspacePos] = RxPackage[j+1];
                    }
                    else{
                        RxCommand[j] = RxPackage[j];
                    }
                }
                newCommandRdy = 1;

            }
            i++;
        }

        if(newCommandRdy > 0){  //received command ready for execution

            for(i = 0; i < RX_PACKAGE_SIZE; i++){     //clear buffer
                RxPackage[i] = 0;
            }
            i = 0;

            //send kp phi
            if(memcmp(RxCommand, "getkpphi", strlen("getkpphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(KP_PHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //send ki phi
            if(memcmp(RxCommand, "getkiphi", strlen("getkiphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(KI_PHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //send kp dphi
            if(memcmp(RxCommand, "getkpdphi", strlen("getkpdphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(KP_DPHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //send ki dphi
            if(memcmp(RxCommand, "getkidphi", strlen("getkidphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(KI_DPHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            
            //send Uout target
            if(memcmp(RxCommand, "getuouttarget", strlen("getuouttarget")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(uOutTarget, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            
            //send UC2 target
            if(memcmp(RxCommand, "getuc2target", strlen("getuc2target")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //sending phi
                floatToArray(uC2Target, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            
            //send dead band
            if(memcmp(RxCommand, "getdeadband", strlen("getdeadband")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;



                //sending phi
                intToStr(deadBand * 5, sendData, 1);            // *5 to get ns
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            
            //get all parameter at once
            if(memcmp(RxCommand, "getdata", strlen("getdata")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;


//                testSend[countSend] = 1;
//                countSend += 1;

                //sending phi
                floatToArray(phi, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending dPhi
                floatToArray(dPhi, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending uOut
                floatToArray(uOut, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending uOutTarget
                floatToArray(uOutTarget, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending uC2
                floatToArray(uC2, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending uC2Target
                floatToArray(uC2Target, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //sending deadBand
                intToStr(deadBand * 5, sendData, 1);            //*5 to get ns
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
//                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set kp phi
            if(memcmp(RxCommand, "setkpphi", strlen("setkpphi")) == 0){
                KP_PHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set ki phi
            if(memcmp(RxCommand, "setkiphi", strlen("setkiphi")) == 0){
                KI_PHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set kp dphi
            if(memcmp(RxCommand, "setkpdphi", strlen("setkpdphi")) == 0){
                KP_DPHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set ki dphi
            if(memcmp(RxCommand, "setkidphi", strlen("setkidphi")) == 0){
                KI_DPHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set deadband
            if(memcmp(RxCommand, "setdeadband", strlen("setdeadband")) == 0){
                float32 deadBand_ns = atof(RxData);
                deadBand = round(deadBand_ns / 5);
                HRPWMupdateDB(deadBand);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set Uout target
            if(memcmp(RxCommand, "setuouttarget", strlen("setuouttarget")) == 0){
                uOutTarget = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }

            //set UC2 target
            if(memcmp(RxCommand, "setuc2target", strlen("setuc2target")) == 0){
                uC2Target = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n\r", 4);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            newCommandRdy = 0;
        }


        GPIO_WritePin(DEBUG_PIN2, 0);
 //       DELAY_US(100000); //delay for 1ms to allow ADC time to power up
    } // end infinite for loop
}

// Converts a floating point number to string.
void floatToArray(float32 n, Uint16 *res, Uint16 afterpoint){
    // Extract integer part
    Uint16 ipart = (Uint16)n;

    // Extract floating part
    float32 fpart = n - (float)ipart;

    // convert integer part to string
    Uint16 i = intToStr(ipart, res, 1);

    // check for display option after point
    if (afterpoint != 0){
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint) + 0.5;
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

// reverses a string 'str' of length 'len'
void reverseString(uint16_t *str, uint16_t len){
    uint16_t i=0, j=len-1, temp;
    while (i<j){
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
uint16_t intToStr(uint16_t x, uint16_t str[], uint16_t d){
    uint16_t i = 0;
    while (x){
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d){
        str[i++] = '0';
    }
    reverseString(str, i);
    str[i] = '\0';
    return i;
}



void ConfigureADC(void){

    //Configure ADC A for input A0
    //Configure start of conversion block 0 (SOC0) to pin A0 (Datasheet S.92, Figure 2-2)
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is 100 SYSCLK cycles

    //trigger ADC by PWM1
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 6; // ePWM1, ADCSOCB


    // Setup the post-processing block 1 to be associated with SOC0 EOC0
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 0; //Set pulse positions to end of conversion
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    //power up the ADC

    //Configure ADC B for input B0
    //Configure start of conversion block 0 (SOC0) to pin A1 (Datasheet S.92, Figure 2-2)
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 6; // ePWM1, ADCSOCB

    // Setup the post-processing block 1 to be associated with SOC0 EOC0
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 0; //Set pulse positions to end of conversion
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;    //power up the ADC

    DELAY_US(1000); //delay for 1ms to allow ADC time to power up
    EDIS;
}

//
// adca1_isr - ADCA1 Interrupt Service Routine
//
interrupt void adca0_isr(void){
    GPIO_WritePin(DEBUG_PIN1, 1);

    //start up (turn on control loop)
    if(rampUp == 1){
        eSumPhi = 0;
        if(upperPhiLimit <= 6){
            upperPhiLimit =upperPhiLimit + 0.00001;
        }
        else{
            rampUp = 0;
            upperPhiLimit = 6;
            upperLowerdPhiLimit = 2;
        }
    }

    //turn off (turn off control loop)
    if(rampDown == 1){
        if(upperPhiLimit > 0){
            upperPhiLimit = upperPhiLimit - 0.00001;
        }
        else{
            rampDown = 0;
            upperPhiLimit = 0;
            upperLowerdPhiLimit = 0;
        }
    }

    //control loop for output voltage----------------------------------
    uOut = AdcbResultRegs.ADCRESULT0 * ADC_UOUT_GAIN + ADC_UOUT_OFFSET;
    utemp = uOut;
    ePhi = uOutTarget - uOut;
    eSumPhi += ePhi;

    //limitation
    if(eSumPhi < -antiWindupMax) eSumPhi = -antiWindupMax;
    if(eSumPhi > antiWindupMax)  eSumPhi =  antiWindupMax;

    phi = (KP_PHI * ePhi + T_PWM * eSumPhi * KI_PHI) * 100;     //*100 for percent

    //limitation of control variable
    if(phi < 0)  phi = 0;
    if(phi > upperPhiLimit) phi = upperPhiLimit;
    //-------------------------------------------------------------------

    //control loop for C2 voltage-----------------------------------------
    uC2 = AdcaResultRegs.ADCRESULT0 * ADC_UC2_GAIN + ADC_UC2_OFFSET;
    edPhi = uC2Target - uC2;
    eSumdPhi += edPhi;

    //      limitation
    if(eSumdPhi < -antiWindupMax) eSumdPhi = -antiWindupMax;
    if(eSumdPhi > antiWindupMax)  eSumdPhi =  antiWindupMax;

    dPhi = (KP_DPHI * edPhi + T_PWM * eSumdPhi * KI_DPHI) * -100;     //*100 for percent

    //limitation of control variable
    if(dPhi < -upperLowerdPhiLimit)  dPhi = -upperLowerdPhiLimit;
    if(dPhi > upperLowerdPhiLimit) dPhi = upperLowerdPhiLimit;
    //------------------------------------------------------------------


    HRPWMupdatePhases(phi, dPhi);                        //set pwm


    sgen.calc(&sgen);

    DacbRegs.DACVALS.bit.DACVALS = sgen.out;       //max = 4095

//    DacbRegs.DACVALS.bit.DACVALS = uC2 * DACgain + DACoffset;       //max = 4095

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag ADC A
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag ADC B
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GPIO_WritePin(DEBUG_PIN1, 0);
}




void HRPWMupdatePhases(float32 Phi, float32 dPhi){
    float32 phaseShift1_3 = PWM_PERIOD_TICKS/2 + 2*PWM_PERIOD_TICKS/100.0 * Phi - 0.5;  //-0.5 cause PWM1 und PWM2 no HR -> round down 0.5
    float32 phaseShift1_3_main = floor(phaseShift1_3);
    float32 phaseShift1_3_rest = phaseShift1_3 - phaseShift1_3_main;
    Uint16 phaseShiftMEP1_3 = 256 * phaseShift1_3_rest;

    EPwm3Regs.CMPA.bit.CMPA = phaseShift1_3_main;
    EPwm3Regs.CMPB.bit.CMPB = phaseShift1_3_main;
    EPwm3Regs.CMPA.bit.CMPAHR = (phaseShiftMEP1_3 << 8);
    EPwm3Regs.CMPB.bit.CMPBHR = (phaseShiftMEP1_3 << 8);

    float32 phi1_4 = Phi + dPhi;

    float32 phaseShift1_4 = PWM_PERIOD_TICKS/2 + 2*PWM_PERIOD_TICKS/100.0 * phi1_4 - 0.5; //-0.5 cause PWM1 und PWM2 no HR -> round down 0.5
    float32 phaseShift1_4_main = floor(phaseShift1_4);
    float32 phaseShift1_4_rest = phaseShift1_4 - phaseShift1_4_main;
    Uint16 phaseShiftMEP1_4 = 256 * phaseShift1_4_rest;

    EPwm4Regs.CMPA.bit.CMPA = phaseShift1_4_main;
    EPwm4Regs.CMPB.bit.CMPB = phaseShift1_4_main;
    EPwm4Regs.CMPA.bit.CMPAHR = (phaseShiftMEP1_4 << 8);
    EPwm4Regs.CMPB.bit.CMPBHR = (phaseShiftMEP1_4 << 8);
}

void HRPWMupdateDB(int DB){
    EPwm1Regs.DBRED.bit.DBRED = DB;
    EPwm1Regs.DBFED.bit.DBFED = DB;

    EPwm2Regs.DBRED.bit.DBRED = DB;
    EPwm2Regs.DBFED.bit.DBFED = DB;

    EPwm3Regs.DBRED.bit.DBRED = DB;
    EPwm3Regs.DBFED.bit.DBFED = DB;

    EPwm4Regs.DBRED.bit.DBRED = DB;
    EPwm4Regs.DBFED.bit.DBFED = DB;
}



//
// HRPWM_Config - Configures all ePWM channels and sets up HRPWM
//                on ePWMxA channels &  ePWMxB channels
//
void HRPWM1_Config(period){
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm1Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm1Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm1Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm1Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm1Regs.CMPB.all |= 1;                // ?????????????????
    EPwm1Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm1Regs.TBCTR = 0;                    // set counter to 0

    //EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Select up-down count mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; //TB_ENABLE;         // to synchronize from EXTSYNCIN1
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM2
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


   // EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // PWM toggle high/low
    EPwm1Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
 //   EPwm1Regs.AQCTLA.bit.CBD = AQ_CLEAR;
    //EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;         //PWM_B generated throu DB module
    //EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
    EPwm1Regs.AQSFRC.bit.ACTSFA = 1;                    //clear
    EPwm1Regs.AQSFRC.bit.ACTSFB = 1;                    //clear
//    EPwm1Regs.AQSFRC.bit.OTSFA = 1;                     //force clear
//    EPwm1Regs.AQSFRC.bit.OTSFB = 1;                     //force clear


    //configure SOC for ADC A
    EPwm1Regs.ETSEL.bit.SOCBSEL =  0x6;             //100: Enable event time-base counter equal to CMPA when the timer
                                                    //is incrementing or CMPC when the timer is incrementing
//    EPwm1Regs.ETSEL.bit.INTSEL = 0x4;               //Enable event time-base counter equal to CMPA when the timer
//                                                    //is incrementing or CMPC when the timer is incrementing
//    EPwm1Regs.ETSEL.bit.INTEN = 0x1;                //1: Enable EPWMx_INT generation

    EPwm1Regs.ETSEL.bit.SOCBEN = 0x1;               //Enable EPWMxSOCA pulse.
//    EPwm1Regs.ETPS.bit.INTPSSEL = 0;                //Selects ETPS [INTCNT, and INTPRD] registers to determine
//                                                    //frequency of events (interrupt once every 0-3 events).

//    EPwm1Regs.ETPS.bit.INTPRD = 0x1;                //01: Generate an interrupt on the first event INTCNT = 01 (first event)


    EPwm1Regs.ETPS.bit.SOCBPRD = 0x2;               //01: Generate the EPWMxSOCA pulse on the second event




    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;


    EPwm1Regs.DBRED.bit.DBRED = deadBand;
    EPwm1Regs.DBFED.bit.DBFED = deadBand;
}

void DAC_A_Config(void){
    EALLOW;
    DacbRegs.DACCTL.bit.LOADMODE = 0;       //Load on next SYSCLK
    DacbRegs.DACCTL.bit.DACREFSEL = 1;      //ADC VREFHI/VREFLO are the reference voltages
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;     //DAC output is enabled
    EDIS;
}


void HRPWM2_Config(period){
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm2Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm2Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm2Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm2Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm2Regs.CMPB.all |= 1;                // ?????????????????
    EPwm2Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm2Regs.TBCTR = 0;                    // set counter to 0

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; //TB_ENABLE;         // sync signal from PWM1 enable
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM3
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm2Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
  //  EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR;
//    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;             // PWM toggle high/low
//    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
    EPwm2Regs.AQSFRC.bit.ACTSFA = 1;                    //clear
    EPwm2Regs.AQSFRC.bit.ACTSFB = 1;                    //clear
//    EPwm2Regs.AQSFRC.bit.OTSFA = 1;                     //force clear
//    EPwm2Regs.AQSFRC.bit.OTSFB = 1;                     //force clear

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm2Regs.DBRED.bit.DBRED = deadBand;
    EPwm2Regs.DBFED.bit.DBFED = deadBand;
}




void HRPWM3_Config(period){
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm3Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm3Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm3Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm3Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm3Regs.CMPB.all |= 1;                // ?????????????????
    EPwm3Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm3Regs.TBCTR = 0;                    // set counter to 0

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; //TB_ENABLE;         // sync signal from PWM2 enable
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM4 (not used)
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm3Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
 //   EPwm3Regs.AQCTLA.bit.CBD = AQ_CLEAR;
//    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;             // PWM toggle high/low
//    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

    EPwm3Regs.AQSFRC.bit.ACTSFA = 1;                    //clear
    EPwm3Regs.AQSFRC.bit.ACTSFB = 1;                    //clear
//    EPwm3Regs.AQSFRC.bit.OTSFA = 1;                     //force clear
//    EPwm3Regs.AQSFRC.bit.OTSFB = 1;                     //force clear

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm3Regs.DBRED.bit.DBRED = deadBand;
    EPwm3Regs.DBFED.bit.DBFED = deadBand;


    EALLOW;
    EPwm3Regs.HRCNFG.all = 0x0;
    EPwm3Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // Rising on A
    EPwm3Regs.HRCNFG.bit.EDGMODEB = HR_BEP;         // Falling on B
    EPwm3Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
    EPwm3Regs.HRCNFG.bit.CTLMODEB = HR_CMP;          // CMPBHR and TBPRDHR HR control
    EPwm3Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
    EPwm3Regs.HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
 //   EPwm3Regs.HRCNFG.bit.SWAPAB = 1;
    EPwm3Regs.HRCNFG.bit.AUTOCONV = 1;              // Enable autoconversion for HR period
    EPwm3Regs.HRPCTL.bit.TBPHSHRLOADE = 1;          // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm3Regs.HRPCTL.bit.HRPE = 1;                  // Turn on high-resolution period control.
    EDIS;
}

void HRPWM4_Config(period){
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm4Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm4Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm4Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm4Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm4Regs.CMPB.all |= 1;                // ?????????????????
    EPwm4Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm4Regs.TBCTR = 0;                    // set counter to 0

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE; //TB_ENABLE;         // to synchronize from EXTSYNCIN1
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM2
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


    EPwm4Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
  //  EPwm4Regs.AQCTLA.bit.CBD = AQ_CLEAR;
    //EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;         //PWM_B generated throu DB module
    //EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    EPwm4Regs.AQSFRC.bit.ACTSFA = 1;                    //clear
    EPwm4Regs.AQSFRC.bit.ACTSFB = 1;                    //clear
//    EPwm4Regs.AQSFRC.bit.OTSFA = 1;                     //force clear
//    EPwm4Regs.AQSFRC.bit.OTSFB = 1;                     //force clear
    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm4Regs.DBRED.bit.DBRED = deadBand;
    EPwm4Regs.DBFED.bit.DBFED = deadBand;

    EALLOW;
    EPwm4Regs.HRCNFG.all = 0x0;
    EPwm4Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // Rising on A
    EPwm4Regs.HRCNFG.bit.EDGMODEB = HR_BEP;         // Falling on B
    EPwm4Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
    EPwm4Regs.HRCNFG.bit.CTLMODEB = HR_CMP;          // CMPBHR and TBPRDHR HR control
    EPwm4Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
    EPwm4Regs.HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
 //   EPwm4Regs.HRCNFG.bit.SWAPAB = 1;
    EPwm4Regs.HRCNFG.bit.AUTOCONV = 1;              // Enable autoconversion for HR period
    EPwm4Regs.HRPCTL.bit.TBPHSHRLOADE = 1;          // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm4Regs.HRPCTL.bit.HRPE = 1;                  // Turn on high-resolution period control.
    EDIS;
}

void HRPWM5_Config(period){
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm5Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm5Regs.CMPA.bit.CMPA = EPwm1Regs.CMPA.bit.CMPA + adcStartDelayClk;   // start delay from PWM1 toggle

    EPwm5Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm5Regs.TBCTR = 0;                    // set counter to 0

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; //TB_ENABLE;         // to synchronize from EXTSYNCIN1
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM2
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm5Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;


   // EPwm5Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
}


//
// scia_fifo_init - Configure SCIA FIFO
//
void scia_fifo_init(void){


    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);


    EALLOW;
   CpuSysRegs.PCLKCR7.bit.SCI_A = 1; // enable clock


   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0000;    // 57600 baud @LSPCLK = 50MHz
   SciaRegs.SCILBAUD.all = 0x006c;    //(200 MHz SYSCLK).
   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back


   SciaRegs.SCIFFRX.bit.RXFFIL = 1;       //Receive FIFO interrupt level bits, after how many bits interrupt
   SciaRegs.SCIFFRX.bit.RXFFIENA = 1;     //Receive FIFO interrupt enable
   SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   //Receive FIFO interrupt clear
   //SciaRegs.SCIFFRX.bit.RXFFINT      //interrupt register, read only
   //SciaRegs.SCIFFRX.bit.RXFFST       //how many byte received, read only
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1; //Receive FIFO reset pointer to zero
   SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   //clear overflow flag if set
   //SciaRegs.SCIFFRX.bit.RXFFOVF         //overflow bit, read only


   SciaRegs.SCIFFTX.bit.TXFFIL = 2;
   SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
   SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
 //  SciaRegs.SCIFFTX.bit.TXFFINT    interrupt flag
 //  SciaRegs.SCIFFTX.bit.TXFFST    how many byte the FIFO contains
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
   SciaRegs.SCIFFTX.bit.SCIRST = 1;

   SciaRegs.SCIFFCT.all = 0x00;


   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
   SciaRegs.SCICTL1.bit.SWRESET = 1;
   EDIS;
}


//
// sciaTxFifoIsr - SCIA Transmit FIFO ISR
//
interrupt void sciaTxFifoIsr(void){
    Uint16 data;

    for(;(SciaRegs.SCIFFTX.bit.TXFFST < 0xF) & (!ringbuffer_empty(&TXBufferStruct));)
    {
        ringbuffer_get(&TXBufferStruct, &data);
        SciaRegs.SCITXBUF.bit.TXDT = (Uint16)data;
    }

    if(ringbuffer_empty(&TXBufferStruct))           //if sending buffer is empty disable interrupt for empty TX buffer
    {
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void){
    uint16_t data;

    while((SciaRegs.SCIFFRX.bit.RXFFST > 0) & (!ringbuffer_full(&RXBufferStruct)))
    {
        /* Read data from RX FIFO */
        data = (uint16_t)SciaRegs.SCIRXBUF.bit.SAR;
        /* Write data to ring buffer */
        ringbuffer_put(&RXBufferStruct, data);
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}


//
// error - Halt debugger when error occurs
//
void error (void){
    ESTOP0;         // Stop here and handle error
}

//
// End of file
//
