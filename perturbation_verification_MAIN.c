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

#define OPEN_LOOP_PERTURBATION_MODE

// Defines
#define SYSCLK    200e6
#define EPWMCLK   100e6
#define F_PWM     200e3
#define PWM_CH    9        // # of PWM channels

#define PWM_PERIOD_TICKS  (EPWMCLK/(F_PWM) -1)

#define DEBUG_PIN1 161
#define DEBUG_PIN2 163

#define adcStartDelayClk 0  //after how many PWM_CLK the ADC start


float32 T_PWM = 1/F_PWM;


ringbuffer_t TXBufferStruct;
ringbuffer_t RXBufferStruct;
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
Uint16 TXBuffer[TX_BUFFER_SIZE];
Uint16 RXBuffer[RX_BUFFER_SIZE];

#define RX_PACKAGE_SIZE 20
Uint16 RxPackage[RX_PACKAGE_SIZE];

Uint16 RxCommand[24];
Uint16 RxData[24];

//Buffer for transfer function measurement
#define stabilityBufferSize 4096       //max ~13300
Uint16 stabilityCounter = 0;
Uint16 stabilityStartMeasure = 0;
#define stabilityMaxFreq 100000

#pragma SET_DATA_SECTION("myData")
float32 stabilityBuffer1[stabilityBufferSize];
Uint16 stabilityBuffer2[stabilityBufferSize];
#pragma SET_DATA_SECTION()


int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO(0) function.


Uint16 deadBand = 10;
Uint16 status;
Uint16 newCommandRdy = 0;
Uint16 i = 0;
Uint16 rampUp = 0;
Uint16 rampDown = 0;

//Uout control loop values
float32 KP_PHI = 0.02;
float32 KI_PHI = 1000;
float32 ADC_UOUT_GAIN = 0.003661;
float32 ADC_UOUT_OFFSET = 0.003590;


float32 eSumPhi = 0;
float32 uOutTarget = 4.0;
float32 ePhi = 0;
float32 phi = 0;
float32 uOut = 0;
float32 upperPhiLimit = 0;


//C2 control loop values
float32 KP_DPHI = 0.01;
float32 KI_DPHI = 100;
float32 ADC_UC2_GAIN = 0.003659;
float32 ADC_UC2_OFFSET = -0.017504;

float32 eSumdPhi = 0;
float32 uC2Target = 4.0;
float32 edPhi = 0;
float32 dPhi = 0;
float32 uC2 = 0;
float32 upperLowerdPhiLimit = 0;

float32 antiWindupMax = 5000;
float32 DACgain = 1024;
float32 DACoffset = 2048;

//parameter for stability
float32 sineOffset = 0;
float32 sineGain   = 0;
float32 sineFreq   = 1000;
Uint16  sinusUpdate = 0;


float32 phiTemp = 0;
float32 uTemp = 0;
float32 dPhiTemp = 0;
float32 duty = 0;

Uint16 delayTemp = 0;

// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
   &EPwm6Regs, &EPwm7Regs, &EPwm8Regs};


// Function Prototypes
void HRPWM1_Config(int);
void HRPWM8_Config(int);
void DAC_A_Config(void);
void reverseString(uint16_t *str, uint16_t len);
uint16_t intToStr(uint16_t x, uint16_t str[], uint16_t d);
void floatToArray(float32 n, Uint16 *res, Uint16 afterpoint);
void HRPWMupdateDutyC(float32 duty);
void HRPWMupdateDB(int deadBand);
void setSine(float32 offset, float32 gain, float32 freq, float32 maxFreq);
void setSinusFreq(float32 freq, float32 maxFreq);
void ConfigureADC(void);
interrupt void adca0_isr(void);
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);

void error(void);

SGENTI_1 sgen=SGENTI_1_DEFAULTS;

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

//    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(7, 1);
//
//    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(6, 1);
//
//    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(0, 1);
//
//    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(1, 1);
//
//    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(2, 1);
//
//    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_WritePin(3, 1);

    InitEPwm1Gpio();
    InitEPwm8Gpio();

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
    EALLOW;                                             // This is needed to write to EALLOW protected registers
    PieVectTable.ADCD1_INT = &adca0_isr;                //function for ADC-B interrupt 1
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;          //ISR for RX interrupt
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;          //ISR for TX interrupt
    EDIS;                                               // This is needed to disable write to EALLOW protected registers


    // configure Serial
    scia_fifo_init();

    // Configure the ADC and power it up
    ConfigureADC();

    // configuarion of the DAC
    DAC_A_Config();

    setSine(0, 0, 1000, stabilityMaxFreq);


    // enable PIE interrupts
//  PieCtrlRegs.PIEIER1.bit.INTx1  = 1;                 //enable interrupt for ADC A

    //enable ADC interrupt after voltage startup
    //if from beginning on => controller will be go to max phase shift
    //max phaseshift at startup will result in high current
    //can partial compensated with dPhi
    PieCtrlRegs.PIEIER1.bit.INTx6  = 1;                 // Enable CPU INT group 1 ADC-D
    //PieCtrlRegs.PIEIER1.bit.INTx1  = 1;               // ADC B is used for interrupt
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;                  // PIE Group 9, INT1, TX interrupt
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;                  // PIE Group 9, INT2, RX interrupt

    IER |= M_INT9;                                      // Enable CPU INT group 9 (RX/TX)
    IER |= M_INT1;                                      // Enable CPU INT group 1 (ADC)

    // Enable global Interrupts and higher priority real-time debug events
    EINT;                                               // Enable Global interrupt INTM
    ERTM;                                               // Enable Global realtime interrupt DBGM


   status = SFO_INCOMPLETE;

    // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
    // HRMSTEP must be populated with a scale factor value prior to enabling
    // high resolution period control.
    //
    while(status == SFO_INCOMPLETE){                    // Call until complete
        status = SFO();
        if (status == SFO_ERROR){
            error();                                    // SFO function returns 2 if an error occurs & # of MEP
        }                                               // steps/coarse step exceeds maximum of 255.
    }


    EALLOW;
    InputXbarRegs.INPUT5SELECT = 0xA;                   //Input5 is normally connected to GPIO0 which is PWM1 output-> every PWM cycle reset
                                                        //use not as input
                                                        //GPIO10 -> INPUT5 -> EXTSYNCIN1 -> Sync singal for PWM modules (S. 1737)

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Disable TBCLK within the EPWM
    EDIS;
    //PWM frequence EPWMCLK = 100MHz (maximum)
    HRPWM1_Config(PWM_PERIOD_TICKS);                    // ePWMx target
    HRPWM8_Config(PWM_PERIOD_TICKS);                    // ePWMx target
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Enable TBCLK within the EPWM
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;                    // Synchronize high resolution phase to start HR period
    EDIS;

    //init RX/TX buffer
    TXBufferStruct.buffer = TXBuffer;
    RXBufferStruct.buffer = RXBuffer;
    TXBufferStruct.size = TX_BUFFER_SIZE;
    RXBufferStruct.size = RX_BUFFER_SIZE;
    ringbuffer_reset(&TXBufferStruct);
    ringbuffer_reset(&RXBufferStruct);


    for(;;){
        GPIO_WritePin(DEBUG_PIN2, 1);
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;      //clear INT1 flag ADC A




        if(!ringbuffer_empty(&RXBufferStruct)){                     //check RX buffer for new character
            ringbuffer_get(&RXBufferStruct, &RxPackage[i]);         //copy character

            if(RxPackage[i] == 0x000D){                             //if carrage return is received => command complete
                Uint16 backspacePos = 0;
                Uint16 j = 0;
                for(j = 0; j < sizeof(RxData); j++){
                    RxData[j] = 0;                                  //clear RxData buffer
                }
                for(j = 0; j < sizeof(RxCommand); j++){
                    RxCommand[j] = 0;                               //clear RxCommand buffer
                }

                for(j = 0; j < i; j++){
                    if(RxPackage[j] == 0x0020){                     //find backspace in RxPackage
                        backspacePos = j;
                    }
                    if(backspacePos > 0){
                        RxData[j-backspacePos] = RxPackage[j+1];    //everything after backspace => RxData
                    }
                    else{
                        RxCommand[j] = RxPackage[j];                //everything before backspace => RxCommand
                    }
                }
                newCommandRdy = 1;
            }
            i++;
        }

        if(newCommandRdy > 0){  //received command ready for execution

            for(i = 0; i < RX_PACKAGE_SIZE; i++){                   //clear buffer
                RxPackage[i] = 0;
            }
            i = 0;

            //get kp phi
            if(memcmp(RxCommand, "getkpphi", strlen("getkpphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(KP_PHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }

            //get ki phi
            if(memcmp(RxCommand, "getkiphi", strlen("getkiphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(KI_PHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }

            //get kp dphi
            if(memcmp(RxCommand, "getkpdphi", strlen("getkpdphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(KP_DPHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }

            //get ki dphi
            if(memcmp(RxCommand, "getkidphi", strlen("getkidphi")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(KI_DPHI, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }
            
            //get Uout target
            if(memcmp(RxCommand, "getuouttarget", strlen("getuouttarget")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(uOutTarget, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }
            
            //get UC2 target
            if(memcmp(RxCommand, "getuc2target", strlen("getuc2target")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                floatToArray(uC2Target, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }
            
            //get dead band
            if(memcmp(RxCommand, "getdeadband", strlen("getdeadband")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                intToStr(deadBand * 5, sendData, 1);                // *5 to get ns
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }
            
            //get all parameter at once
            if(memcmp(RxCommand, "getdata", strlen("getdata")) == 0){
                Uint16 sendData[10];
                Uint16  j = 0;

                //load phi into ringbuffer
                floatToArray(phi, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load dPhi into ringbuffer
                floatToArray(dPhi, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load uOut into ringbuffer
                floatToArray(uOut, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load uOutTarget into ringbuffer
                floatToArray(uOutTarget, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load uC2 into ringbuffer
                floatToArray(uC2, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load uC2Target into ringbuffer
                floatToArray(uC2Target, sendData, 4);
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, " ", 1);

                //load deadBand into ringbuffer
                intToStr(deadBand * 5, sendData, 1);                //*5 to get ns
                j = 0;
                while(sendData[j] != '\0'){
                    ringbuffer_put(&TXBufferStruct, sendData[j]);
                    j++;
                }
                ringbuffer_put_array(&TXBufferStruct, "\n", 1);
//                ringbuffer_put_array(&TXBufferStruct, "\r", 1);

                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;                  //start sending
            }

            //get whole stability buffer data
            if(memcmp(RxCommand, "getMemory", strlen("getMemory")) == 0){
                Uint16  j = 0;
                char temp[2];
                for(j=0; j<stabilityBufferSize; j++){
                    memcpy(temp, &stabilityBuffer1[j], 2);
                    //float to 4 bytes, little endian
                    ringbuffer_put(&TXBufferStruct, temp[0]);
                    ringbuffer_put(&TXBufferStruct, temp[0] >> 8);
                    ringbuffer_put(&TXBufferStruct, temp[1]);
                    ringbuffer_put(&TXBufferStruct, temp[1] >> 8);

                    //int to 2 bytes, little endian
                    ringbuffer_put(&TXBufferStruct, stabilityBuffer2[j]);
                    ringbuffer_put(&TXBufferStruct, (stabilityBuffer2[j] >> 8));

                    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;              //start sending

                    while(!ringbuffer_empty(&TXBufferStruct)){      //wait until buffer is empty before sending next package
                        ;
                    }
                }
            }

            //set the new sine offset, waveform change if "setperturbation" will executed (0% to 10%)
            if(memcmp(RxCommand, "setsineoffset", strlen("setsineoffset")) == 0){
                sineOffset = atof(RxData);
#ifdef OPEN_LOOP_PERTURBATION_MODE
                setSine(sineOffset/100.0, sineGain/100.0, sineFreq, stabilityMaxFreq);
#endif
#ifdef CLOSE_LOOP_PERTURBATION_MODE
                setSine(sineOffset, sineGain, sineFreq, stabilityMaxFreq);
#endif

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            
            //set the new sine gain, waveform change if "setperturbation" will executed (0% to 10%)
            if(memcmp(RxCommand, "setsinegain", strlen("setsinegain")) == 0){
                sineGain = atof(RxData);
#ifdef OPEN_LOOP_PERTURBATION_MODE
                setSine(sineOffset/100.0, sineGain/100.0, sineFreq, stabilityMaxFreq);
#endif
#ifdef CLOSE_LOOP_PERTURBATION_MODE
                setSine(sineOffset, sineGain, sineFreq, stabilityMaxFreq);
#endif

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }            
            
            //set the new sine frequency, waveform change if "setperturbation" will executed (0 to stabilityMaxFreq)
            if(memcmp(RxCommand, "setsinefreq", strlen("setsinefreq")) == 0){
                sineFreq = atof(RxData);
#ifdef OPEN_LOOP_PERTURBATION_MODE
                setSine(sineOffset/10.0, sineGain/10.0, sineFreq, stabilityMaxFreq);
#endif
#ifdef CLOSE_LOOP_PERTURBATION_MODE
                setSine(sineOffset, sineGain, sineFreq, stabilityMaxFreq);
#endif

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }   

            
            //start the mesurement, stabilityBufferSize values store in the buffer
            if(memcmp(RxCommand, "startpertmeasure", strlen("startpertmeasure")) == 0){
                stabilityCounter = 0;               //reset counter
                stabilityStartMeasure = 1;
                
                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set kp phi
            if(memcmp(RxCommand, "setkpphi", strlen("setkpphi")) == 0){
                KP_PHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set ki phi
            if(memcmp(RxCommand, "setkiphi", strlen("setkiphi")) == 0){
                KI_PHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set kp dphi
            if(memcmp(RxCommand, "setkpdphi", strlen("setkpdphi")) == 0){
                KP_DPHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set ki dphi
            if(memcmp(RxCommand, "setkidphi", strlen("setkidphi")) == 0){
                KI_DPHI = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set deadband
            if(memcmp(RxCommand, "setdeadband", strlen("setdeadband")) == 0){
                float32 deadBand_ns = atof(RxData);
                deadBand = round(deadBand_ns / 5);
                HRPWMupdateDB(deadBand);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set Uout target
            if(memcmp(RxCommand, "setuouttarget", strlen("setuouttarget")) == 0){
                uOutTarget = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }


            //set UC2 target
            if(memcmp(RxCommand, "setuc2target", strlen("setuc2target")) == 0){
                uC2Target = atof(RxData);

                ringbuffer_put_array(&TXBufferStruct, "OK\n", 3);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending
            }
            newCommandRdy = 0;
        }
        GPIO_WritePin(DEBUG_PIN2, 0);
    } // end infinite for loop
}

// Converts a floating point number to string.
void floatToArray(float32 n, Uint16 *res, Uint16 afterpoint){
    int16 ipart, i;
    float32 fpart,temp;

    if(n<0)
    {
        res[0] = '-';
        ipart = (int16)n;
        ipart =-1 * ipart;
        temp = (n*(-2));
        fpart = temp + n;
        fpart = fpart - (float32)ipart;
        i = intToStr(ipart, res+1, 1)+1;
    }
    else
    {
        // Extract integer part
        ipart = (int16)n;

        // Extract floating part
        fpart = n - (float32)ipart;

        // convert integer part to string
        i = intToStr(ipart, res, 1);

    }

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

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

void setSine(float32 offset, float32 gain, float32 freq, float32 maxFreq){
    sgen.offset = offset * 0x7FFF;              // Range(Q15) = 0x8000 -> 0x7FFF (-1 to 1);
    sgen.gain = gain * 0x7FFF;                  // Range(Q15) = 0x0000 -> 0x7FFF (0 to 1)
    sgen.freq = (freq/maxFreq) * 32768;         // freq = (Required Freq/Max Freq)*2^15
    sgen.step_max=(maxFreq * 65536)/F_PWM;      // Max Freq= (step_max * sampling freq)/65536
                                                // sampling freq = pwm freq = F_PWM
}

void setSinusFreq(float32 freq, float32 maxFreq){
    sgen.freq = (freq/maxFreq) * 32768;         // freq = (Required Freq/Max Freq)*2^15
    sgen.step_max=(maxFreq * 65536)/F_PWM;      // Max Freq= (step_max * sampling freq)/65536
}

void ConfigureADC(void){
    //Configure ADC-A for input A0
    //Configure start of conversion block 0 (SOC0) to pin A0 (Datasheet S.92, Figure 2-2)
    EALLOW;
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 1;          // ADC-A SOC0 will convert pin A0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;         // sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 6;        // trigger ADC-D by ePWM1, ADCSOCB (CMPD)

    AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 0;      // post-processing block 1 associated with SOC0, EOC0

    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // end of ADC-D will set INT1 flag
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
    AdcdRegs.ADCINTSEL1N2.bit.INT1CONT = 1;

    AdcdRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 0;       // interrrupt pulse occurs at the end of the acquisition window
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC-A




    DELAY_US(1000);                             //delay for 1ms to allow ADC time to power up
    EDIS;
}

//
// adca1_isr - ADCA1 Interrupt Service Routine
//
interrupt void adca0_isr(void){
    GPIO_WritePin(DEBUG_PIN1, 1);

    //as soon as serial command set "stabilityStartMeasure=1" the measurement begins
    if((stabilityStartMeasure ==  1) & (stabilityCounter < stabilityBufferSize)){
        stabilityBuffer1[stabilityCounter] = duty/100;
        stabilityBuffer2[stabilityCounter] = AdcdResultRegs.ADCRESULT0;     //ADC Uout
//            stabilityBuffer2[stabilityCounter] = AdcaResultRegs.ADCRESULT0;     //ADC UC2
        stabilityCounter++;
    }

        sgen.calc(&sgen);                               //calculate sine
        duty = sgen.out/327.67;                      //sgen.out max = 32767, scale output: 0 to 10
        //if(phiTemp < 0) phiTemp = 0;
        HRPWMupdateDutyC(duty);           //set pwm


        DacbRegs.DACVALS.bit.DACVALS = duty * 13.65;  // DAC_max_out=3V (DAC_max_digital = 4095)






    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //clear INT1 flag ADC A
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;      //clear INT1 flag ADC B
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //clear INT1 flag ADC B
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GPIO_WritePin(DEBUG_PIN1, 0);
}


void HRPWMupdateDutyC(float32 duty){
    float32 phaseShift1_3 = PWM_PERIOD_TICKS/100.0 * duty;
    float32 phaseShift1_3_main = floor(phaseShift1_3);
    float32 phaseShift1_3_rest = phaseShift1_3 - phaseShift1_3_main;
    Uint16 phaseShiftMEP1_3 = 256 * phaseShift1_3_rest;

    EPwm8Regs.CMPA.bit.CMPA = phaseShift1_3_main;
    EPwm8Regs.CMPB.bit.CMPB = phaseShift1_3_main;
    EPwm8Regs.CMPA.bit.CMPAHR = (phaseShiftMEP1_3 << 8);
    EPwm8Regs.CMPB.bit.CMPBHR = (phaseShiftMEP1_3 << 8);
}

void HRPWMupdateDB(int db){
    EPwm1Regs.DBRED.bit.DBRED = db;
    EPwm1Regs.DBFED.bit.DBFED = db;

    EPwm8Regs.DBRED.bit.DBRED = db;
    EPwm8Regs.DBFED.bit.DBFED = db;
}

void HRPWM1_Config(period){
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;                      // set shadow load
    EPwm1Regs.TBPRD = period;                                   // set PWM frequency
    EPwm1Regs.CMPA.bit.CMPA = period / 2;                       // set duty 50% initially
    EPwm1Regs.CMPA.bit.CMPAHR = (1 << 8);                       // initialize HRPWM extension
    EPwm1Regs.CMPB.bit.CMPB = period / 2 + adcStartDelayClk;    // ADC interrupt, set duty 50% initially + delay before adc starts
    EPwm1Regs.CMPB.all |= 1;                                    // ?????????????????
    EPwm1Regs.TBPHS.all = 0;                                    // value which is loaded into TBCTR when sinc puls occur
    EPwm1Regs.TBCTR = 0;                                        // set counter to 0

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;                  // up-count mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;                     // sync signal disable
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;             // sync feed trough disable
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                    // High Speed Time Base Clock Pre-Scale
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;                       // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;                         // free running

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;               // LOAD CMPA on CTR = 0
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;               // LOAD CMPB on CTR = 0
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                 // CMPA shadow load
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                 // CMPB shadow load

    EPwm1Regs.AQCTLA.bit.CAU = AQ_TOGGLE;                       // PWM toggle high/low
    //EPwm1Regs.AQCTLA.bit.CBD = AQ_CLEAR;                      // not used
    //EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;                      // not used, PWM_B generated thru DB module
    //EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;                        // not used, PWM_B generated thru DB module

    EPwm1Regs.AQSFRC.bit.ACTSFA = 1;                            // clear (low) when at software force
    EPwm1Regs.AQSFRC.bit.ACTSFB = 1;                            // clear (low) when at software force
    //EPwm1Regs.AQSFRC.bit.OTSFA = 1;                           // software force
    //EPwm1Regs.AQSFRC.bit.OTSFB = 1;                           // software force

    //configure SOC for ADC A
    EPwm1Regs.ETSEL.bit.SOCBSEL = 0x6;                          // enable even if PWM counter upcounting and equal CMPB
    EPwm1Regs.ETSEL.bit.SOCBEN = 0x1;                           // enable the ADC Start of Conversion B (EPWMxSOCB) Pulse
    EPwm1Regs.ETPS.bit.SOCBPRD = 0x1;                           // generate the EPWMxSOCA pulse on the first event

    // activate Deadband
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;                      // EPWMxA In (from the action-qualifier) is the source for both
                                                                // falling-edge and rising-edge delay.
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;                   // active high complementary (AHC)
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;              // DBM is fully enabled (i.e. both RED and FED active)
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;                          // Half cycle clocking enabled. The dead-band counters are clocked at TBCLK*2.

    EPwm1Regs.DBRED.bit.DBRED = deadBand;                       // init deadBand
    EPwm1Regs.DBFED.bit.DBFED = deadBand;                       // init deadBand
}



void HRPWM8_Config(period){

    EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;                      // set Shadow load
    EPwm8Regs.TBPRD = period;                                   // set PWM frequency
    EPwm8Regs.CMPA.bit.CMPA = period / 2;                       // set duty 50% initially
    EPwm8Regs.CMPA.bit.CMPAHR = (1 << 8);                       // initialize HRPWM extension
    EPwm8Regs.CMPB.bit.CMPB = period / 2;                       // set duty 50% initially
    EPwm8Regs.CMPB.all |= 1;                                    // ?????????????????
    EPwm8Regs.TBPHS.all = 0;                                    // value which is loaded into TBCTR when sinc puls occur
    EPwm8Regs.TBCTR = 0;                                        // set counter to 0

    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;                  // up-count mode
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;                     // sync signal disable
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                  // sync feed trough disable
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                    // High Speed Time Base Clock Pre-Scale
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;                       // TBCLK = SYSCLKOUT
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 11;                         // free running

    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;               // LOAD CMPA on CTR = 0
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;               // LOAD CMPB on CTR = 0
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                 // CMPA shadow load
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                 // CMPB shadow load

    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;                       // PWM toggle high/low
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
    //EPwm8Regs.AQCTLA.bit.CBD = AQ_CLEAR;                      // not used
    //EPwm8Regs.AQCTLB.bit.CAU = AQ_CLEAR;                      // not used, PWM_B generated thru DB module
    //EPwm8Regs.AQCTLB.bit.CBD = AQ_SET;                        // not used, PWM_B generated thru DB module

    EPwm8Regs.AQSFRC.bit.ACTSFA = 1;                            // clear (low) when at software force
    EPwm8Regs.AQSFRC.bit.ACTSFB = 1;                            // clear (low) when at software force
    //EPwm8Regs.AQSFRC.bit.OTSFA = 1;                           // software force
    //EPwm8Regs.AQSFRC.bit.OTSFB = 1;                           // software force

    // activate Deadband
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;                      // EPWMxA In (from the action-qualifier) is the source for both
                                                                // falling-edge and rising-edge delay.
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;                   // active high complementary (AHC)
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;              // DBM is fully enabled (i.e. both RED and FED active)
    EPwm8Regs.DBCTL.bit.HALFCYCLE = 1;                          // Half cycle clocking enabled. The dead-band counters are clocked at TBCLK*2.

    EPwm8Regs.DBRED.bit.DBRED = deadBand;                       // init deadBand
    EPwm8Regs.DBFED.bit.DBFED = deadBand;                       // init deadBand


    EALLOW;
    EPwm8Regs.HRCNFG.all = 0x0;
    EPwm8Regs.HRCNFG.bit.EDGMODE = HR_BEP;                      // MEP control of both edges (TBPHSHR or TBPRDHR)
    EPwm8Regs.HRCNFG.bit.EDGMODEB = HR_BEP;                     // MEP control of both edges (TBPHSHR or TBPRDHR)
    EPwm8Regs.HRCNFG.bit.CTLMODE = HR_CMP;                      // CMPAHR and TBPRDHR HR control
    EPwm8Regs.HRCNFG.bit.CTLMODEB = HR_CMP;                     // CMPBHR and TBPRDHR HR control
    EPwm8Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD;             // load on CTR = 0 and CTR = TBPRD
    EPwm8Regs.HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD;            // load on CTR = 0 and CTR = TBPRD
    //EPwm8Regs.HRCNFG.bit.SWAPAB = 1;
    EPwm8Regs.HRCNFG.bit.AUTOCONV = 1;                          // Enable autoconversion for HR period
    EPwm8Regs.HRPCTL.bit.TBPHSHRLOADE = 1;                      // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm8Regs.HRPCTL.bit.HRPE = 1;                              // Turn on high-resolution period control.
    EDIS;
}



void DAC_A_Config(void){
    EALLOW;
    DacbRegs.DACCTL.bit.LOADMODE = 0;                           //Load on next SYSCLK
    DacbRegs.DACCTL.bit.DACREFSEL = 1;                          //ADC VREFHI/VREFLO are the reference voltages
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;                         //DAC output enable
    EDIS;
}


// scia_fifo_init - Configure SCIA FIFO
void scia_fifo_init(void){
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);


    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;                           // enable clock
    SciaRegs.SCICCR.all = 0x0007;                               // 1 stop bit,  No loopback
                                                                // No parity,8 char bits,
                                                                // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;                              // enable TX, RX, internal SCICLK,
                                                                // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD.all = 0x0000;                             // 115200 baud @LSPCLK = 50MHz
    SciaRegs.SCILBAUD.all = 0x00A2;                             // at 200 MHz SYSCLK   (0x0035 = 115200Baud, 0x00A2 = 38400Baud)
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;                          // Enable loop back

    // receive buffer
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;                            //Receive FIFO interrupt level bits, after how many bits interrupt
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;                          //Receive FIFO interrupt enable
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;                        //Receive FIFO interrupt clear
    //SciaRegs.SCIFFRX.bit.RXFFINT                              //interrupt register, read only
    //SciaRegs.SCIFFRX.bit.RXFFST                               //how many byte received, read only
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;                       //Receive FIFO reset pointer to zero
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;                        //clear overflow flag if set
    //SciaRegs.SCIFFRX.bit.RXFFOVF                              //overflow bit, read only

    // transmi buffer
    SciaRegs.SCIFFTX.bit.TXFFIL = 2;
    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    //  SciaRegs.SCIFFTX.bit.TXFFINT                            // interrupt flag
    //  SciaRegs.SCIFFTX.bit.TXFFST                             // how many byte the FIFO contains
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
    SciaRegs.SCIFFTX.bit.SCIRST = 1;

    SciaRegs.SCIFFCT.all = 0x00;
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    SciaRegs.SCICTL1.bit.SWRESET = 1;
   EDIS;
}

// sciaTxFifoIsr - SCIA Transmit FIFO ISR
interrupt void sciaTxFifoIsr(void){
    Uint16 data;

    for(;(SciaRegs.SCIFFTX.bit.TXFFST < 0xF) & (!ringbuffer_empty(&TXBufferStruct));)
    {
        ringbuffer_get(&TXBufferStruct, &data);
        SciaRegs.SCITXBUF.bit.TXDT = (Uint16)data;
    }

    if(ringbuffer_empty(&TXBufferStruct))                       //if sending buffer is empty disable interrupt for empty TX buffer
    {
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;                          // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;                              // Issue PIE ACK
}

// sciaRxFifoIsr - SCIA Receive FIFO ISR
interrupt void sciaRxFifoIsr(void){
    uint16_t data;

    while((SciaRegs.SCIFFRX.bit.RXFFST > 0) & (!ringbuffer_full(&RXBufferStruct)))
    {
        /* Read data from RX FIFO */
        data = (uint16_t)SciaRegs.SCIRXBUF.bit.SAR;
        /* Write data to ring buffer */
        ringbuffer_put(&RXBufferStruct, data);
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;                          // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;                          // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;                              // Issue PIE ack
}


// error - Halt debugger when error occurs
void error (void){
    ESTOP0;         // Stop here and handle error
}



