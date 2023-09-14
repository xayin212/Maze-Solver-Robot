
#include "engr2350_msp432.h"

void GPIOInit();
void TimerInit();
void I2C_Init(eUSCI_I2C_MasterConfig econfig);
void Encoder_ISR();
void T1_100ms_ISR();
uint16_t readRanger();
void getInput();
void I2C_writeData(uint32_t moduleInstance
                  ,uint8_t PeriphAddress
                  ,uint8_t StartReg
                  ,uint8_t *data
                  ,uint8_t len);
void I2C_readData(uint32_t moduleInstance
                 ,uint8_t PeriphAddress
                 ,uint8_t StartReg
                 ,uint8_t *data
                 ,uint8_t len);


Timer_A_UpModeConfig TA0cfg;
Timer_A_UpModeConfig TA1cfg;
Timer_A_ContinuousModeConfig TA3cfg;
Timer_A_CompareModeConfig TA0_ccr3;
Timer_A_CompareModeConfig TA0_ccr4;
Timer_A_CaptureModeConfig TA3_ccr0;
Timer_A_CaptureModeConfig TA3_ccr1;


// Encoder total events
uint32_t enc_total_L,enc_total_R;
// Speed measurement variables
int32_t Tach_L_count,Tach_L,Tach_L_sum,Tach_L_sum_count,Tach_L_avg; // Left wheel
int32_t Tach_R_count,Tach_R,Tach_R_sum,Tach_R_sum_count,Tach_R_avg; // Right wheel

uint8_t run_control = 0; // Flag to denote that 100ms has passed and control should be run.

uint8_t drive = 0; // drive if 1, else turn if 0
uint16_t distanceL;
uint16_t distanceR;

float distvol;      // tracking speed and voltage potentiometer values
float speedvol;

uint16_t speed;

uint8_t BMP0 = 0;
uint8_t BMP1 = 0;
uint8_t BMP2 = 0;
uint8_t BMP3 = 0;
uint8_t BMP4 = 0;
uint8_t BMP5 = 0;

uint8_t firstloop = 0;

uint8_t rightflag = 0;
uint8_t leftflag = 0;

uint8_t leftflagevt1 = 0;
uint8_t leftflagevt2 = 0;
uint8_t leftflagevt3 = 0;

uint8_t rightflagevt0 = 0;
uint8_t rightflagevt1 = 0;
uint8_t rightflagevt2 = 0;

uint16_t desdist = 0; // desired distance
uint16_t desradius = 0; // desired radius
int16_t desspeed;
int16_t desspeedleft;
int16_t desspeedright;
uint16_t distance = 0;

uint16_t desdistance;
uint16_t distcap;

int16_t diffspeed; // differential speed
float error_left;
float error_right;

int16_t pwmset_left;
int16_t pwmset_right;

eUSCI_I2C_MasterConfig econfig;

uint8_t compassarray[2];
uint16_t compassoutput;

uint8_t rangerarray[2];
uint16_t rangeroutput;

uint16_t rangerval;

uint8_t inputarray[] = {0x51};

float ki = 0.3;


int main(void)
{
    SysInit();
    GPIOInit();
    I2C_Init(econfig);
    TimerInit();

    __delay_cycles(24e6);

    desdistance = 100;
    distcap = 250;
    desspeed = 150;

    Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);


    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    while(1){
        if (run_control) {

            getInput();
            run_control = 0;

            if (!BMP0 || !BMP1 || !BMP2 || !BMP3 || !BMP4 || !BMP5) {

                if (!rightflag) {
                    enc_total_L = 0;
                    enc_total_R = 0;

                }
                rightflag = 1;
                GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);

            }

            //distance control
            if (!firstloop) {
            rangerval = 10 * readRanger();
            } else {
                rangerval = 10 * readRanger();
                firstloop = 1;
                rangerval = 80;
            }


            if (rangerval >= distcap) {
                if (!rightflag) {
                if (!leftflag) {
                    enc_total_L = 0;
                    enc_total_R = 0;

                }
                leftflag = 1;
                GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
                }
            } else {

                diffspeed = (-0.09 * (desdistance - rangerval));
                GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
            }

            uint16_t distance = (((enc_total_L+enc_total_R)/2) * (0.0027777) * 2 * 35 * 3.14159265);

            if (rightflag) {

                if (!rightflagevt0) {

                    rightflagevt0 = 1;
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);
                    enc_total_L = 0;
                    enc_total_R = 0;

                } else if (!rightflagevt1) {

                    if (distance >= 90) {

                        enc_total_L = 0;
                        enc_total_R = 0;
                        GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);

                        rightflagevt1 = 1;

                    } else {

                        desspeed = 150;
                        diffspeed = 0;
                    }
                } else if (!rightflagevt2) {

                    if (distance >= 111) {

                        enc_total_L = 0;
                        enc_total_R = 0;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);

                        rightflagevt2 = 1;
                    } else {

                        desspeed = 150;
                        diffspeed = 0;
                    }

                } else if (rightflagevt2) {

                    leftflag = 0;
                    rightflag = 0;
                    leftflagevt1 = 0;
                    leftflagevt2 = 0;
                    leftflagevt3 = 0;
                    rightflagevt0 = 0;
                    rightflagevt1 = 0;
                    rightflagevt2 = 0;
                }


            } else if (leftflag) {

                if (!leftflagevt1) {

                    if (rangerval < 200) {

                        leftflag = 0;
                        rightflag = 0;
                        leftflagevt1 = 0;
                        leftflagevt2 = 0;
                        leftflagevt3 = 0;
                        rightflagevt0 = 0;
                        rightflagevt1 = 0;
                        rightflagevt2 = 0;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);

                    } else if (distance >= 100) {
                        leftflagevt1 = 1;
                        enc_total_L = 0;
                        enc_total_R = 0;

                        GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);

                        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);

                    } else {
                        desspeed = 150;
                        diffspeed = 0;
                    }

                } else if (!leftflagevt2) {

                    if (distance >= 111) {

                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);
                        enc_total_L = 0;
                        enc_total_R = 0;

                        leftflagevt2 = 1;

                    } else {

                        desspeed = 150;
                        diffspeed = 0;
                    }


                } else if (!leftflagevt3) {
                    if ((rangerval < 200) || (distance >= 200)) {
                        leftflagevt3 = 1;

                    } else {

                        desspeed = 150;
                        diffspeed = 0;
                    }

                } else if (leftflagevt3) {

                    leftflag = 0;
                    rightflag = 0;
                    leftflagevt1 = 0;
                    leftflagevt2 = 0;
                    leftflagevt3 = 0;
                    rightflagevt0 = 0;
                    rightflagevt1 = 0;
                    rightflagevt2 = 0;
                }


            }

            desspeedleft = desspeed - diffspeed;

            if ((desspeedleft <= 80) && (desspeedleft >= -80)) {
                desspeedleft = 0;

                }
                    if ((pwmset_left > 80) && (pwmset_left < -80)) {
                    error_left += desspeedleft-(8*(1500000/Tach_L_avg));
                    }
                    pwmset_left = desspeedleft + (ki*error_left);


            if(pwmset_left >= 720) pwmset_left = 720;
            if(pwmset_left <= 80) pwmset_left = 0;


            desspeedright = desspeed + diffspeed;

            if ((desspeedright <= 80) && (desspeedright >= -80)) {
                desspeedright = 0;

                }
                    if ((pwmset_right > 80) && (pwmset_right < -80)) {
                    error_right += desspeedright-(8*(1500000/Tach_R_avg));
                    }
                    pwmset_right = desspeedright + (ki*error_right);


            if(pwmset_right >= 720) pwmset_right = 720;
            if(pwmset_right <= 80) pwmset_right = 0;
            Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, pwmset_left);
            Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, pwmset_right);

        }

    }


}

void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2); //led light up

        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION); //init I2C Peripherals
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7); //bmp0-5 respectively

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motors are OFF

}

void TimerInit(){
    // Configure PWM timer for 30 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 800;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA1cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA1cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA1cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA1cfg.timerPeriod = 37500;
    Timer_A_configureUpMode(TIMER_A1_BASE,&TA1cfg);
    Timer_A_registerInterrupt(TIMER_A1_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T1_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}


void I2C_Init(eUSCI_I2C_MasterConfig econfig){
    econfig.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    econfig.i2cClk = 24000000;
    econfig.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    econfig.byteCounterThreshold = 0;
    econfig.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    I2C_initMaster(EUSCI_B3_BASE , &econfig);
    I2C_enableModule(EUSCI_B3_BASE);

}

uint16_t readRanger(){
    I2C_readData(EUSCI_B3_BASE, 0x70, 2, rangerarray, 2);
    rangeroutput = (rangerarray[0] << 8) + rangerarray[1];
    I2C_writeData(EUSCI_B3_BASE, 0x70, 0, inputarray, 1);
    return rangeroutput;
}

void getInput() {

    BMP0 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0);
    BMP1 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2);
    BMP2 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3);
    BMP3 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN5);
    BMP4 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6);
    BMP5 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7);
}


void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_R_count += 65536;
        Tach_L_count += 65536;
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_R++;   // Increment the total number of encoder events for the left encoder
        // Calculate and track the encoder count values
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        // If 6 values have been received, average them.
        if(Tach_R_sum_count == 6){
            Tach_R_avg = Tach_R_sum/6;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_L++;
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        if(Tach_L_sum_count == 6){
            Tach_L_avg = Tach_L_sum/6;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    }
}

void T1_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    run_control = 1;
}

void I2C_writeData(uint32_t moduleInstance
                  ,uint8_t PeriphAddress
                  ,uint8_t StartReg
                  ,uint8_t *data
                  ,uint8_t len)
{
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_TRANSMIT_MODE); // Indicate a write operation

    I2C_masterSendMultiByteStart(moduleInstance,StartReg); // Start the communication.
                // This function does three things. It sends the START signal,
                // sends the address, and then sends the start register.

    // This code loops through all of the bytes to send.
    uint8_t ctr;
    for(ctr = 0;ctr<len;ctr++){
        I2C_masterSendMultiByteNext(moduleInstance,data[ctr]);
    }
    // Once all bytes are sent, the I2C transaction is stopped by sending the STOP signal
    I2C_masterSendMultiByteStop(moduleInstance);

    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}

void I2C_readData(uint32_t moduleInstance
                 ,uint8_t PeriphAddress
                 ,uint8_t StartReg
                 ,uint8_t *data
                 ,uint8_t len)
{
    // First write the start register to the peripheral device. This can be
    // done by using the I2C_writeData function with a length of 0.
    I2C_writeData(moduleInstance,PeriphAddress,StartReg,0,0);

    Interrupt_disableMaster(); //  Disable all interrupts to prevent timing issues

    // Then do read transaction...
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_RECEIVE_MODE); // Indicate a read operation
    I2C_masterReceiveStart(moduleInstance); // Start the communication. This function
                // doe two things: It first sends the START signal and
                // then sends the peripheral address. Once started, the eUSCI
                // will automatically fetch bytes from the peripheral until
                // a STOP signal is requested to be sent.

    // This code loops through 1 less than all bytes to receive
    uint8_t ctr;
    for(ctr = 0;ctr<(len-1);ctr++){
        uint32_t tout_tmp = 10000;
        while(!(EUSCI_B_CMSIS(moduleInstance)->IFG & EUSCI_B_IFG_RXIFG0) && --tout_tmp); // Wait for a data byte to become available
        if(tout_tmp){
            data[ctr] = I2C_masterReceiveMultiByteNext(moduleInstance); // read and store received byte
        }else{
            data[ctr] = 0xFF;
        }
    }
    // Prior to receiving the final byte, request the STOP signal such that the
    // communication will halt after the byte is received.
    data[ctr] = I2C_masterReceiveMultiByteFinish(moduleInstance); // send STOP, read and store received byte

    Interrupt_enableMaster(); // Re-enable interrupts

    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}
