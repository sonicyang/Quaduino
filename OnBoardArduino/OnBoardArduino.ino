#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//pin definition
const int PIN_RF_CS = 3;
const int PIN_RF_EN = 4;
const int PIN_ESC_FRONT = 5;
const int PIN_ESC_BACK = 6;
const int PIN_ESC_LEFT = 9;
const int PIN_ESC_RIGHT = 10;

RF24 radio( PIN_RF_CS, PIN_RF_EN );
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//ESC control
Servo front;
Servo back;
Servo left;
Servo right;

//Basic Ctrl
int offest = 0;
unsigned long last_time_loop = 0;
int first = 1;
int modeCtl = 2;

//x axis speed PID ctrl
double x_P = 0.120;
double x_I = 0.097;
double x_D = 0.011;
double x_accmulate_I = 0;
double x_last_D = 0;
double x_last_angle = 0;
double x_target = 0;
double x_adjustment = 0;

//x axis position PD ctrl
double xp_P = 0.133;
double xp_D = -0.082;
double xp_last_D = 0;
double xp_target = 0;

//y axis PID ctrl
double y_P = 0.130;
double y_I = 0.133;
double y_D = 0.024;
double y_accmulate_I = 0;
double y_last_D = 0;
double y_last_angle = 0;
double y_target = 0;
double y_adjustment = 0;

//y axis position PD ctrl
double yp_P = 0.082;
double yp_D = -0.075;
double yp_last_D = 0;
double yp_target = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void attachESCs(){
    front.attach(PIN_ESC_FRONT);
    back.attach(PIN_ESC_BACK);
    left.attach(PIN_ESC_LEFT);
    right.attach(PIN_ESC_RIGHT);
}

void haltESCs(){
    front.write(10);
    back.write(10);
    left.write(10);
    right.write(10);
}

void configureDMP(){
        // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void configureRadio(){
    radio.begin();
    radio.setRetries(15,15);
    radio.openReadingPipe(1,pipes[0]);
    radio.startListening();
}

boolean radioGetPacket(){
        //safety measures
        static int radio_failcounter = 0;
        static int acti = 0;
        if ( !radio.available() ){
            radio_failcounter++;
            if(radio_failcounter > 200){
                haltESCs();
                acti = 0;
            }
        }else{
            float data[7];
            int ctl;
            radio.read( data, sizeof(float)*7 );
            
            ctl = data[2];
            
            modeCtl = ctl & 0x111E;
            if(modeCtl == 2){
                x_target = data[0];
                y_target = data[1];
            } else if(modeCtl == 4){
                xp_target = data[0];
                yp_target = data[1];
            }
            
            offest = data[3];
            
            //yp_P = data[4];
            //yp_I = data[5];
            // yp_D = data[6];

            radio_failcounter = 0;

            acti = ctl & 0x0001;
        }
        return acti;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(115200);
    
    configureRadio();
    
    mpu.initialize();
    configureDMP();
    
    attachESCs();
    haltESCs();

    delay(3000);
    
    Serial.println("Init done");
    last_time_loop = micros();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize);
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);      
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;      
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        //calibrate the default error
        ypr[1] += 0.13;
        ypr[2] -= 0.01;
               
        //Over rotating auto-stop
        if ((ypr[1] * 180/M_PI) > 75 || (ypr[1] * 180/M_PI) < -75 || (ypr[2] * 180/M_PI) > 75 || (ypr[2] * 180/M_PI) < -75){
              haltESCs();
              while(1);
        }	

        //First loop setup        
        if(first){
            x_last_angle = ypr[1] * 180/M_PI;
            y_last_angle = ypr[2] * 180/M_PI;
            last_time_loop = micros();
            first = 0;
            return;
        }	
            
       int activate = radioGetPacket();
       
       
       
       //Activate spilit
       if(activate){ 
           //PID setup
           double time_passed = (micros() - last_time_loop)*(10E-6);
           x_adjustment = 0;
           y_adjustment = 0;
           
           if(modeCtl == 4){
               //Pitch Position PD controller
               double xp_err = (int)(ypr[1] * 180/M_PI)*10/10.0 - xp_target;
               x_target = (-1)*((xp_P * xp_err) + (xp_D * (xp_err-xp_last_D) / time_passed));
               xp_last_D = xp_err;
               
               //Roll Position PD controller
               double yp_err = (int)(ypr[2] * 180/M_PI)*10/10.0 - yp_target;
               y_target = (-1)*((yp_P * yp_err) + (yp_D * (yp_err-yp_last_D) / time_passed));
               yp_last_D = yp_err;
               
               /*if (x_target < -20)
                   x_target = -20;
               if (x_target > 20)
                   x_target = 20;
               if (y_target < -20)
                   y_target = -20;
               if (y_target > 20)
                   y_target = 20;*/
           }

           //Pitch speed PID controller
	   double x_err = (int)(((ypr[1] * 180/M_PI - x_last_angle) / time_passed) * 10)/10.0 - x_target;
           x_accmulate_I += x_err * time_passed;
           if(x_accmulate_I > 1500 || x_accmulate_I < -1500)
               x_accmulate_I = 0;
           x_adjustment = (x_P * x_err) + (x_I * x_accmulate_I) + (x_D * (x_err-x_last_D) / time_passed);
           x_last_D = x_err;
           x_last_angle = ypr[1] * 180/M_PI;
           
           Serial.print("Pitch valuse:");
           Serial.println(ypr[1] * 180/M_PI);
           Serial.print("Pitch Volecity:");
           Serial.println((int)(((ypr[1] * 180/M_PI - x_last_angle) / time_passed) * 10)/10.0);
           Serial.print("Pitch Volecity Target:");
           Serial.println(x_target);
           
            
           //Roll speed PID controller            
           double y_err = (int)(((ypr[2] * 180/M_PI - y_last_angle) / time_passed) * 10)/10.0 - y_target;
           y_accmulate_I += y_err * time_passed;
           if(y_accmulate_I > 1500 || y_accmulate_I < -1500)
               y_accmulate_I = 0;
           y_adjustment = (y_P * y_err) + (y_I * y_accmulate_I) + (y_D * (y_err-y_last_D) / time_passed);
           y_last_D = y_err;
           y_last_angle = ypr[2] * 180/M_PI;
        
           //Rounding up Power value
           int power_f = floor(offest - x_adjustment + 0.5);
           int power_b = floor(offest + x_adjustment + 0.5);
           int power_l = floor(offest - y_adjustment + 0.5);
           int power_r = floor(offest + y_adjustment + 0.5);
            
           //Make sure the Power value are with in the driver Interval
           if (power_f < 75)
               power_f = 75;
           if (power_f > 180)
               power_f = 180;
           if (power_b < 75)
               power_b = 75;
           if (power_b > 180)
               power_b = 180;
           if (power_l < 75)
               power_l = 75;
           if (power_l > 180)
               power_l = 180;
           if (power_r < 75)
               power_r = 75;
           if (power_r > 180)
               power_r = 180;
           
           //put up new power value
           front.write(power_f);
           back.write(power_b);
           left.write(power_l);
           right.write(power_r);
           
           

       }else{
           //Non-Activated
           haltESCs();
           y_accmulate_I = 0;
           x_accmulate_I = 0;
        }
        
        //Save up Time for dt
        last_time_loop = micros();
    }
}
