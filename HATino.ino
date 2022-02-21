// Arduino sketch for MPU6050 on NanoWII using DMP MotionApps v4.1 
// HAT 14/04/2013 by FuraX49
//
// Head Arduino Tracker  for FaceTrackNoIR 
//   http://facetracknoir.sourceforge.net/home/default.htm	
// I2C device class (I2Cdev)
//   https://github.com/jrowberg/i2cdevlib
//
// Some fixes done by Juan M. Casillas <juan,.casillas@gmail.com>
// Code cleanup, Serial fixes and porting to Arduino Micro.
//
//          ARDUINOMICRO GY-521
//          -------------------------------
// PINOUT:  SDA (D2)      SDA
//          SCL (D3)      SCL
//          5V            VCC
//          GND           GND
//                        XDA Not connected
//                        XCL Not Connected
//                        ADO Not Connected (my board, low, i2c address 0x68. High, 0x69)
//                        INT Not Connected
// 
//  NO PULLOUT resistors in SDA, SCL
//


#include <avr/eeprom.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

//#define DEBUG_JMC 1

#define BTSERIAL 1

MPU6050 mpu;
SoftwareSerial BTSerial(8, 9); // RX | TX

typedef struct  {
  int16_t  Begin  ;   // 2  Debut
  uint16_t Cpt ;      // 2  Compteur trame or Code info or error
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End ;      // 2  Fin
} _hatire;

typedef struct  {
  int16_t  Begin  ;   // 2  Debut
  uint16_t Code ;     // 2  Code info
  char     Msg[24];   // 24 Message
  int16_t  End ;      // 2  Fin
} _msginfo;

typedef struct 
{
  byte   rate;
  double gyro_offset[3] ;
  double acc_offset[3]  ;
} _eprom_save;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
bool dmpLoaded = false;  // set true if DMP loaded  successfuly
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

char Commande;
char Version[] = "HAT V 1.10";
const byte      INTERRUPT_PINNUMBER = 7;                    // where the INT is attached. Varies from Arduinos

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float Rad2Deg = (180/M_PI) ;

// trame for message 
_hatire hatire;
_msginfo msginfo;
_eprom_save eprom_save;


bool   AskCalibrate = false;  // set true when calibrating is ask
int    CptCal =  0;
const int    NbCal = 5; 




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               PRINT SERIAL FORMATTE                      ===
// ================================================================
void PrintCodeSerial(uint16_t code,const char Msg[24],bool EOL ) {
  msginfo.Code=code;
  memset(msginfo.Msg,0x00,24);
  strcpy(msginfo.Msg,Msg);
  if (EOL) msginfo.Msg[23]=0x0A;
  // Send HATIRE message to  PC
  

#ifndef BTSERIAL
    Serial.write((byte*)&msginfo,30);
#else
    BTSerial.write((byte*)&msginfo,30);
  #endif
}


   
  
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24;      
  // put the i2c bus at 200kHz to prevent buffer Overflow Don't work so much.
  // problems are related with the SERIAL port (to low)
  // try to write less packets.

  // initialize serial communication

#ifndef BTSERIAL
  Serial.begin(115200); // max speed supported by OpenTrack
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
#else
  // BT INIT
  
  pinMode(10, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  // unconnect it or set it to low.
  digitalWrite(10, LOW);
  BTSerial.begin(115200);  // HC-05 default speed in AT command more
#endif  
  
  //Serial.begin(115200);
  //Serial.begin(230440); // runs smooth
  

  // From arduino manual
  // The default is 8 data bits, no parity, one stop bit.
  // baudrate:  115200
  // data bits: 8
  // parity:  None
  // stop bit: 1
  // Flow-Control:  None.
  
  
  
  
  
  PrintCodeSerial(2000,Version,true);

  hatire.Begin=0xAAAA;
  hatire.Cpt=0;
  hatire.End=0x5555;

  msginfo.Begin=0xAAAA;
  msginfo.Code=0;
  msginfo.End=0x5555;

  // initialize device
  PrintCodeSerial(3001,"Initializing I2C",true);
  mpu.initialize();

  // verify connection
  PrintCodeSerial(3002,"Testing connections",true);

  if (mpu.testConnection()){
     PrintCodeSerial(3003,"MPU6050 connection OK",true);
  } else {
     PrintCodeSerial(9007,"MPU6050 ERRROR CNX",true);
  }

  while (Serial.available() && Serial.read()); // empty buffer

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  // load and configure the DMP
  PrintCodeSerial(3004,"Initializing DMP...",true);
  devStatus = mpu.dmpInitialize();

  // JMC. Set the offsets for this thing (from MPU6050_DMP6 demo program)

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    dmpLoaded=true;

    // Read Epprom saved params
    PrintCodeSerial(3005,"Reading saved params...",true);
    //ReadParams();

    mpu.setXGyroOffset(68);
    mpu.setYGyroOffset(-8); 
    mpu.setZGyroOffset(12);
    mpu.setXAccelOffset(-257); // 1688 factory default for my test chip
    mpu.setYAccelOffset(638); // 1688 factory default for my test chip
    mpu.setZAccelOffset(1220); // 1688 factory default for my test chip
    
    // turn on the DMP, now that it's ready
    PrintCodeSerial(3006,"Enabling DMP...",true);
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    PrintCodeSerial(3007,"Enabling interrupt",true);
    // jmc ¿ what pin ? you should point to the required one
    // INT0 -> SCL in Micro. D3
    // INT1 -> SDA           D2 
    // INT2 -> TX (Serial)   RX1 
    // INT3 -> RX (Serial)   TX1
    // INT4 ??? 
     
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PINNUMBER), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    PrintCodeSerial(5000,"HAT BEGIN",true);
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();    

    // JMC this is NOT in the example -- remove it
    // Empty FIFO
    /*
    fifoCount = mpu.getFIFOCount();  
    while (fifoCount > packetSize) {
      fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, fifoCount);
    }
    */
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    dmpLoaded=false;
    PrintCodeSerial(9000+devStatus,"DMP Initialization failed",true);
  }
}


// ================================================================
// ===                    RAZ OFFSET                            ===
// ================================================================
void razoffset() {
  eprom_save.gyro_offset[0] = 0;
  eprom_save.gyro_offset[1] = 0;
  eprom_save.gyro_offset[2] = 0;
  eprom_save.acc_offset[0] = 0;
  eprom_save.acc_offset[1] = 0;
  eprom_save.acc_offset[2] = 0;
}


// ================================================================
// ===                    SAVE PARAMS                           ===
// ================================================================
void SaveParams() {
  eeprom_write_block((const void*)&eprom_save, (void*) 0, sizeof(eprom_save));
}



// ================================================================
// ===                    READ PARAMS                           ===
// ================================================================
void ReadParams() {
  eeprom_read_block( (void*)&eprom_save, (void*) 0, sizeof(eprom_save));
}


// ================================================================
// ===                    Serial Command                        ===
// ================================================================
void serialEvent(){
  Commande = (char)Serial.read();
  switch (Commande) {
  case 'S':
    PrintCodeSerial(5001,"HAT START",true);
    if (dmpLoaded==true) {
      mpu.resetFIFO();   
      hatire.Cpt=0;              
      attachInterrupt(0, dmpDataReady, RISING);
      mpu.setDMPEnabled(true);
      dmpReady = true;
    } 
    else {
      PrintCodeSerial(9011,"Error DMP not loaded",true);
    }
    break;      

  case 's':
    PrintCodeSerial(5002,"HAT STOP",true);
    if (dmpReady==true) {
      mpu.setDMPEnabled(false);
      detachInterrupt(0);
      dmpReady = false;
    }
    break;      

  case 'R':
    PrintCodeSerial(5003,"HAT RESET",true);
    if (dmpLoaded==true) {
      mpu.setDMPEnabled(false);
      detachInterrupt(0);
      mpu.resetFIFO();   
      hatire.Cpt=0;              
      dmpReady = false;
      setup();
    } 
    else {
      PrintCodeSerial(9011,"Error DMP not loaded",true);
    }
    break;      


  case 'C':
    CptCal=0;
    razoffset();  
    AskCalibrate=true;
    break;      

  case 'V':
    PrintCodeSerial(2000,Version,true);
    break;      

  case 'I':
  	BTSerial.println();
  	BTSerial.print("Version : \t");
    BTSerial.println(Version);
    BTSerial.println("Gyroscopes offsets");
    for (int i=0; i <= 2; i++) {
  	  BTSerial.print(i);
  	  BTSerial.print(" : ");
    	BTSerial.print(eprom_save.gyro_offset[i]);
  	  BTSerial.println();
    }
    BTSerial.println("Accelerometers offsets");
    for (int i=0; i <= 2; i++) {
  	  BTSerial.print(i);
  	  BTSerial.print(" : ");
    	BTSerial.print(eprom_save.acc_offset[i]);
  	  BTSerial.println();
    }
    break;      


  default:
    break;
  }	
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // Leonardo BUG (simul Serial Event)
  if(Serial.available() > 0)  serialEvent();


  // if programming failed, don't try to do anything
  if (dmpReady)  { 


    while ((!mpuInterrupt) && (fifoCount < packetSize)) ;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      PrintCodeSerial(9010,"Overflow FIFO DMP",true);
      hatire.Cpt=0;

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // Get Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(hatire.gyro, &q, &gravity);

 

      // Get real acceleration, adjusted to remove gravity
      // not used in this script
      // mpu.dmpGetAccel(&aa, fifoBuffer);
      // mpu.dmpGetLinearAccel(&hatire.acc, &aa, &gravity);

      // Calibration sur X mesures  
      if (AskCalibrate) {
        if ( CptCal>=NbCal) {
          CptCal=0;
          eprom_save.gyro_offset[0] = eprom_save.gyro_offset[0] / NbCal ;
          eprom_save.gyro_offset[1] = eprom_save.gyro_offset[1] / NbCal ;
          eprom_save.gyro_offset[2] = eprom_save.gyro_offset[2] / NbCal ;
          AskCalibrate=false;
          SaveParams();
        } 
        else {
          eprom_save.gyro_offset[0] += (float) hatire.gyro[0];
          eprom_save.gyro_offset[1] += (float) hatire.gyro[1];
          eprom_save.gyro_offset[2] += (float) hatire.gyro[2];

          CptCal++;
        }
      }



      // adjust board position. By default works if the GY-521 board is set flat (chip and led facing up)
      // and pin solders facing to right. far pin should be INT, near pin should be VCC. X Asis is the 
      // board plane, Y axis the perpendicular to it.
      
      // if you want rotate the board 90 ANTICLOCKWISE you should add + PI/2 angle to X (add it in degrees)

      // rotate 90 to LEFT (ANTICLOCWISE)

        hatire.gyro[0] += PI/2;


      // Conversion angles Euler en +-180 Degrees
      for (int i=0; i <= 2; i++) {
        hatire.gyro[i]= (hatire.gyro[i] - eprom_save.gyro_offset[i] ) * Rad2Deg;
        if  (hatire.gyro[i]>180) {
          hatire.gyro[i] = hatire.gyro[i] - 360;
        }
      }




      if (AskCalibrate) {
        hatire.gyro[0] = 0; 
        hatire.gyro[1] = 0;
        hatire.gyro[2] = 0;
        hatire.acc[0]= 0;
        hatire.acc[1] = 0;
        hatire.acc[2] = 0;
      }



      // Send Trame to HATIRE PC
      // DEBUG THE THING
#ifndef BTSERIAL
        Serial.write((byte*)&hatire,30);
#else
        BTSerial.write((byte*)&hatire,30);    
      #endif
      

      hatire.Cpt++;
      if (hatire.Cpt>999) {
        hatire.Cpt=0;
      }
    }
  }
  delay(1);
}


