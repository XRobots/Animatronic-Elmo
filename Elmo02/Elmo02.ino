#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>                          // https://github.com/siteswapjuggler/RAMP
#include <Servo.h>

Servo servo1;   // left slider
Servo servo2;   // right slider
Servo servo3;   // left lifter
Servo servo4;   // right lifter

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
 
    int16_t menuDown;      
    int16_t Select; 
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

int servo1Middle = 1400;    //middle
int servo2Middle = 1500;    //middle
int servo3Down = 1300;    //down
int servo4Down = 1600;    //down

int servo1Back = 2300;    //back
int servo2Back = 500;     //back
int servo3Up = 1900;    //up orig 2300
int servo4Up = 1000;     //up orig 600

int servo1Forward = 400;    //forward
int servo2Forward = 2400;   //forward

int currentServo1;
int currentServo2;
int currentServo3;
int currentServo4;

int currentServo1a;
int currentServo2a;
int currentServo3a;
int currentServo4a;

RECEIVE_DATA_STRUCTURE mydata_remote;

unsigned long currentMillis;
unsigned long previousMillis = 0;       // main loop
unsigned long previousSafetyMillis = 0;

unsigned long previousForwardMillis = 0;

int output1;
int output2;
int output1a;
int output2a;

int walkingSpeedForward;
int walkingSpeedBackward;
int walkingSpeed;

int forwardFlag = 0;
int backwardFlag = 0;

int forwardTimer;
int backwardTimer;

class Interpolation {  
public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;    

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
          interpolationFlag = 1;
      }
    
      int output = myRamp.update();               
      return output;
    }
};    // end of class

// interpolation objects

Interpolation interp1;      // left slider   
Interpolation interp2;      // right slider
Interpolation interp3;      // left lifter
Interpolation interp4;      // right lifter

void setup() {

  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);  
  radio.startListening();

  servo1.attach(40);
  servo2.attach(42);
  servo3.attach(44);
  servo4.attach(46);

  servo1.writeMicroseconds(servo1Middle);     // left slider
  servo2.writeMicroseconds(servo2Middle);     // right slider
  servo3.writeMicroseconds(servo3Down);     // left lifter
  servo4.writeMicroseconds(servo4Down);     // right lefter

}

void loop() {

        currentMillis = millis();   
        if (currentMillis - previousMillis >= 10) {     // start of 10ms cycle
        previousMillis = currentMillis;   // reset the clock so the loop timing is correct

            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis; 
            } 

            if(currentMillis - previousSafetyMillis > 200) {         
                Serial.print("*no remote data*   ");
                mydata_remote.RLR = 512;
                mydata_remote.RFB = 512;
                mydata_remote.RT = 512;
                mydata_remote.LLR = 512;
                mydata_remote.LFB = 512;
                mydata_remote.LT = 512;
            } 
/*
             
            // print control data, count and mode to terminal
            Serial.print(mydata_remote.menuDown);
            Serial.print(" , ");
            Serial.print(mydata_remote.Select);
            Serial.print(" , ");
            Serial.print(mydata_remote.menuUp);
            Serial.print(" , ");
            Serial.print(mydata_remote.toggleTop);
            Serial.print(" , ");
            Serial.print(mydata_remote.toggleBottom);
            Serial.print(" *** ");
            Serial.print(mydata_remote.RLR);
            Serial.print(" , ");            
            Serial.print(mydata_remote.RFB);
            Serial.print(" , ");
            Serial.print(mydata_remote.RT);
            Serial.print(" , ");
            Serial.print(mydata_remote.LLR);
            Serial.print(" , ");
            Serial.print(mydata_remote.LFB);
            Serial.print(" , ");
            Serial.print(mydata_remote.LT);
            Serial.println(" *** ");
           */

            mydata_remote.RFB = constrain(mydata_remote.RFB,150,900);
            mydata_remote.RLR = constrain(mydata_remote.RLR,400,650);

            output1 = (mydata_remote.RFB - 512) + (mydata_remote.RLR - 512);
            output2 = (mydata_remote.RFB - 512) - (mydata_remote.RLR - 512);

            output1a = map(output1,-512,512,-80,80); 
            output2a = map(output2,-512,512,-80,80); 

            output1a = constrain(output1a,-100,100);
            output2a = constrain(output2a,-100,100);

            /*
            Serial.print(output1a);
            Serial.print(" , ");
            Serial.println(output2a);
            */

            // right motor

            if (output1a > 15) {
              output1a = abs(output1a);
              analogWrite(2, output1a);
              analogWrite(3, 0);
            }
            else if (output1a < -15) {
              output1a = abs(output1a);
              analogWrite(3, output1a);
              analogWrite(2, 0);
            }
            else {
              analogWrite(3, 0);
              analogWrite(2, 0);
            }

            // left motor

            if (output2a > 15) {
              output2a = abs(output2a);
              analogWrite(5, output2a);
              analogWrite(4, 0);
            }
            else if (output2a < -15) {
              output2a = abs(output2a);
              analogWrite(4, output2a);
              analogWrite(5, 0);
            }
            else {
              analogWrite(5, 0);
              analogWrite(4, 0);
            }       

            // interpolation test

            /*
            int test1 = interp1.go(2000,2000);
            Serial.print(test1);
            Serial.print(" , ");
            int test2 = interp2.go(3000,1000);
            Serial.println(test2);
            */

            // drive leg servos

            walkingSpeedForward = constrain(mydata_remote.RFB,0,500);
            walkingSpeedForward = map(walkingSpeedForward,500,0,900,0);

            walkingSpeedBackward = constrain(mydata_remote.RFB,550,1023);
            walkingSpeedBackward = map(walkingSpeedBackward,550,1023,1000,0);

            /*
            Serial.print(walkingSpeedForward);
            Serial.print(" , ");
            Serial.println(walkingSpeedBackward);
            */


            if (walkingSpeedForward > 850 && walkingSpeedBackward > 850) {    // standing still

              currentServo1 = servo1Middle;
              currentServo2 = servo2Middle;
              currentServo3 = servo3Down;
              currentServo4 = servo4Down;   
              forwardFlag = 0;           
            }            

            else if (walkingSpeedForward < 850) {      // he is walking forward

              if (forwardFlag == 0) {             
                  currentServo1 = servo1Back;
                  currentServo2 = servo2Forward;
                  currentServo4 = servo4Up;
                  forwardFlag = 1;
                  previousForwardMillis = currentMillis;                    
              }  

              else if (forwardFlag == 1 && currentMillis - previousForwardMillis > walkingSpeedForward) {
                  currentServo1 = servo1Forward;
                  currentServo3 = servo3Up;
                  currentServo4 = servo4Down;
                  forwardFlag = 2;
                  previousForwardMillis = currentMillis;
              }
              else if (forwardFlag == 2 && currentMillis - previousForwardMillis > walkingSpeedForward) {
                  currentServo2 = servo2Back;
                  currentServo1 = servo1Forward;
                  forwardFlag = 3;
                  previousForwardMillis = currentMillis;
              }
              else if (forwardFlag == 3 && currentMillis - previousForwardMillis > walkingSpeedForward) {
                  currentServo3 = servo3Down;
                  currentServo2 = servo2Forward;
                  currentServo4 = servo4Up;
                  forwardFlag = 4;
                  previousForwardMillis = currentMillis;
              }
              else if (forwardFlag == 4 && currentMillis - previousForwardMillis > walkingSpeedForward) {
                  // add some extra time to the first stage when we loop
                  forwardFlag = 0;
                  previousForwardMillis = currentMillis;
              }
            }
            

            // calc actual walking speed for interpolation 

            if (walkingSpeedForward < 850) {
              walkingSpeed = walkingSpeedForward;
            }
            else if (walkingSpeedBackward < 850) {
              walkingSpeed = walkingSpeedBackward;
            }
            else walkingSpeed = 400; // do it quicker when it comes to rest            
               
            currentServo1a = interp1.go(currentServo1,walkingSpeed);
            currentServo2a = interp2.go(currentServo2,walkingSpeed);
            currentServo3a = interp3.go(currentServo3,walkingSpeed);
            currentServo4a = interp4.go(currentServo4,walkingSpeed); 

            // write out servo positions 

            servo1.writeMicroseconds(currentServo1a);
            servo2.writeMicroseconds(currentServo2a);
            servo3.writeMicroseconds(currentServo3a);
            servo4.writeMicroseconds(currentServo4a);        


        
       }    // end of timed event


}
