#include <Arduino.h>
//#include <SoftwareSerial.h>


/*
  Left mottors
*/
#define LM_PWM_ENA  3   // D3
#define LM_PWM_ENB  5   // D5
#define LM_IN1      2   // D2
#define LM_IN2      4   // D4
#define LM_IN3      12  // D12
#define LM_IN4      13  // D13

/*
  Right mottors
*/
#define RM_PWM_ENA  6   // D6
#define RM_PWM_ENB  9   // D9
#define RM_IN1      14  // A0
#define RM_IN2      15  // A1
#define RM_IN3      7   // A2
#define RM_IN4      8   // A3

/*
  Front-left speed encoder
*/
#define SP_FRONT_A_IN   A6
#define SP_FRONT_D_IN   A4

/*
  Rear-right speed encoder
*/
#define SP_REAR_A_IN   A5
#define SP_READ_D_IN   A7

/*
  Communication with master
*/
#define MASTER_RX_PIN 10  // D10
#define MASTER_TX_PIN 11  // D11

//------------------------------------------------------------------------------
#define MAX_SPEED_ARAY          100

enum Commands {
    cmdNone = 0,
    cmdGetInstantSpeed=1,
    cmdGetAvgSpeed=2,
    cmdSetSpeed=3,
    cmdSetDirection=4,
    cmdHeartbeat=5,
    cmdAck=6,
    cmdNack=7
};

enum CommandType {
    ctRead,
    ctWrite
};
enum Motors {
    mLeftFront,
    mLeftRear,
    mRightFront,
    mRightRear,
};

enum Directions {
    dirForward,
    dirBackward,
};

//------------------------------------------------------------------------------
//SoftwareSerial masterSerial(MASTER_RX_PIN, MASTER_TX_PIN); // RX, TX
#define masterSerial Serial


const long speedReadInterval = 1000;           // interval at which to blink (milliseconds)

char frontSpeedArray [MAX_SPEED_ARAY];
char rearSpeedArray [MAX_SPEED_ARAY];
int speedCounter = 0;


unsigned long  previousMillis = 0;
unsigned long  previousIdleMillis = 0;


//------------------------------------------------------------------------------
void setup() {

  previousMillis = 0;

  masterSerial.begin(115200);

  // setup pins
  pinMode(LM_PWM_ENA, OUTPUT);
  pinMode(LM_PWM_ENB, OUTPUT);
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(LM_IN3, OUTPUT);
  pinMode(LM_IN4, OUTPUT);

  pinMode(RM_PWM_ENA, OUTPUT);
  pinMode(RM_PWM_ENB, OUTPUT);
  pinMode(RM_IN1, OUTPUT);
  pinMode(RM_IN2, OUTPUT);
  pinMode(RM_IN3, OUTPUT);
  pinMode(RM_IN4, OUTPUT);


  pinMode(SP_FRONT_A_IN, INPUT);
  pinMode(SP_FRONT_D_IN, INPUT);

  pinMode(SP_REAR_A_IN, INPUT);
  pinMode(SP_READ_D_IN, INPUT);
}

//------------------------------------------------------------------------------
void sendSerialPacket(unsigned char cmd, unsigned char motor, unsigned char data) {
    unsigned char txData [] = { 0xCA, 0xFE, cmd, motor, data};
    masterSerial.write(txData, sizeof (txData));
}

//------------------------------------------------------------------------------
void setDirection(Motors motor, Directions dir) {

    int motorPort1 = 0;
    int motorPort2 = 0;

    int dirVal1 = 0;
    int dirVal2 = 1;

    if ( motor == mLeftFront ) {
        motorPort1 = LM_IN1;
        motorPort2 = LM_IN2;

        dirVal1 = dir;
        dirVal2 = !dir;
    }
    else if ( motor == mRightFront ) {
        motorPort1 = RM_IN1;
        motorPort2 = RM_IN2;

        dirVal1 = !dir;
        dirVal2 = dir;
    }
    else if ( motor == mLeftRear ) {
        motorPort1 = LM_IN3;
        motorPort2 = LM_IN4;

        dirVal1 = dir;
        dirVal2 = !dir;

    }
    else if ( motor == mRightRear ) {
        motorPort1 = RM_IN3;
        motorPort2 = RM_IN4;

        dirVal1 = dir;
        dirVal2 = !dir;
    }

    digitalWrite(motorPort1, dirVal1);
    digitalWrite(motorPort2, dirVal2);

}

void setSpeed(Motors motor, int speed) {
    int motorSpeedPort = 0;

    if ( motor == mLeftFront ) {
        motorSpeedPort = LM_PWM_ENA;

    }
    else if ( motor == mRightFront ) {
        motorSpeedPort = RM_PWM_ENA;

    }
    else if ( motor == mLeftRear ) {
        motorSpeedPort = LM_PWM_ENB;
    }
    else if ( motor == mRightRear ) {
        motorSpeedPort = RM_PWM_ENB;

    }
    analogWrite(motorSpeedPort, speed);
}


//------------------------------------------------------------------------------
// [0xCAFE][CMD][MOTOR][DATA]
// Cmd - read or write speed/direction
// Motor number
// Data  - speed value or direction value
// Each read should be 5 bytes
int readIndex = 0;
int rxData [3];

int decodeSerialCommand (int *motor, int *value) {
    

    if (!masterSerial.available()) {
        return cmdNone;
    }

    while (masterSerial.available()) {
        unsigned char ch = (Serial.read() & 0xFF);
        
        if (readIndex == 0 && ch == 0xCA) {
            //tft.print("CA");
            readIndex = 1;
            
        } else if (readIndex == 1 && ch == 0xFE) {
            //tft.print("FE");
            readIndex = 2;
            
        } else if (readIndex >= 2 && readIndex  <= (3 + 2)) {
            
            if (readIndex - 2 == 3) {
                readIndex = 0;
                *motor = rxData[1];
                *value = rxData[2];

                
                return rxData[0];

            }
            rxData[readIndex - 2] = ch;
            readIndex ++ ;
        } else {
           readIndex = 0;
           return cmdNone;            
        }
    }
    return cmdNone;
}
//------------------------------------------------------------------------------
char getAvgValue( char *values, int size) {

    return 0;
}

//------------------------------------------------------------------------------
void executeCommand (int cmd, int motor, int value) {

    if (cmd == cmdGetInstantSpeed) {

        if (motor == mLeftFront || motor == mRightFront) {
            sendSerialPacket(cmdGetInstantSpeed, motor, frontSpeedArray[0]);
        } else if (motor == mLeftRear || motor == mRightRear) {
            sendSerialPacket(cmdGetInstantSpeed, motor, rearSpeedArray[0]);
        }
    }
    else if (cmd == cmdGetAvgSpeed) {
        if (motor == mLeftFront || motor == mRightFront) {
            sendSerialPacket(cmdGetInstantSpeed, motor, getAvgValue(frontSpeedArray, speedCounter));
        } else if (motor == mLeftRear || motor == mRightRear) {
            sendSerialPacket(cmdGetInstantSpeed, motor, getAvgValue(rearSpeedArray, speedCounter));
        }
    }
    else if (cmd == cmdSetSpeed) {
        setSpeed(motor, value);
        sendSerialPacket(cmdAck, motor, cmdSetSpeed);
    }
    else if (cmd == cmdSetDirection) {
        setDirection(motor, value);
        sendSerialPacket(cmdAck, motor, cmdSetDirection);
    } else {
        sendSerialPacket(cmdNack, motor, cmd);
    }
}


unsigned long lastSpeedUpdate = 0;

//------------------------------------------------------------------------------
int hbCounter = 0;
void loop() {

    int motor;
    int value;

    unsigned long currentMillis = millis();
    int cmd = decodeSerialCommand(&motor, &value);

    if (cmd != cmdNone) {
        executeCommand(cmd, motor, value);
        previousIdleMillis = millis();
    } else {

        if (currentMillis - previousIdleMillis >= speedReadInterval) {

            //long ml = millis();
            // masterSerial.print(millis());
            // masterSerial.println(":: Heart beat");

            hbCounter ++;
            sendSerialPacket(cmdHeartbeat,hbCounter & 0xFF, (hbCounter >> 8) & 0xFF);
            previousIdleMillis = millis();
        }

        if (currentMillis - lastSpeedUpdate >= speedReadInterval) {
                lastSpeedUpdate = millis();

                // stop mottors if no communication from master

                setSpeed(mLeftFront, 0);
                setSpeed(mLeftRear, 0);

                setSpeed(mRightFront, 0);
                setSpeed(mRightRear, 0);
        }
    }

    // read speed from encoders in an array to caluclate acceleration value and direction
    if (currentMillis - previousMillis >= speedReadInterval) {
        if (speedCounter < MAX_SPEED_ARAY) {
            frontSpeedArray[speedCounter] = analogRead(SP_FRONT_A_IN);
            rearSpeedArray[speedCounter] = analogRead(SP_REAR_A_IN);
            speedCounter ++;
        } else {
            memcpy(&frontSpeedArray[0], &frontSpeedArray[1], (MAX_SPEED_ARAY - 1) * sizeof(char));
            frontSpeedArray[MAX_SPEED_ARAY - 1 ] = analogRead(SP_FRONT_A_IN);
            rearSpeedArray[MAX_SPEED_ARAY - 1] = analogRead(SP_REAR_A_IN);
        }
    }

}
