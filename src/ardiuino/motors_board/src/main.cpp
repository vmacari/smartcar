#include <Arduino.h>
#include <SoftwareSerial.h>

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
    cmdNone,
    cmdGetSpeed,
    cmdSetSpeed,
    cmdSetDirection
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
SoftwareSerial masterSerial(MASTER_RX_PIN, MASTER_TX_PIN); // RX, TX

const long speedReadInterval = 1000;           // interval at which to blink (milliseconds)

char frontSpeedArray [MAX_SPEED_ARAY];
char rearSpeedArray [MAX_SPEED_ARAY];
int speedCounter = 0;


unsigned long  previousMillis = 0;
unsigned long  previousIdleMillis = 0;


//------------------------------------------------------------------------------
void setup() {

  previousMillis = 0;
  masterSerial.begin(9600);

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
int decodeSerialCommand (int *motor, int *value) {
    char comm_data[3];

    if (!masterSerial.available()) {
        return cmdNone;
    }

    if (masterSerial.readBytes(comm_data, 2) != 2) {
        return cmdNone;
    }

    if (comm_data[0] != 0xCA || comm_data[1] != 0xFE)  {
        return cmdNone;
    }

    if (masterSerial.readBytes(comm_data, 3) != 3) {
        return cmdNone;
    }

    *motor =  comm_data[1];
    *value =  comm_data[2];
    return comm_data[0];
}
//------------------------------------------------------------------------------
char getAvgValue( char *values, int size) {

    return 0;
}

//------------------------------------------------------------------------------
void executeCommand (int cmd, int motor, int value) {
    if (cmd == cmdGetSpeed) {

        char response[]= {
            0xCA, 0xFE, 
            cmdGetSpeed, 
            frontSpeedArray[0], 
            rearSpeedArray[0], 
            getAvgValue(frontSpeedArray, speedCounter),
            getAvgValue(rearSpeedArray, speedCounter)

        };

        // response [3] = ;
        // response [4] = rearSpeedArray[0];

        // response []


        masterSerial.write(response, sizeof(response));

        //
        // if (motor == mLeftFront || motor == mRightFront) {
        //     masterSerial.write(frontSpeedArray[0]);
        // } else if (motor == mLeftRear || motor == mRightRear) {
        //     masterSerial.write(rearSpeedArray[0]);
        // }
    }
    else if (cmd == cmdSetSpeed) {
        setSpeed(motor, value);
    }
    else if (cmd == cmdSetDirection) {
        setDirection(motor, value);
    }

}

unsigned long lastSpeedUpdate = 0;

//------------------------------------------------------------------------------
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
            masterSerial.print(millis());
            masterSerial.println(":: Heart beat");
            previousIdleMillis = millis();
        }

        if (currentMillis - lastSpeedUpdate >= speedReadInterval) {
                lastSpeedUpdate = millis();

                // stop mottors if no communication from master
                executeCommand(cmdSetSpeed, mLeftFront, 0);
                executeCommand(cmdSetSpeed, mLeftRear, 0);
                executeCommand(cmdSetSpeed, mRightFront, 0);
                executeCommand(cmdSetSpeed, mRightRear, 0);
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
