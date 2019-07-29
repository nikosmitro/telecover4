
#define     FWV  "LMC firmware version: 0.1.6"


//*****inputs(switches are normally open******
//SHUTTER
#define P1_POS_CLOSE            4   // 25 ( Atmel pin )     
#define P1_POS_OPEN             12  // 26 ( Atmel pin )
//RAMAN
#define P2_POS_CLOSE            9   // 29 ( Atmel pin )
#define P2_POS_OPEN             10  // 30 ( Atmel pin )
//CTND
#define P3_POS_CLOSE            13  // 32 ( Atmel pin )
#define P3_POS_OPEN             A2  // 38 ( Atmel pin )M,1

//P4_MOTOR
#define P4_POS_CLOSE            A5  // 41 ( Atmel pin )
#define P4_POS_OPEN             7   //  1 ( Atmel pin )

#define V_read                  A0  // 36 ( Atmel pin )


//*****outputs********************************
//SHUTTER
#define P1_M_CLOSE              2   // 19 ( Atmel pin )
#define P1_M_OPEN               3   // 18 ( Atmel pin )
//RAMAN
#define P2_M_CLOSE              8   // 28 ( Atmel pin )
#define P2_M_OPEN               6   // 27 ( Atmel pin )
//CTND
#define P3_M_CLOSE              A1  // 37 ( Atmel pin )
#define P3_M_OPEN               5   // 31 ( Atmel pin )
//P4_MOTOR
#define P4_M_CLOSE              A4  // 40 ( Atmel pin )
#define P4_M_OPEN               A3  // 39 ( Atmel pin )


//*****PWM************************************
#define PWM                     11  // 12 ( Atmel pin )

#define MOTORS_COUNT            4
bool  defaultIsMotorPresent[4]  = {true, true, true, false};
#define DELAY_TO_BRAKE          75   //ms
#define DEFAULT_TIMEOUT         7000
#define MIN_SPEED               140
#define DEFAULT_SPEED           140       //0->DutyCycle=100% (max speed), 185=DutyCycle=27% (min speed)
#define MYDIGITAL_READ(X)       (!digitalRead(X)) 
#define PULL_UP_FOR_INPUT       LOW
#define V_min                   7.0
#define V_max                   8.5
#define VREF_min                3.34
#define VREF_max                Vin
#define VREF                    (VREF_max-(((VREF_max-VREF_min)/MIN_SPEED)*Speed))
#define duty_cycle              VREF/Vin


//*****position name**************************
char MotorsNames[][9]           = {"SHUTTER", "RAMAN", "CTND", "P4_MOTOR"};


//*****motors position************************
#define P1_MOTOR                0
#define P2_MOTOR                1
#define P3_MOTOR                2
#define P4_MOTOR                3


//*****index for MotorsToMove[][index];*******
#define MOTOR_TO_OPEN           0
#define MOTOR_TO_CLOSE          1
#define MOTOR_TO_MOVE           2
#define MOTOR_TIMEOUT           3


//*****value for MotorsStatus[];**************
#define STATUS_Close            0
#define STATUS_Open             1
#define STATUS_Moving           2
#define STATUS_Not_connected    3
#define STATUS_Not_present      4
#define STATUS_Timeout          5


//*****commands*******************************
#define MOTORS_PRESENT          'A'
#define VERSION                 'V'
#define SPEED                   'F'
#define TIMEOUT                 'T'
#define SERVICE_MODE            'Q'
#define MOVE                    'M'
#define MOVE_P1                 'N'
#define MOVE_P2                 'E'
#define MOVE_P3                 'S'
#define MOVE_P4                 'W'
#define OPEN_ALL                'O'
#define CLOSE_ALL               'H'
#define DEMO                    'R'
#define GET_STATUS              'C'
