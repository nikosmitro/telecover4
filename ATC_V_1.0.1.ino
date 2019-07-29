//Raymetrics ATC400-LMC100 firmware
//Release version: 1.1.6
//Release date: 18-06-2018
//Artist: Harris Triantafillidis
//Raymetrics S.A. all rights reserved

#define     IS_ATC                //define product
//#define     IS_LMC

#ifdef IS_ATC
#include "ATC_Settings.h"
#endif
#ifdef IS_LMC
#include "LMC_Settings.h"
#endif

#include <EEPROM.h>
//#include "IO.h"
#include <avr/eeprom.h>


//***********************  GLOBAL VARIABLES **********************

char            SerialBuffer[SERIAL_BUFFER_SIZE];
unsigned char   SerialBufferIndex = 0;
char            S_CMD;
bool            service_mode = false;

int             Error = 0, Previous_Error = 0, V_temp = 0,
                TimeOut = DEFAULT_TIMEOUT;    // TimeOut is readed from eeprom

unsigned char   Speed = DEFAULT_SPEED;


int   delayNorth = DEFAULT_DELAY_NORTH;
int   delaySouth = DEFAULT_DELAY_SOUTH;
int   delayEast = DEFAULT_DELAY_EAST;
int   delayWest = DEFAULT_DELAY_WEST;

int   time_for_move[5]          = {0, 0, 0, 0, 0};
bool  IsMotorPresent[4]         = {true, true, true, true};
int   MotorsPOS_OPEN_Pins[4]    = {P1_POS_OPEN, P2_POS_OPEN, P3_POS_OPEN, P4_POS_OPEN};
int   MotorsPOS_CLOSE_Pins[4]   = {P1_POS_CLOSE, P2_POS_CLOSE, P3_POS_CLOSE, P4_POS_CLOSE};
int   MotorsM_CLOSE_Pins[4]     = {P1_M_CLOSE, P2_M_CLOSE, P3_M_CLOSE, P4_M_CLOSE};
int   MotorsM_OPEN_Pins[4]      = {P1_M_OPEN, P2_M_OPEN, P3_M_OPEN, P4_M_OPEN};
bool  MotorsToMove[4][4]        = { {false, false, false, false},
  {false, false, false, false},
  {false, false, false, false},
  {false, false, false, false}
};
byte  MotorsStatus[4]           = {0, 0, 0, 0};

unsigned long   LastReset;

float           Vin = 0;


//**** variables in eeprom  ********
unsigned char EEMEM             eeSpeed;
int           EEMEM             eeTimeOut;
bool          EEMEM             eeIsMotorPresent[MOTORS_COUNT];
int           EEMEM             eedelayNorth;
int           EEMEM             eedelaySouth;
int           EEMEM             eedelayEast;
int           EEMEM             eedelayWest;


typedef struct move_info {
  bool pincheck_end;
  uint32_t time_end;
} Move_Info;


void Dos232();
bool motor_do_the_move(int pinhigh, int pinlow, int pincheck, unsigned long break_delay);
bool* two_motors_do_the_move(int pinhigh1, int pinhigh2, int pinlow1, int pinlow2, int pincheck1, int pincheck2, unsigned long break_delay1, unsigned long break_delay2, bool NS);
void start_moving(int pinhigh, int pinlow);
Move_Info check_moving(int pincheck);
void stop_moving(int pinhigh, int pinlow);
/*
   not call directly the motor_do_the_move
*/
void internal_motor_move(unsigned char motor);
void internal_two_motors_move(unsigned char motor1, unsigned char motor2);
/*
   not call directly the motor_move
*/
void p1_move();
void p2_move();
void p3_move();
void p4_move();
void move_all_motors(int seq[6], bool open[6], int use_delay = 200);
void open_all();
void close_all();

void p1(void);
void p3(void);
void p2(void);
void p4(void);

void V_input();

void get_status();
void print_status();
void ExecuteCommands();
int do_cmd_motor(int motor);
int motor_id_from_char(char m);
void readPresentMotors();
void writePresentMotors();

void (*g_functions[4])(void) = {p1, p2, p3,  p4};


void setup()
{
  Serial.begin(9600);

  int i;
  for (i = 0; i < MOTORS_COUNT; i++)
  {
    //Input setup
    pinMode(MotorsPOS_OPEN_Pins[i], INPUT);
    pinMode( MotorsPOS_CLOSE_Pins[i], INPUT);
    digitalWrite( MotorsPOS_CLOSE_Pins[i], PULL_UP_FOR_INPUT);
    digitalWrite(MotorsPOS_OPEN_Pins[i], PULL_UP_FOR_INPUT);

    //Output setup
    pinMode(MotorsM_OPEN_Pins[i], OUTPUT);
    pinMode(MotorsM_CLOSE_Pins[i], OUTPUT);
    digitalWrite(MotorsM_OPEN_Pins[i], LOW);
    digitalWrite(MotorsM_CLOSE_Pins[i], LOW);
  }
  pinMode(PWM, OUTPUT);
  analogWrite(PWM, Speed);

  Speed = eeprom_read_byte((uint8_t *) &eeSpeed);
  //  Speed=EEPROM.read(0);
  if (Speed > MIN_SPEED)      //if EEPROM value out of limits, set Speed to default
  {
    Speed = DEFAULT_SPEED;
    eeprom_write_byte((uint8_t *)&eeSpeed, Speed);
  }

  TimeOut = eeprom_read_word((uint16_t *) &eeTimeOut);
  if (TimeOut <= 0 || TimeOut > 16000)
  {
    TimeOut = DEFAULT_TIMEOUT;
    eeprom_write_word((uint16_t *)&eeTimeOut, TimeOut);
  }

  delayNorth = eeprom_read_dword((uint32_t *) &eedelayNorth);
  if (delayNorth <= 0)
  {
    delayNorth = DEFAULT_DELAY_NORTH;
    eeprom_update_dword((uint32_t *)&eedelayNorth, delayNorth);
  }

  delaySouth = eeprom_read_dword((uint32_t *) &eedelaySouth);
  if (delaySouth <= 0)
  {
    delaySouth = DEFAULT_DELAY_SOUTH;
    eeprom_update_dword((uint32_t *)&eedelaySouth, delaySouth);
  }

  delayEast = eeprom_read_dword((uint32_t *)&eedelayEast);
  if (delayEast <= 0)
  {
    delayEast = DEFAULT_DELAY_EAST;
    eeprom_update_dword((uint32_t *)&eedelayEast, delayEast);
  }

  delayWest = eeprom_read_dword((uint32_t *)&eedelayWest);
  if (delayWest <= 0)
  {
    delayWest = DEFAULT_DELAY_WEST;
    eeprom_update_dword((uint32_t *)&eedelayWest, delayWest);
  }

  readPresentMotors();

  while (!Serial) {;}

  get_status();
  print_status();
  Previous_Error = Error;
}

void Dos232()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      if (SerialBufferIndex == 0) continue;
      else
      {
        ExecuteCommands();
        //  clear the buffer
        SerialBufferIndex = 0;
        SerialBuffer[SerialBufferIndex] = 0;
        return;   //  retrun to main loop
        //  if there are data in Serial
        //  we will read at the next call
      }
    }
    else
    {
      SerialBuffer[SerialBufferIndex++] = c;
      if (SerialBufferIndex > SERIAL_BUFFER_SIZE)
      {
        SerialBufferIndex = 0;
      }
      SerialBuffer[SerialBufferIndex] = 0; //mark end of buffer
    }
  }
}

void loop()
{
  Dos232();
  get_status();
  if (Previous_Error != Error)
  {
    Previous_Error = Error;
    print_status();
  }
  analogWrite(PWM, Speed);
}

void start_moving(int pinhigh, int pinlow)
{
  //start the motor
  digitalWrite(pinhigh, HIGH);
  digitalWrite(pinlow, LOW);
}

void stop_moving(int pinhigh, int pinlow)
{
  //stop the motor
#ifdef IS_ATC
  digitalWrite(pinhigh, LOW);
  digitalWrite(pinlow, LOW);
  delay(DELAY_TO_BRAKE / 5);
  digitalWrite(pinhigh, HIGH);
  digitalWrite(pinlow, HIGH);
#endif
#ifdef IS_LMC
  digitalWrite(pinhigh, LOW);
  delay(DELAY_TO_BRAKE / 5);
  digitalWrite(pinhigh, HIGH);
  digitalWrite(pinlow, HIGH);
  delay(200);
  digitalWrite(pinhigh, LOW);
  digitalWrite(pinlow, LOW);
#endif
}

Move_Info check_moving(int pincheck)
{
  //checks 10 times if the pin is pressed and mode is on (true)
  uint32_t pincheck_time = 0;   //time that pincheck is pressed
  int count = 0;
  bool test = false;
  Move_Info s;
  if (MYDIGITAL_READ(pincheck))
  {
    for (int i = 0; i < 5; i++)
    {
      if (MYDIGITAL_READ(pincheck))
      {
        count++;
        //Serial.print("Count is:");
        //Serial.println(count);
        delay(DEBOUNCE_DELAY);
      }
    }
    if (count >= 5)
    {
      pincheck_time = millis();
     // Serial.print("The pincheck is pressed at :");
     // Serial.println(pincheck_time);
      test = true;
    }
  }
  s.pincheck_end = test;
  s.time_end = pincheck_time;
  return s;
}

/*function motor_do_the_move
  bool motor_do_the_move(int pinhigh,int pinlow,int pincheck)
  input int pinhigh  the pin that goes HIGH
        int pinlow   the pin that goes LOW
  int pincheck the pin for the check
  return true if all ok
         false if timeout
  for each motor there is a specific delay (del) after the pincheck is pressed
*/
bool motor_do_the_move(int pinhigh, int pinlow, int pincheck, unsigned long break_delay)
{
  if (MYDIGITAL_READ(pincheck)) return (true); //it is at the point!
  bool ok = false;
  uint32_t pincheck_pressed;
  Move_Info r;

  //start moving
  start_moving(pinhigh, pinlow);

  uint32_t time_pressed, current;
  unsigned long tm1, tm = millis();
  tm1 = tm;
  tm += TimeOut;
//  Serial.print("Motor delay:");
//  Serial.println(break_delay);
//  Serial.println(tm);
//  Serial.println(millis());
  while (!ok && tm > millis())
  {
    r = check_moving(pincheck);
//    Serial.println(r.pincheck_end);
//    Serial.println(r.time_end);

    if (r.pincheck_end == true)  //wait for the pincheck to be pressed and then delay
    {
      delay(break_delay);
      stop_moving(pinhigh, pinlow);   //if so stop the motor and set the flags
      ok = true;
      current = millis();
      //Serial.print("Motor stops at:");
      //Serial.println(current);
    }
  }
  return ok;
}

bool* two_motors_do_the_move(int pinhigh1, int pinhigh2, int pinlow1, int pinlow2, int pincheck1, int pincheck2, unsigned long break_delay1, unsigned long break_delay2, bool NS)
{

  bool ok[] = {false, false};    //motor has finish the move flag
  //bool mode1_on = true;
  //bool mode2_on = true;
  unsigned long stop1, stop2;
  //unsigned long delay_offset = 0;
  Move_Info s1, s2;
  //checking which of the 2 motors need moving
  if ((MYDIGITAL_READ(pincheck1)) && (!MYDIGITAL_READ(pincheck2)))
  {
    ok[0] = true;
    ok[1] = motor_do_the_move(pinhigh2, pinlow2, pincheck2, break_delay2);
    //return (ok);
  }
  else if ((MYDIGITAL_READ(pincheck2)) && (!MYDIGITAL_READ(pincheck1)))
  {
    ok[1] = true;
    ok[0] = motor_do_the_move(pinhigh1, pinlow1, pincheck1, break_delay1);
    //return (ok);
  }
  else if ((MYDIGITAL_READ(pincheck2)) && (MYDIGITAL_READ(pincheck1)))
  {
    ok[0] = true;
    ok[1] = true;
    //return (ok);
  }
  else
  {
    //start moving motors
    start_moving(pinhigh1, pinlow1);
    start_moving(pinhigh2, pinlow2);
    uint32_t pincheck1_pressed, pincheck2_pressed;
    uint32_t tm1, tm = millis();
    tm1 = tm;
    tm += TimeOut;
//    Serial.print("Motor1 delay:");
//    Serial.println(break_delay1);
//    Serial.print("Motor2 delay:");
//    Serial.println(break_delay2);
//    Serial.println(tm);
//    Serial.println(millis());
    while ((!ok[0]) && (!ok[1]) && (tm > millis()))
    {
      s1 = check_moving(pincheck1);
      s2 = check_moving(pincheck2);

      //Both pincheck pressed simoultaneously
      if ((s1.pincheck_end) && (s2.pincheck_end))
      {
//        Serial.print("Pincheck1 and Pincheck2 is pressed\n");
        delay(break_delay1 - (5 * DEBOUNCE_DELAY)); //minus 50ms (time that pincheck2 required)
        //stop motor1
        stop_moving(pinhigh1, pinlow1);   //if so stop the motor and set the flags
        time_for_move[3] = millis() - tm1;
        ok[0] = true;
        stop1 = millis();
//        Serial.print("Motor1 stops at:");
//        Serial.println(stop1);
        delay(break_delay2 - break_delay1);
        //stop motor2
        stop_moving(pinhigh2, pinlow2);   //if so stop the motor and set the flags
        time_for_move[4] = millis() - tm1;
        ok[1] = true;
        stop2 = millis();
//        Serial.print("Motor2 stops at:");
//        Serial.println(stop2);
      }

      //Pincheck1 is pressed first
      else if ((s1.pincheck_end) && !(s2.pincheck_end))
      {
//        Serial.print("Pincheck1 is pressed first");
        while (millis() - break_delay1 < (s1.time_end))
        {
          if (!ok[1])
          {
            s2 = check_moving(pincheck2);
            if (s2.pincheck_end)
            {
              pincheck2_pressed = s2.time_end;
//              Serial.print(" Pincheck2 here pressed\n");
              //Serial.println(pincheck2_pressed);
              //Serial.println(s2.time_end);
              ok[1] = true;
            }
          }
        }
        //stop motor1
        stop_moving(pinhigh1, pinlow1);   //if so stop the motor and set the flags
        time_for_move[3] = millis() - tm1;
        ok[0] = true;
        stop1 = millis();
//        Serial.print("Motor1 stops at:");
//        Serial.println(stop1);

        //if pinckeck2 has not pressed yet
        while (!s2.pincheck_end)
        {
          s2 = check_moving(pincheck2);
          pincheck2_pressed = s2.time_end;
        }
        stop2 = millis();
        //Serial.print("millis time is:");
        //Serial.println(stop2);
        //Serial.print("pincheck2 time is:");
        //Serial.println(pincheck2_pressed);
        while (stop2 - break_delay2 < (pincheck2_pressed))
        {
          stop2 = millis();
        }
        //motor2 stops
        stop_moving(pinhigh2, pinlow2);   //if so stop the motor and set the flags
        time_for_move[4] = millis() - tm1;
        ok[1] = true;
        stop2 = millis();
//        Serial.print("Motor2 stops at:");
//        Serial.println(stop2);
      }

      //Pincheck2 is pressed first
      else if (!(s1.pincheck_end) && (s2.pincheck_end))
      {
//        Serial.print("Pincheck2 is pressed first");
        while (millis() - break_delay2 < (s2.time_end))
        {
          if (!ok[0])
          {
            s1 = check_moving(pincheck1);
            if (s1.pincheck_end)
            {
              pincheck1_pressed = s1.time_end;
//              Serial.print(" Pincheck1 here pressed\n");
//              Serial.println(pincheck1_pressed);
              ok[0] = true;
            }
          }
        }
        //stop motor2
        stop_moving(pinhigh2, pinlow2);   //if so stop the motor and set the flags
        time_for_move[4] = millis() - tm1;
        ok[1] = true;
        stop2 = millis();
//        Serial.print("Motor2 stops at:");
//        Serial.println(stop2);

        //if pinckeck1 has not pressed yet
        while (!s1.pincheck_end)
        {
          s1 = check_moving(pincheck1);
          pincheck1_pressed = s1.time_end;
        }
        stop1 = millis();
        while (stop1 - break_delay1 < (pincheck1_pressed))
        {
          stop1 = millis();
        }
        //motor1 stops
        stop_moving(pinhigh1, pinlow1);   //if so stop the motor and set the flags
        time_for_move[3] = millis() - tm1;
        ok[0] = true;
        stop1 = millis();
//        Serial.print("Motor1 stops at:");
//        Serial.println(stop1);
      }
    }
  }
  static bool flags[] = {false, false};
  flags[0] = ok[0];
  flags[1] = ok[1];
  return flags;
}


void internal_motor_move(unsigned char motor)
{
  bool ok = true;   //motor has finish the move flag
  unsigned long break_delay;    //NA DO AN PREPEI NA EINAI INT
  //define the delay of the motor to move
  //define delay NORTH->250ms, SOUTH->380ms, EAST->310ms, WEST->320ms
  if (motor == P1_MOTOR)
  {
    //Serial.print("North1\n");
    break_delay = delayNorth;
    //Serial.println(delayNorth);
  }
  else if (motor == P2_MOTOR)
  {
    //Serial.print("East1\n");
    break_delay = delayEast;
    //Serial.println(delayEast);
  }
  else if (motor == P3_MOTOR)
  {
    //Serial.print("South1\n");
    break_delay = delaySouth;
    //Serial.println(delaySouth);
  }
  else if (motor == P4_MOTOR)
  {
    //Serial.print("West1\n");
    break_delay = delayWest;
    //Serial.println(delayWest);
  }

  if (MotorsStatus[motor] == STATUS_Not_connected)
  {
    //clear all the flags and return
    MotorsToMove[motor][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor][MOTOR_TIMEOUT] = !ok;
    return;
  }
  if (MotorsToMove[motor][MOTOR_TO_OPEN])
  {
    ok =  motor_do_the_move(MotorsM_OPEN_Pins[motor],       //set high the open
                            MotorsM_CLOSE_Pins[motor],      //set low the close
                            MotorsPOS_OPEN_Pins[motor], break_delay);   //and check the open
    MotorsToMove[motor][MOTOR_TO_OPEN] = false;
  }
  else if (MotorsToMove[motor][MOTOR_TO_CLOSE])
  {
    ok =  motor_do_the_move(MotorsM_CLOSE_Pins[motor],      //set high the close
                            MotorsM_OPEN_Pins[motor],       //set low the open
                            MotorsPOS_CLOSE_Pins[motor], break_delay);  //and check the close
    MotorsToMove[motor][MOTOR_TO_CLOSE] = false;
  }
  //Serial.print("The ok is:");
  //Serial.println(ok);
  MotorsToMove[motor][MOTOR_TIMEOUT] = !ok;
  time_for_move[motor] = time_for_move[4];
}

void internal_two_motors_move(unsigned char motor1, unsigned char motor2)
{
  bool *check;
  bool NS;
  unsigned long break_delay1, break_delay2;
  //define delay NORTH->250ms, SOUTH->380ms, EAST->310ms, WEST->320ms
  if (motor1 == P1_MOTOR)
  {
    //Serial.print("North\n");
    break_delay1 = delayNorth;
    //Serial.println(delayNorth);
  }
  else if (motor1 == P2_MOTOR)
  {
    //Serial.print("East\n");
    break_delay1 = delayEast;
    //Serial.println(delayEast);
  }

  if (motor2 == P3_MOTOR)
  {
    //Serial.print("South\n");
    break_delay2 = delaySouth;
    //Serial.println(delaySouth);
  }
  else if (motor2 == P4_MOTOR)
  {
    //Serial.print("West\n");
    break_delay2 = delayWest;
    //Serial.println(delayWest);
  }

  if ((MotorsStatus[motor1] == STATUS_Not_connected) && (MotorsStatus[motor2] != STATUS_Not_connected))
  {
    //clear all the flags and return
    internal_motor_move(motor2);
    MotorsToMove[motor1][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor1][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor1][MOTOR_TIMEOUT] = false;
    return;
  }
  else if ((MotorsStatus[motor2] == STATUS_Not_connected) && (MotorsStatus[motor1] != STATUS_Not_connected))
  {
    //clear all the flags and return
    internal_motor_move(motor1);
    MotorsToMove[motor2][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor2][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor2][MOTOR_TIMEOUT] = false;
    return;
  }
  else if ((MotorsStatus[motor2] == STATUS_Not_connected) && (MotorsStatus[motor1] == STATUS_Not_connected))
  {
    //clear all the flags and return
    MotorsToMove[motor1][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor1][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor1][MOTOR_TIMEOUT] = false;
    MotorsToMove[motor2][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor2][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor2][MOTOR_TIMEOUT] = false;
    return;
  }

  //check if the pair North-South will move, if so set the delay (NS==true)
  if (motor1 == P1_MOTOR || motor1 == P3_MOTOR)
  {
    NS = true;
    //Serial.print("NS moving\n");
  }
  else
  {
    //Serial.print("WE moving\n");
    NS = false;
  }
  //check which motor will close and which will open
  if ((MotorsToMove[motor1][MOTOR_TO_OPEN]) && (MotorsToMove[motor2][MOTOR_TO_OPEN]))
  {
    check =  two_motors_do_the_move(MotorsM_OPEN_Pins[motor1],       //set high the opens
                                    MotorsM_OPEN_Pins[motor2],
                                    MotorsM_CLOSE_Pins[motor1],      //set low the closes
                                    MotorsM_CLOSE_Pins[motor2],
                                    MotorsPOS_OPEN_Pins[motor1],    //and check the open
                                    MotorsPOS_OPEN_Pins[motor2], break_delay1, break_delay2, NS);    //and check the open
    MotorsToMove[motor1][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor2][MOTOR_TO_OPEN] = false;
  }
  else if ((MotorsToMove[motor1][MOTOR_TO_OPEN]) && (MotorsToMove[motor2][MOTOR_TO_CLOSE]))
  {
    check =  two_motors_do_the_move(MotorsM_OPEN_Pins[motor1],       //set high the opens
                                    MotorsM_CLOSE_Pins[motor2],
                                    MotorsM_CLOSE_Pins[motor1],      //set low the closes
                                    MotorsM_OPEN_Pins[motor2],
                                    MotorsPOS_OPEN_Pins[motor1],    //and check the open
                                    MotorsPOS_CLOSE_Pins[motor2], break_delay1, break_delay2, NS);    //and check the open
    MotorsToMove[motor1][MOTOR_TO_OPEN] = false;
    MotorsToMove[motor2][MOTOR_TO_CLOSE] = false;
  }
  else if ((MotorsToMove[motor2][MOTOR_TO_OPEN]) && (MotorsToMove[motor1][MOTOR_TO_CLOSE]))
  {
    check =  two_motors_do_the_move(MotorsM_CLOSE_Pins[motor1],       //set high the opens
                                    MotorsM_OPEN_Pins[motor2],
                                    MotorsM_OPEN_Pins[motor1],      //set low the closes
                                    MotorsM_CLOSE_Pins[motor2],
                                    MotorsPOS_CLOSE_Pins[motor1],    //and check the open
                                    MotorsPOS_OPEN_Pins[motor2], break_delay1, break_delay2, NS);    //and check the open
    MotorsToMove[motor1][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor2][MOTOR_TO_OPEN] = false;
  }
  else if ((MotorsToMove[motor1][MOTOR_TO_CLOSE]) && (MotorsToMove[motor2][MOTOR_TO_CLOSE]))
  {
    check =  two_motors_do_the_move(MotorsM_CLOSE_Pins[motor1],       //set high the opens
                                    MotorsM_CLOSE_Pins[motor2],
                                    MotorsM_OPEN_Pins[motor1],      //set low the closes
                                    MotorsM_OPEN_Pins[motor2],
                                    MotorsPOS_CLOSE_Pins[motor1],    //and check the open
                                    MotorsPOS_CLOSE_Pins[motor2], break_delay1, break_delay2, NS);    //and check the open
    MotorsToMove[motor1][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor2][MOTOR_TO_CLOSE] = false;
  }
  //Serial.print("The oks are:");
  //Serial.print(check[0]);
  //Serial.println(check[1]);
  MotorsToMove[motor1][MOTOR_TIMEOUT] = !check[0];
  time_for_move[motor1] = time_for_move[3];
  MotorsToMove[motor2][MOTOR_TIMEOUT] = !check[1];
  time_for_move[motor2] = time_for_move[4];
}

void p1_move()
{
  if (IsMotorPresent[P1_MOTOR]) internal_motor_move(P1_MOTOR);
}

void p2_move()
{
  if (IsMotorPresent[P2_MOTOR]) internal_motor_move(P2_MOTOR);
}

void p3_move()
{
  if (IsMotorPresent[P3_MOTOR]) internal_motor_move(P3_MOTOR);
}

void p4_move()
{
  if (IsMotorPresent[P4_MOTOR]) internal_motor_move(P4_MOTOR);
}

void move_all_motors(int seq[6], bool open[6], int use_delay)
{
  get_status();
  int i;
  for (i = 0; i < 6; i = i + 2)
  {
    MotorsToMove[seq[i]][MOTOR_TO_CLOSE] = !open[i];
    MotorsToMove[seq[i]][MOTOR_TO_OPEN] = open[i];
    MotorsToMove[seq[i + 1]][MOTOR_TO_CLOSE] = !open[i + 1];
    MotorsToMove[seq[i + 1]][MOTOR_TO_OPEN] = open[i + 1];
    internal_two_motors_move(seq[i], seq[i + 1]);
    if (use_delay > 0) delay(use_delay);
  }
}

void open_all()
{
#ifdef IS_ATC
  int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
  bool op[6] = {true, true, true, true, true, true};
#endif
#ifdef IS_LMC
  int seq[4] = {P1_MOTOR, P2_MOTOR, P3_MOTOR, P4_MOTOR};
  bool op[4] = {true, true, true, true};
#endif

  move_all_motors(seq, op);
}

void close_all()
{
#ifdef IS_ATC
  if ((MotorsStatus[0] == STATUS_Close||STATUS_Open) && (MotorsStatus[1] == STATUS_Close) && (MotorsStatus[2] == STATUS_Close||STATUS_Open) && (MotorsStatus[3] == STATUS_Close))
  {
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {false, false, false, false, false, false};
    move_all_motors(seq, op, 10);
  }
  else
  { 
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {true, true, false, false, false, false};
    move_all_motors(seq, op, 10);
  }
#endif
#ifdef IS_LMC
  int seq[4] = {P1_MOTOR, P2_MOTOR, P3_MOTOR, P4_MOTOR};
  bool op[4] = {false, false, false, false};
  move_all_motors(seq, op, 10);
#endif
}

void p1() //NORTH
{
  if ((MotorsStatus[1] == STATUS_Open) || (MotorsStatus[3] == STATUS_Open))
  {
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {true, true, false, false, true, false};
    move_all_motors(seq, op, 10);
  }
  else
  {
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {true, false, false, false, true, false};
    move_all_motors(seq, op, 10);
  }
}

void p2() //EAST
{
  int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
  bool op[6] = {true, true, true, false, false, false};
  move_all_motors(seq, op, 10);
}

void p3() //SOUTH
{
  if ((MotorsStatus[1] == STATUS_Open) || (MotorsStatus[3] == STATUS_Open))
  {
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {true, true, false, false, false, true};
    move_all_motors(seq, op, 10);
  }
  else
  {
    int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
    bool op[6] = {false, true, false, false, false, true};
    move_all_motors(seq, op, 10);
  }
}

void p4() //WEST
{
  int seq[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
  bool op[6] = {true, true, false, true, false, false};
  move_all_motors(seq, op, 10);
}

void V_input()
{
  V_temp = analogRead(V_read);
  delay(50);
  Vin = ((V_temp * 5 * 2.5) / 1023);  //(((Vcc/1023)*V_temp) * ((R1+R2)/R2)
}

void get_status()
{
  int i;
  Error &= 0b11100000;  //the first 3 bytes of the error can be used
  //for other types of error.
  unsigned char err_b;
  for (i = 0; i < MOTORS_COUNT; i++)
  {
    if (IsMotorPresent[i])
    {
      err_b = _BV(i);

      if (MYDIGITAL_READ(MotorsPOS_CLOSE_Pins[i]) && MYDIGITAL_READ(MotorsPOS_OPEN_Pins[i]))
      {
        MotorsStatus[i] = STATUS_Not_connected;
        Error = Error | err_b;
      }
      else if (!MYDIGITAL_READ(MotorsPOS_CLOSE_Pins[i]) && !MYDIGITAL_READ(MotorsPOS_OPEN_Pins[i]))
      {
        MotorsStatus[i] = STATUS_Moving;
        //    Error = Error & (0xff-err_b);
      }
      else if (MYDIGITAL_READ(MotorsPOS_OPEN_Pins[i]))
      {
        MotorsStatus[i] = STATUS_Open;
        //    Error = Error & (0xff-err_b);
      }
      else if (MYDIGITAL_READ(MotorsPOS_CLOSE_Pins[i]))
      {
        MotorsStatus[i] = STATUS_Close;
        //    Error = Error & (0xff-err_b);
      }
      if (MotorsToMove[i][MOTOR_TIMEOUT])
      {
        MotorsStatus[i] = STATUS_Timeout;
        MotorsToMove[i][MOTOR_TIMEOUT] = false;
        Error = Error | err_b;
      }
    }
    else
    {
      MotorsStatus[i] = STATUS_Not_present;
    }
  }

  V_input();
  if (Vin < V_min || Vin > V_max)
  {
    Error = Error | B10000;
  }
}

void print_status()
{
  int i;
  Previous_Error = Error;
#ifdef IS_ATC
  Serial.print("Error :");
  Serial.print(Error, BIN);
#endif
#ifdef IS_LMC
  Serial.print("S,");
#endif
  for (i = 0; i < MOTORS_COUNT; i++)
  {
    if (IsMotorPresent[i])
    {
#ifdef IS_ATC
      Serial.print(", ");
      Serial.print(MotorsNames[i]);
      Serial.print(" :");
#endif
      Serial.print(MotorsStatus[i]);
    }
  }
#ifdef IS_ATC
  Serial.println(".");
#endif
#ifdef IS_LMC
  Serial.println();
#endif
  if (service_mode == true)
  {
    for (i = 0; i < MOTORS_COUNT; i++)
    {
      Serial.print  (MotorsNames[i]);
      Serial.print  (" Time:");
      Serial.print  (time_for_move[i]);
      Serial.print  ("ms, ");
    }
    Serial.println  (". ");
    Serial.print    ("Speed:");
    Serial.print    (int(duty_cycle * 100));
    Serial.print    ("%, ");
    Serial.print    ("VREF:");
    Serial.print    (VREF);
    Serial.print    ("V, ");
    Serial.print    ("Time Out:");
    Serial.print    (TimeOut);
    Serial.print    ("ms, ");
    Serial.print    ("Vin:");
    Serial.print    (Vin);
    Serial.print    ("V, ");
    Serial.print    (" ( Test Mode )");
    Serial.println  (".");
  }
}

void ExecuteCommands()
{
  S_CMD = SerialBuffer[0];
  switch (S_CMD)
  {
    case (MOTORS_PRESENT):
      int i;
      if (SerialBufferIndex == 1)
      {
        Serial.print("A,");
        readPresentMotors();
        for (i = 0; i < 4; i++)
        {
          Serial.print(IsMotorPresent[i]);
        }
        Serial.println();
      }
      else
      {
        if (service_mode == true)
        {
          bool er = false;
          bool t[4];
          Serial.print("A,");
          for (i = 0; i < 4; i++)
          {
            if  (SerialBuffer[i + 1] == '0')
            {
              Serial.print("F");
              t[i] = false;
            }
            else  if (SerialBuffer[i + 1] == '1')
            {
              Serial.print("T");
              t[i] = true;
            }
            else
            {
              Serial.print("Err");
              er = true;
              break;
            }
          }
          Serial.println();
          if (!er)
          {
            for (i = 0; i < 4; i++) IsMotorPresent[i] = t[i];
            writePresentMotors();
          }
        }
      }
      break;

    case (VERSION):
      if (service_mode == true)
      {
        Serial.print(FWV);
        Serial.println(" ( Test Mode )");
      }
      else
      {
        Serial.println(FWV);
      }
      break;

    case (SPEED):
      if (SerialBufferIndex == 1)
      {
        Serial.print("Speed at ");
        Serial.print(int(duty_cycle * 100));
        Serial.println("%");
      }
      else
      {
        if (service_mode == true)
        {
          int New_Speed = atoi(SerialBuffer + 1);
          if ((New_Speed <= MIN_SPEED) && (New_Speed != Speed))
          {
            Speed = New_Speed;
            eeprom_write_byte((uint8_t *)&eeSpeed, Speed);
            //   EEPROM.write(0 , Speed);
            Serial.print("Speed at ");
            Serial.print(int(duty_cycle * 100));
            Serial.println("%");
            analogWrite(PWM, Speed);
          }
          else
          {
            Serial.print("Speed Range : 0-");
            Serial.println(MIN_SPEED);
          }
        }
      }
      break;

    case (TIMEOUT):
      if (SerialBufferIndex == 1)
      {
        Serial.print("Time Out=");
        Serial.println(TimeOut);
      }
      else
      {
        if (service_mode == true)
        {
          TimeOut = atoi(SerialBuffer + 1);
          eeprom_write_word((uint16_t *)&eeTimeOut, TimeOut);
          Serial.print("Time Out=");
          Serial.println(TimeOut);
        }
      }
      break;

    case (DELAYS):
      if (SerialBufferIndex == 1 && !service_mode)
      {
        Serial.print("Default Delays:\n");
        Serial.print("North Delay=");
        Serial.print(delayNorth);
        Serial.print("ms\n");
        Serial.print("East Delay=");
        Serial.print(delayEast);
        Serial.print("ms\n");
        Serial.print("South Delay=");
        Serial.print(delaySouth);
        Serial.print("ms\n");
        Serial.print("West Delay=");
        Serial.print(delayWest);
        Serial.print("ms\n");
      }
      else
      {
        Serial.print("Press DN and the desired delay to set up delay for North, and do the the same for the other sections\n");
        if (service_mode == true)
        {
          if (SerialBuffer[1] == 'N')
          {
            delayNorth = atoi(SerialBuffer + 2);
            eeprom_write_dword((uint32_t *)&eedelayNorth, delayNorth);
            Serial.print("North Delay=");
            Serial.print(delayNorth);
            Serial.print("ms\n");
          }
          else if (SerialBuffer[1] == 'E')
          {
            delayEast = atoi(SerialBuffer + 2);
            eeprom_write_dword((uint32_t *)&eedelayEast, delayEast);
            Serial.print("East Delay=");
            Serial.print(delayEast);
            Serial.print("ms\n");
          }
          else if (SerialBuffer[1] == 'S')
          {
            delaySouth = atoi(SerialBuffer + 2);
            eeprom_write_dword((uint32_t *)&eedelaySouth, delaySouth);
            Serial.print("South Delay=");
            Serial.print(delaySouth);
            Serial.print("ms\n");
          }
          else if (SerialBuffer[1] == 'W')
          {
            delayWest = atoi(SerialBuffer + 2);
            eeprom_write_dword((uint32_t *)&eedelayWest, delayWest);
            Serial.print("West Delay=");
            Serial.print(delayWest);
            Serial.print("ms\n");
          }
          else
          {
            Serial.print("North Delay=");
            Serial.print(delayNorth);
            Serial.print("ms\n");
            Serial.print("East Delay=");
            Serial.print(delayEast);
            Serial.print("ms\n");
            Serial.print("South Delay=");
            Serial.print(delaySouth);
            Serial.print("ms\n");
            Serial.print("West Delay=");
            Serial.print(delayWest);
            Serial.print("ms\n");
          }
        }
      }
      break;

    case (SERVICE_MODE):
      if (service_mode == true)
      {
        service_mode = false;
        Serial.println("Normal Operation");
      }
      else
      {
        service_mode = true;
        Serial.println("Test Mode");
      }
      break;

    case (MOVE):
      if (SerialBuffer[1] == ',')
      {
        int seq2[6] = {P1_MOTOR, P3_MOTOR, P2_MOTOR, P4_MOTOR, P1_MOTOR, P3_MOTOR};
        if (SerialBuffer[2] == '0' && SerialBuffer[3] == '0' && SerialBuffer[4] == '0' && SerialBuffer[5] == '0')
        {
          close_all();
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '0' && SerialBuffer[4] == '0' && SerialBuffer[5] == '1')
        {
          g_functions[3]();
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '0' && SerialBuffer[4] == '1' && SerialBuffer[5] == '0')
        {
          g_functions[2]();
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '0' && SerialBuffer[4] == '1' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, false, true, false, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '1' && SerialBuffer[4] == '0' && SerialBuffer[5] == '0')
        {
          g_functions[1]();
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '1' && SerialBuffer[4] == '0' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, true, true, false, false};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '1' && SerialBuffer[4] == '1' && SerialBuffer[5] == '0')
        {
          bool op2[6] = {true, true, true, false, false, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '0' && SerialBuffer[3] == '1' && SerialBuffer[4] == '1' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, true, true, false, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '0' && SerialBuffer[4] == '0' && SerialBuffer[5] == '0')
        {
          g_functions[0]();
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '0' && SerialBuffer[4] == '0' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, false, true, true, false};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '0' && SerialBuffer[4] == '1' && SerialBuffer[5] == '0')
        {
          bool op2[6] = {true, true, false, false, true, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '0' && SerialBuffer[4] == '1' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, false, true, true, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '1' && SerialBuffer[4] == '0' && SerialBuffer[5] == '0')
        {
          bool op2[6] = {true, true, true, false, true, false};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '1' && SerialBuffer[4] == '0' && SerialBuffer[5] == '1')
        {
          bool op2[6] = {true, true, true, true, true, false};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '1' && SerialBuffer[4] == '1' && SerialBuffer[5] == '0')
        {
          bool op2[6] = {true, true, true, false, true, true};
          move_all_motors(seq2, op2, 10);
        }
        else if (SerialBuffer[2] == '1' && SerialBuffer[3] == '1' && SerialBuffer[4] == '1' && SerialBuffer[5] == '1')
        {
          open_all();
        }
      }
      get_status();
      print_status();
      break;

    case (DEMO):
      if (service_mode == true)
      {
        while (service_mode == true)
        {
          S_CMD = Serial.read();
          if (S_CMD == 'R')
          {
            service_mode = false;
          }
          open_all();
          get_status();
          print_status();
          delay(1000);
          close_all();
          get_status();
          print_status();
          delay(1000);
          g_functions[0]();
          get_status();
          print_status();
          delay(1000);
          g_functions[1]();
          get_status();
          print_status();
          delay(1000);
          g_functions[2]();
          get_status();
          print_status();
          delay(1000);
          g_functions[3]();
          get_status();
          print_status();
          delay(1000);
        }
      }
      break;

    case (MOVE_P1):
      do_cmd_motor(P1_MOTOR);
      break;

    case (MOVE_P2):
      do_cmd_motor(P2_MOTOR);
      break;

    case (MOVE_P3):
      do_cmd_motor(P3_MOTOR);
      break;

    case (MOVE_P4):
      do_cmd_motor(P4_MOTOR);
      break;

    case (OPEN_ALL):
      open_all();
      get_status();
      print_status();
      break;

    case (CLOSE_ALL):
      close_all();
      get_status();
      print_status();
      break;

    case (GET_STATUS):
      get_status();
      print_status();
      break;
  }
}

int do_cmd_motor(int motor)
{
#ifdef IS_ATC
  if (service_mode == false)
  {
    g_functions[motor]();
    get_status();
    print_status();
  }
  else
  {
    get_status();
    if (MotorsStatus[motor] != STATUS_Not_present )
    {
      MotorsToMove[motor][MOTOR_TO_CLOSE] = false;
      MotorsToMove[motor][MOTOR_TO_OPEN] = false;

      if (MYDIGITAL_READ(MotorsPOS_OPEN_Pins[motor]))
      {
        MotorsToMove[motor][MOTOR_TO_CLOSE] = true;
      }
      else
      {
        MotorsToMove[motor][MOTOR_TO_OPEN] = true;
      }
      internal_motor_move(motor);
      get_status();
      print_status();
    }
    else
    {
      print_status();
    }
  }
#endif

#ifdef IS_LMC
  get_status();
  if (MotorsStatus[motor] != STATUS_Not_present )
  {
    MotorsToMove[motor][MOTOR_TO_CLOSE] = false;
    MotorsToMove[motor][MOTOR_TO_OPEN] = false;

    if (MYDIGITAL_READ(MotorsPOS_OPEN_Pins[motor]))
    {
      MotorsToMove[motor][MOTOR_TO_CLOSE] = true;
    }
    else
    {
      MotorsToMove[motor][MOTOR_TO_OPEN] = true;
    }
    internal_motor_move(motor);
    get_status();
    print_status();
  }
#endif
}

int motor_id_from_char(char m)
{
  int i;
  for (i = 0; i < MOTORS_COUNT; i++)
  {
    if (MotorsNames[i][0] == m) return (i);
  }
  return (-1);
}

void readPresentMotors()
{
  unsigned char t[4];
  int i;
  eeprom_read_block((void *)&t[0], (uint16_t *) &eeIsMotorPresent, sizeof(IsMotorPresent));
  unsigned char CRC = 0;
  for (i = 0; i < MOTORS_COUNT; i++) CRC += t[i];
  if (CRC > 4)
  {
    for (i = 0; i < MOTORS_COUNT; i++) IsMotorPresent[i] = defaultIsMotorPresent[i];
  }
  else
  {
    for (i = 0; i < MOTORS_COUNT; i++) IsMotorPresent[i] = t[i];
  }
}

void writePresentMotors()
{
  eeprom_write_block((void *)&IsMotorPresent[0], (uint16_t *) &eeIsMotorPresent, sizeof(IsMotorPresent));
}
