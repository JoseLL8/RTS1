// --------------------------------------
// Include files
// --------------------------------------
#include <string.h>
#include <stdio.h>

// --------------------------------------
// Global Constants
// --------------------------------------
#define MESSAGE_SIZE 9
#define GAS_PIN 2
#define BRAKE_PIN 3
#define MIXER_PIN 4
#define SLOPEUP_PIN 9
#define SLOPEDOWN_PIN 8
#define SPEED_PIN 5

// --------------------------------------
// Global Variables
// --------------------------------------

/*** Planner ***/
int PC = 6; //number of secondary cycles per primary cycle
int SC = 0; //current secondary cycle. starts at 0
int f = 10; //duration of secondary cycle in ms
int time1 = 0; //time-tracking variable
int time2 = 0; //time-tracking variable
int flag = 0;

/*** Cart Control ***/
double speed = 55.5;
int slope = 0;
int gas = 0;
int brake = 0;
int mixer = 0;

bool request_received = false;
bool requested_answered = false;
char request[MESSAGE_SIZE+1];
char answer[MESSAGE_SIZE+1];

// --------------------------------------
// Function: comm_server
// --------------------------------------
int comm_server()
{
   static int count = 0;
   char car_aux;

   // If there were a received msg, send the processed answer or ERROR if none.
   // then reset for the next request.
   // NOTE: this requires that between two calls of com_server all possible 
   //       answers have been processed.
   if (request_received) {
      // if there is an answer send it, else error
      if (requested_answered) {
          Serial.print(answer);
      } else {
          Serial.print("MSG: ERR\n");
      }  
      // reset flags and buffers
      request_received = false;
      requested_answered = false;
      memset(request,'\0', MESSAGE_SIZE+1);
      memset(answer,'\0', MESSAGE_SIZE+1);
   }

   while (Serial.available()) {
      // read one character
      car_aux =Serial.read();
        
      //skip if it is not a valid character
      if  ( ( (car_aux < 'A') || (car_aux > 'Z') ) &&
           (car_aux != ':') && (car_aux != ' ') && (car_aux != '\n') ) {
         continue;
      }
      
      //Store the character
      request[count] = car_aux;
      
      // If the last character is an enter or
      // There are 9th characters set an enter and finish.
      if ( (request[count] == '\n') || (count == 8) ) {
         request[count] = '\n';
         count = 0;
         request_received = true;
         break;
      }

      // Increment the count
      count++;
   }
  speed_req();
  slope_req();
  gas_req();
  brake_req();
  mixer_req();
}


int speed_task() 
{
  speed -= (0.25*slope - 0.5*gas + 0.5*brake)*PC*f/1000;
  speed = constrain(speed,0,100);
  //Serial.println(speed);

  int result = map(speed,0,70,0,100);
  analogWrite(SPEED_PIN, constrain(result,0,100));
  return 0;
}

// --------------------------------------
// Function: speed_req
// --------------------------------------
int speed_req()
{
   // If there is a request not answered, check if this is the one
   if ( (request_received) && (!requested_answered) &&
        (0 == strcmp("SPD: REQ\n",request)) ) {
          
      // send the answer for speed request
      char num_str[5];
      dtostrf(speed,4,1,num_str);
      sprintf(answer,"SPD:%s\n",num_str);

      // set request as answered
      requested_answered = true;
   }
   return 0;
}

int slope_task() 
{
  if (digitalRead(SLOPEUP_PIN)==HIGH)
    {slope=1;}
  else if (digitalRead(SLOPEDOWN_PIN)==HIGH)
    {slope=-1;}
  else {slope=0;}
  return 0;
}
// --------------------------------------
// Function: slope_req
// --------------------------------------
int slope_req()
{
   // If there is a request not answered, check if this is the one
   if ( (request_received) && (!requested_answered) &&
        (0 == strcmp("SLP: REQ\n",request)) ) {
      
     if (slope==1)
     {sprintf(answer,"SLP:  UP\n");}
     else if (slope==-1)
     {sprintf(answer,"SLP:DOWN\n");}
     else {sprintf(answer,"SLP:FLAT\n");}
      // send the answer for slope request

      // set request as answered
      requested_answered = true;
   }
   return 0;
}

int gas_task() 
{
  digitalWrite(GAS_PIN, gas);
  return 0;
}
// --------------------------------------
// Function: gas_req
// --------------------------------------
int gas_req()
{
   // If there is a request not answered, check if this is the one
   if ( (request_received) && (!requested_answered) ) {
      if (0 == strcmp("GAS: SET\n",request)) {
        gas = 1;
        sprintf(answer,"GAS:  OK\n");
        requested_answered = true;
      }
      else if (0 == strcmp("GAS: CLR\n",request)) {
        gas = 0;
        sprintf(answer,"GAS:  OK\n");
        requested_answered = true;
      }
    }
   return 0;
}

int brake_task()
{
  digitalWrite(BRAKE_PIN, brake);
  return 0;
}
// --------------------------------------
// Function: brake_req
// --------------------------------------
int brake_req()
{
   // If there is a request not answered, check if this is the one
   if ( (request_received) && (!requested_answered) ) {
      if (0 == strcmp("BRK: SET\n",request)) {
        brake = 1;
        sprintf(answer,"BRK:  OK\n");
        requested_answered = true;
      }
      else if (0 == strcmp("BRK: CLR\n",request)) {
        brake = 0;
        sprintf(answer,"BRK:  OK\n");
        requested_answered = true;
      }
    }
   return 0;
}

int mixer_task()
{
  digitalWrite(MIXER_PIN, mixer);
  return 0;
}
// --------------------------------------
// Function: mixer_req
// --------------------------------------
int mixer_req()
{
   // If there is a request not answered, check if this is the one
   if ( (request_received) && (!requested_answered) ) {
      if (0 == strcmp("MIX: SET\n",request)) {
        mixer = 1;
        sprintf(answer,"MIX:  OK\n");
        requested_answered = true;
      }
      else if (0 == strcmp("MIX: CLR\n",request)) {
        mixer = 0;
        sprintf(answer,"MIX:  OK\n");
        requested_answered = true;
      }
    }
   return 0;
}

// --------------------------------------
// Function: setup
// --------------------------------------
void setup()
{
  pinMode(GAS_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(MIXER_PIN, OUTPUT);
  pinMode(SLOPEUP_PIN, INPUT);
  pinMode(SLOPEDOWN_PIN, INPUT);
  pinMode(SPEED_PIN, OUTPUT);

  //Timer 2 (timer for ADC)
  //interrupt period = (1/16MHz) * 1023 * 156 = 0.00997425 s (100.258Hz)
  TCCR2B = B00000101;     //Reset entire TCCR2B to 0 and prescaler of 1023
  TIMSK2 &= B11111000;    //Disables the interrupts from timer2
  OCR2B = 156;            //Set compare register A to this value
  TCNT2 = 0;              //Set the timer2 count to 0
  // Setup Serial Monitor
  Serial.begin(9600);
  TIMSK2 |= B00000100; //enable compB interrupts from timer2
}

// --------------------------------------
// Function: loop
// --------------------------------------
void loop()
{
  if (flag) {
  flag = 0;
  if ((time2-time1)>f) {
  //Critical error, the system was too slow. Kernel panic or something and then exit
  //exit(1);
  }

  time1 = time2;
  switch(SC) {
    case 0:
      comm_server();
      break;
    case 1:
      slope_task();
      break;
    case 2:
      speed_task();
      break;
    case 3:
      gas_task();
      break;
    case 4:
      brake_task();
      break;
    case 5:
      mixer_task();
      break;
    }
  SC = (SC+1) % PC;
  time2 = millis();
  }
}


ISR(TIMER2_COMPB_vect) {
  flag = 1;
}
