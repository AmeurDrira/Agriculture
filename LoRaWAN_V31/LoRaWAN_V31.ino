


/******************************************************************************************
  INCLUDE FILES
*****************************************************************************************
*/
#include <SPI.h>
#include <JeeLib.h> 
#include "AES-128_V10.h"
#include "Encrypt_V31.h"
#include "LoRaWAN_V31.h"
#include "RFM95_V21.h"
#include "LoRaMAC_V11.h"
#include "Waitloop_V11.h"
#include "avr/sleep.h"


/*
*****************************************************************************************
  GLOBAL VARIABLES
*****************************************************************************************
*/
unsigned char AppSkey[16] = {
  0x15, 0x33, 0xB6, 0x9B, 0x0E, 0xFD, 0xCD, 0x49, 0x83, 0x91, 0xC6, 0xF9, 0xEA, 0x0A, 0x35, 0x15
};

unsigned char NwkSkey[16] = {
  0x24, 0xE9, 0x5E, 0xD1, 0xF9, 0xEC, 0x97, 0xAE, 0x08, 0x2F, 0x0A, 0x21, 0xE9, 0x0B, 0x73, 0xBA
};

unsigned char DevAddr[4] = {
  0x02, 0x01, 0x05, 0x04
};

unsigned char Test = 0x00;
unsigned char i;
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
boolean inTrasmissionMode = false;
unsigned char Sleep_Sec = 0x00;
unsigned char Sleep_Time = 0x01;
unsigned char Data_Tx[256];
unsigned char Data_Rx[64];
unsigned char Data_Length_Tx;
unsigned char Anc_Length_Tx;
unsigned char Data_Length_Rx = 0x00;
unsigned int Frame_Counter_Up = 0x0000;
float amplitude_current;               //amplitude current
float effective_value;       //effective current

int wakePin = 2;                 // pin used for waking up
int sleepStatus = 0;             // variable to store a request for sleep
volatile int f_wdt = 0;
//boolean o1 = false;
boolean s_now = false;

//ISR(WDT_vect) { Sleepy::watchdogEvent(); }
/*********************************************/
ISR(WDT_vect)
{
  if (f_wdt < 2)
  {
    f_wdt++;
  }
  else
  {
    //Serial.println("WDT Overrun!!!");
  }
}
/********************************************/
void sendData()
{
  //TX base adress
  RFM_Write(0x0E, 0x40);
  int sensor1 = analogRead(A0);
  int sensor2 = analogRead(A1);
  int WaterLevel = analogRead(A2);
  String frameHeader = "{";
  String frameSensor1 = "\"moisture1\":";
  String comma = ",";
  String frameSensor2 = "\"moisture2\":";
  String frameWaterLevel = "\"waterLevel\":";
  String frameTailer = "}";
  String toSend = frameHeader + frameSensor1 + sensor1 + comma + frameSensor2 + sensor2 + comma + frameWaterLevel + WaterLevel +frameTailer;
int TempNumOne= toSend.length();
for (int a=0;a<=TempNumOne;a++)
        {
            Data_Tx[a]=toSend[a];
        }

  Data_Length_Tx = (unsigned char)TempNumOne;

  LORA_Send_Data(Data_Tx, Data_Length_Tx, Frame_Counter_Up);

  //Raise frame counter
  Frame_Counter_Up++;
}
/********************************************************************************/

/*******************************************************************************/
void wakeUpNow()        // here the interrupt is handled after wakeup
{
  /*sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...
    detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                            // wakeUpNow code will not be executed
                            // during normal running time.*/

  Data_Length_Rx = LORA_Receive_Data(Data_Rx);
  if (Data_Length_Rx > 0) {
    /*if (Data_Rx[0] == 'S' && Data_Rx[1] == 'S' &&  Data_Rx[3] == 'l' && Data_Rx[4] == 'a' && Data_Rx[5] == 'm' && Data_Rx[6] == 'p'
        && Data_Rx[7] == '1' && Data_Rx[9] == 'O' && Data_Rx[10] == 'N') {
      digitalWrite(8, HIGH);
    } else if (Data_Rx[0] == 'S' && Data_Rx[1] == 'S' &&  Data_Rx[3] == 'l' && Data_Rx[4] == 'a' && Data_Rx[5] == 'm' && Data_Rx[6] == 'p'
               && Data_Rx[7] == '1' && Data_Rx[9] == 'O' && Data_Rx[10] == 'F') {
      digitalWrite(8, LOW);
    }/*
    /*************************/
    digitalWrite(9,HIGH);
    delay(3000);
    digitalWrite(9, LOW);
    
     if (Data_Rx[0] == '1' && Data_Rx[1] == 'O') {
      digitalWrite(8, LOW);
      s_now = true;
    } else if (Data_Rx[0] == '1' && Data_Rx[1] == 'F') {
      digitalWrite(8, HIGH);
      s_now = true;
    } 
    else if (Data_Rx[0] == '2' && Data_Rx[1] == 'O') {
      digitalWrite(7, LOW);
      s_now = true;
    } else if (Data_Rx[0] == '2' && Data_Rx[1] == 'F') {
      digitalWrite(7, HIGH);
      s_now = true;
    } 


    
    /*int i = 0;
    while ( i < Data_Length_Rx) {
      if (Data_Rx[i] == DevAddr[3]) {
        if (Data_Rx[i + 3] == ' ') {

          //--------------actuators1 ON-------------//
          if (Data_Rx[i + 1] == 0x01) {
            if (Data_Rx[i + 2] == 'O') {
              digitalWrite(A1, HIGH);
            }
          

          //--------------actuators1 OFF-------------//
          
            else if (Data_Rx[i + 2] == 'F') {
              digitalWrite(A1, LOW);
            }
          }

          //--------------actuators2 ON-------------//
          else if (Data_Rx[i + 1] == 0x02) {
            if (Data_Rx[i + 2] == 'O') {
              digitalWrite(A2, HIGH);
            }
          

          //--------------actuators2 OFF-------------//
          
            if (Data_Rx[i + 2] == 'F') {
              digitalWrite(A2, LOW);
            }
          }
        }


        //else if (Data_Rx[i+3] != ' '){
        //--------------adjusting servo-------------//
        // myservo.write(90);
        //}
      }
      // pass to next command
      i = i + 4;
    }*/

    Serial.print("DATA recieved: ");

    String r = String((char*)Data_Rx);
    Serial.println(r);
    memset(Data_Rx, 0, Data_Length_Rx);
    /*digitalWrite(9, HIGH);
      delay(3000);
      digitalWrite(9, LOW);/*
      /*
      Serial.print("DATA recieved: ");
      for (i = 0; i < Data_Length_Rx; i++)
      {
      Serial.print(Data_Rx[i]);
      }
      Serial.println();
      f_wdt = 0;*/

  }

}
/************************************************
  Function: Announce
  description: The arduino announce its devices.
************************************************/
void Announce() {
  RFM_Write(0x0E, 0x40);
  //unsigned char Anc_Tx[264];
  //Concstruct data
  //String Anc_str ="A:lamp1:1:0-24:O=ON:F=OFF;A:lamp2:2:25-49:O=ON:F=OFF;S:sensor1:3:50-x:0:1100;";
  //Anc_str.toCharArray(Anc_Tx, 264);
  unsigned char Anc_Tx[264] = {'A', ':', 'l', 'a', 'm', 'p', '1', ':', '1', ':', '0', '-', '2', '5', ':', 'O', '=', 'O', 'N', ':', 'F', '=', 'O', 'F', 'F', ';',
                               'A', ':', 'l', 'a', 'm', 'p', '2', ':', '2', ';', '2', '6', '-', '5', '2', ':', 'O', '=', 'O', 'N', ':', 'F', '=', 'O', 'F', 'F', ';',
                               'S', ':', 's', 'e', 'n', 's', 'o', 'r', '1', ':', '3', ':', '5', '3', '-', '7', '7', ':', '0', ':', '2', '0', '0', '0', ';'
                              };


  /*Anc_Tx[0] = 'L';
    Anc_Tx[1] = '1';
    Anc_Tx[2] = ' ';
    Anc_Tx[3] = 'O';
    Anc_Tx[4] = ' ';
    Anc_Tx[5] = 'F';
    Anc_Tx[6] = ';';
    Anc_Tx[7] = 'L';
    Anc_Tx[8] = '2';
    Anc_Tx[9] = ' ';
    Anc_Tx[10] = 'O';
    Anc_Tx[11] = ' ';
    Anc_Tx[12] = 'F';
    Anc_Tx[13] = ';';
    Anc_Tx[14] = 'R';
    Anc_Tx[15] = '3';
    Anc_Tx[16] = ';';
    /*Anc_Tx[17] = 'S';
    Anc_Tx[17] = '4';
    Anc_Tx[17] = ' ';
    Anc_Tx[17] = '0';
    Anc_Tx[17] = ' ';
    Anc_Tx[17] = '';*/

  Anc_Length_Tx = 0x4D;

  LORA_announce(Anc_Tx, Anc_Length_Tx, Frame_Counter_Up);

  //Raise frame counter
  Frame_Counter_Up++;
}
/***********************************************/

void setup()
{
  pinMode(8, OUTPUT);
   digitalWrite(8,HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7,HIGH);
 
  //digitalWrite(8, LOW);
  //Initialize the UART
  Serial.begin(9600);

  //Initialise the SPI port
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  //Initialize I/O pins
  // pinMode(DS2401, OUTPUT);
  //pinMode(9, OUTPUT);
  pinMode(DIO0, INPUT);
  pinMode(DIO5, INPUT);
  pinMode(LED, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  WaitLoop_Init();
  //intialisation relai


  //Wait until RFM module is started
  WaitLoop(20);
  //pciSetup(14);
  RFM_Init();
  WaitLoop(100);
  /*//TX base adress
      RFM_Write(0x0E, 0x80);
      //Concstruct data
      Data_Tx[0] = Test++;
      Data_Length_Tx = 0x01;

      LORA_Send_Data(Data_Tx, Data_Length_Tx, Frame_Counter_Up);

      //Raise frame counter
      Frame_Counter_Up++;
      digitalWrite(9, HIGH);
      delay(1000);
      digitalWrite(9, LOW);*/

  //attachInterrupt(1, sendData, RISING);
  /*** Setup the WDT ***/

  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);

  /* In order to change WDE or the prescaler, we need to
     set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);




sendData();
  //attachInterrupt(0, wakeUpNow, RISING); // use interrupt 0 (pin 2) and run function
                                      // wakeUpNow when pin 2 gets LOW*/
}

void sleepNow()         // here we put the arduino to sleep
{

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
     accidentally pushed interrupt button doesn't interrupt
     our running program. if you want to be able to run
     interrupt code besides the sleep function, place it in
     setup() for example.

     In the function call attachInterrupt(A, B, C)
     A   can be either 0 or 1 for interrupts on pin 2 or 3.

     B   Name of a function you want to execute at interrupt for A.

     C   Trigger mode of the interrupt pin. can be:
                 LOW        a low level triggers
                 CHANGE     a change in level triggers
                 RISING     a rising edge of a level triggers
                 FALLING    a falling edge of a level triggers

     In all but the IDLE sleep modes only LOW can be used.*/


  attachInterrupt(0, wakeUpNow, RISING); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}

/*int getMaxValue()
{
  int sensorValue = 0;
  long cpt = 0;
  //value read from the sensor
  int sensorMax = 0;
  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) //sample for 1000ms
  {
    //cpt++;
    sensorValue = analogRead(ELECTRICITY_SENSOR);
    if (sensorValue > sensorMax)
    {

      sensorMax = sensorValue;
    }
    //sensorMax = sensorMax + sensorValue;
    //WaitLoop(100);
  }

  return sensorMax;

}*/

void loop()
{
  //inTrasmissionMode = false;
  /*if (analogRead(A0) > 800) { o1 = true;}
  if (analogRead(A1) > 800) { o2 = true;}*/
  //if (f_wdt == 2 )
  //sendData();
  if ((f_wdt == 2) || (s_now == true))
  {
    sendData();
    f_wdt = 0 ;

    s_now = false;
  }
  /*if (o1 == true) {
      digitalWrite(8, HIGH);
      delay(20000);
      digitalWrite(8, LOW);

    }
    
  if (o1 == true) {
      digitalWrite(7, HIGH);
      delay(20000);
      digitalWrite(7, LOW);


    }*/

    /* Toggle the LED */
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    /* Don't forget to clear the flag. */
    //f_wdt = 0 ;

    //s_now = false;
    //o2 = false;
    /* Re-enter sleep mode. */
  
  delay (200);
  RFM_Init();
  //Set carrier freqeuncy
  //869.525 / 61.035 = 14246334 = 0xD961BE
  RFM_Write(0x06, 0xD9);
  RFM_Write(0x07, 0x61);
  RFM_Write(0x08, 0xBE);

  //Change DIO 0 to RxDone
  RFM_Write(0x40, 0x00);

  //Switch to SF 15 (changed from 9: 0x94) payload on CRC on
  RFM_Write(0x1E, 0xC4);

  //Invert IQ
  RFM_Write(0x33, 0x67);
  RFM_Write(0x3B, 0x19);

  //Switch RFM to Continuous reception
  RFM_Write(0x01, 0x85);
  //Switch RFM to Single reception
  //RFM_Write(0x01,0x86);

  //Wait on mode ready
  while (digitalRead(DIO5) == LOW)
  {
  }
  sleepNow();
  //attachInterrupt(0, wakeUpNow, RISING); 
  //Sleepy::loseSomeTime(15000);    // sleep function called here

}
