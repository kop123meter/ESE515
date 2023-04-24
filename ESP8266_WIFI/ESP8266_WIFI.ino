/***************************************************************************
 * For ESE515 Spring 2022
 ***************************************************************************/
 
/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.

/************************ Program Starts Here *******************************/

// PIR Sensor_Lab4

#include "config.h"
#include "SoftwareSerial.h" // use software serial to communicate with uno R3

// communcation macro
#define SENSOR_DATA 1      //Communication flag 
#define SAVE_CARD 2
#define TX_PIN D0
#define RX_PIN D1
#define FRAME_LENGETH 8
#define END_CODE 5      //END reading flag
#define PWD_LENGETH 6
int sensor_data[FRAME_LENGETH]; // FRAME receive buffer 
int control_data[FRAME_LENGETH]; //FRAME send buffer
int receive_pwd[PWD_LENGETH];
int pwd_index = 0;
int check_flag = 0;
int pwd_current_ol[PWD_LENGETH];
int save_flag = 0;
const char * LENERROR = "LEN ERROR";
const char * SAVE ="SAVED!";
int readbuffer[FRAME_LENGETH];

//SoftwareSerial setup
SoftwareSerial WIFImoduleSerial(RX_PIN,TX_PIN);

AdafruitIO_Feed *light  = io.feed("light");
AdafruitIO_Feed *temperatureDHT = io.feed("temperatureDHT");
AdafruitIO_Feed *humidityDHT = io.feed("humidityDHT");
AdafruitIO_Feed *pwdDoor = io.feed("pwdDoor");
AdafruitIO_Feed *pwdCurrent = io.feed("pwdCurrent");

float temperature = 0;
float humidity = 0;
float light_value = 0;
int PIR_distance = 0;

void send_dht_Sensor()
{
   temperatureDHT->save(temperature);
   humidityDHT->save(humidity);
}
void send_light(){
  light->save(light_value);
}


void read_sensor_data(){
  
  for(int i = 1;i<FRAME_LENGETH;i++){
    int flag = WIFImoduleSerial.read();
    sensor_data[i] = flag;
  }
  temperature = sensor_data[1]+(float)(sensor_data[2]%100);
  float temp = (float)sensor_data[2]/100;
  temperature = sensor_data[1] + temp;
  temp = (float)sensor_data[4]/100;
  humidity = sensor_data[3]+temp;
  light_value = sensor_data[5];
  PIR_distance = sensor_data[6];
  //Serial.println(sensor_data[7]);
}

/*
Name: receive_data()
Function: receive the data from the lower MCU

*/
void receive_data(){
  while(1){
  if(WIFImoduleSerial.available()>0){
    int communication_type = WIFImoduleSerial.read();
    Serial.println(communication_type);
    if(communication_type == SENSOR_DATA ){
      read_sensor_data();
      communication_type = 0;
    }
  }
  if(sensor_data[7] == 5)
  break;
  Serial.println("reading...........");
  }
  return;
}

 

void setup() {

   
  // start the serial connection
  Serial.begin(9600);
  // start the communication with UNO R3
  WIFImoduleSerial.begin(9600);
  

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();


  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  

}

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();
  receive_data();
  send_dht_Sensor();
  delay(3000);
  send_light();
  delay(3000);
 // receive_data();
  //send_dht_Sensor();
  //send_light();

  // send soil moisture data
  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events. 

}

