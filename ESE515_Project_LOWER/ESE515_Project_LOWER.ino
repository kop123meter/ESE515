/**

SmartBox system ---- Lower MCU

In this part, MCU can collect temprature, humidity, light value and door status. 
With using the communication portcal module, it can T/R data with main MCU.

Connection:

1.RFID
SDA------------------------Digital 10
SCK------------------------Digital 13
MOSI----------------------Digital 11
MISO----------------------Digital 12
IRQ------------------------DO NOT CONNECT
GND-----------------------GND
RST------------------------Digital 9                    
3.3V------------------------3.3V 

2.PIR
OUTPUT-------------------- 4

3.DHT 11
SIG----------------------2

4.CdS
OUTPUT---------------- A0

5.TX RX(SoftwareSerial)

TX-------------------5
RX------------------6

Note: Upper MCU(coonection should be different)
TX--------------------3
RX--------------------2


6.lcd screen
rs-------------------A1 15
en-------------------A2 16
d4-------------------A3 17
d5-------------------A4 18
d6-------------------A5 19
d7-------------------0 


Author:Ze Li
Date:04/3/2023

*/




#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <EEPROM.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// RFID ERROR Define
#define CARD_TYPE_ERROE -1
#define CARD_NOT_VAILD -2
#define CARD_SUCCESS 1


// Communication define
#define SENSOR_DATA_LENGETH 8


#define END_CODE 5


// PIN Define

#define RFID_RESET_PIN 9 // RFID_receiver reset pin
#define RFID_SDA_PIN 10 // RFID SDA pin
#define MOTOR_PIN 3 // SG90 control pin
#define DHT_PIN 2 // DHT
#define PIR_PIN 4 // PIR
#define LIGHT_PIN A0 // CdS
#define TX_PIN 6
#define RX_PIN 7



// MAX_Number define
#define MAX_CARD_NUMBER 512
#define UID_LENGETH 4

//Defult Thershold value define
#define MAX_TEMP 30



// Initialize Object

MFRC522 rfid(RFID_SDA_PIN,RFID_RESET_PIN);
//Servo door_servo;
int uid_write_address = 0; // uid can be saved in this address of EEPROM
int uid_read_address = 0;
byte nuidPICC[4];
byte adamin[4];

DHT dht( DHT_PIN, DHT11);
float temprature = 0;
float humidity = 0;

int PIR_distance = 0;

int light_value = 0;

int door_status = 0; // 0:closed;1:opened

LiquidCrystal lcd(A2, A1, A3, A4, A5, 5);
byte temperturechar[8] = {
  0b10000,
  0b00110,
  0b01001,
  0b01000,
  0b01000,
  0b01001,
  0b00110,
  0b00000,
}; // characte for degree

SoftwareSerial lowermcuSerial(RX_PIN,TX_PIN); 
int sensor_data[SENSOR_DATA_LENGETH];


int page_flag = 0; //0:Hum and temp; 1:light


bool check_setup(){
  // find card
  
  if (!rfid.PICC_IsNewCardPresent()){
     //Serial.println("can not read caasasasasrd..");
    return false;}
    
  // readable?
  if (!rfid.PICC_ReadCardSerial()){
    Serial.println("can not read card..");
    return false;}
  
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
        piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
        piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
        Serial.println("The card type is wrong!");
        return false; }
  
  return true;
}

int check_id(){
  if(check_setup()){
  // First save the reading content into nuidPICC array
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
    //Serial.println(nuidPICC[i],HEX);
    }  
    // stop reading the same card
    rfid.PICC_HaltA(); 
   rfid.PCD_StopCrypto1();
   // Begin check id based on the EEPROM
    uid_read_address = 0;
    while(uid_read_address != 512){
      int temp_address = uid_read_address;
      byte value_1 = EEPROM.read(temp_address);
      byte value_2 = EEPROM.read(temp_address+1);
      byte value_3 = EEPROM.read(temp_address+2);
      byte value_4 = EEPROM.read(temp_address+3);
      if (nuidPICC[0]==value_1&&nuidPICC[1]==value_2&&nuidPICC[2]==value_3&&nuidPICC[3]==value_4)
      { return CARD_SUCCESS; }
      uid_read_address = uid_read_address + 4;
    }
    Serial.println("Can't find the card!");
    return CARD_NOT_VAILD;

  }
  //Serial.println("Check Card type!!!!");
  //lcd.clear();
  //lcd.println("check your card!");
  //delay(4000);
  return CARD_TYPE_ERROE;
}
 
bool save_uid(){
  if(uid_write_address == 512 ){
    Serial.print("Card is enough!!!!,this activity may replace the first card");
    uid_write_address = 0;
    return false;
  }
  else{
    for(int i=0;i<4;i++){
      EEPROM.write(uid_write_address,nuidPICC[i]);
      uid_write_address = uid_write_address + 1;
    }
    Serial.println("Saved!!!");
    return true;
  }

}

void rfid_check(int id)
{
  if( id == CARD_NOT_VAILD){
    Serial.println("Do you want to save this card?(Y/N)");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Save this card?");
    lcd.setCursor(0, 1);
    lcd.print("Scan adamin");
   while(1) // Use adamin card to save other users card
    {  lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Save this card?");
      lcd.setCursor(0, 1);
       lcd.print("Scan Adamin..");
      if(check_setup()){
        for (byte i = 0; i < 4; i++) {
        adamin[i] = rfid.uid.uidByte[i];}
        rfid.PICC_HaltA();  
        rfid.PCD_StopCrypto1();
        if (adamin[0]==0x03&&adamin[1]==0x36&&adamin[2]==0x44&&adamin[3]==0x1B){
          lcd.clear();
          lcd.print("Welcomeback....");
          if(save_uid()){
            lcd.clear();
            lcd.print("Saved!!");
            delay(1000);
          }
          else{
            lcd.clear();
            lcd.print("Failed!!");
            delay(1000);
          }
    }
    return;
   }
  }

  }
  if(id == CARD_SUCCESS){
    Serial.print("身份确认，解锁成功");
    lcd.clear();
    lcd.print("Welcomeback.....");
    servopulse(180);
    delay(2000);
    servopulse(45);
    delay(2000);
  }
  //rfid.PCD_StopCrypto1();
  return;
}


void read_temp_hum(){
    humidity = dht.readHumidity();
    temprature = dht.readTemperature();  
}


void read_PIR(){
  PIR_distance = digitalRead(PIR_PIN);
  if(PIR_distance == 1){
    while(1){
    if(check_id() == CARD_SUCCESS){ 
      // check the card 
      PIR_distance= 0; // reset the PIR flag
      lcd.clear();
      lcd.print("Welcomeback.....");
      servopulse(180);
      delay(2000);
      servopulse(45);
      delay(2000);
      return;
    }
    lcd.clear();
    lcd.println("Use Card to Unlock....");
    }
  }
}


void read_light(){
  light_value = analogRead(LIGHT_PIN);
}
/*
Name: send_sensor_data()
Function: package temprature,humidity,light value,PIR distance into a frame   and send it to UpperMCU.
Frame:
     Strating Code-------------------------1 start to receive sensors data
     temprature_part1
     temprature_part2
     Humidity Code-------------------------2
     humidity_part1
     humdity_part2
     Light code----------------------------3
     light value
     PIR code-----------------------------4
     PIR_distance
     end code------------------------------5
            
*/
void send_sensor_data(){
  sensor_data[0] = 1;
  sensor_data[1] = (int)temprature;
  sensor_data[2] = (int)((temprature - sensor_data[1])*100);
  sensor_data[3] = (int)humidity;
  sensor_data[4] = (int)((humidity - sensor_data[3])*100);
  sensor_data[5] = light_value;
  sensor_data[6] = PIR_distance;
  sensor_data[7] = END_CODE;
  for(int i = 0;i<SENSOR_DATA_LENGETH;i++){
    lowermcuSerial.write(sensor_data[i]);
  }
}
/*
Name: servopulse(int angle)
Function:Send PWM for Servo
input: the angle(0~180)
*/
void servopulse(int angle)
{ 
   
  //Send 50 Pulse
  for (int i = 0; i < 50; i++) { 
   
    int pulsewidth = (angle * 11) + 500;  //calculate the duration of PWM
    digitalWrite(MOTOR_PIN, HIGH);   
    delayMicroseconds(pulsewidth);  
    digitalWrite(MOTOR_PIN, LOW);   
    delayMicroseconds(20000 - pulsewidth);
  }
  delay(20);
}




void setup() {
  Serial.begin(9600);
  lowermcuSerial.begin(9600);
  lcd.createChar(6, temperturechar);
 
  // RFID Set up

  SPI.begin();
  rfid.PCD_Init();
  pinMode(MOTOR_PIN, OUTPUT); //set servo
  for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
   }
  //door_servo.write(45);
  

  // set dht11
  dht.begin();

  //set PIR sensor
  pinMode( PIR_PIN, INPUT);
  Serial.print("setup ok!");
  //lcd setup
  lcd.begin(16, 2);
  //system ver
  lcd.print("System ver 0.0.1");
  delay(1000);
}

void loop() {
  //int uid_flag = check_id(); // get the status of current RFID reader status.
  
  //rfid.PCD_Init();
  
  rfid_check(check_id()); //check card
  read_temp_hum();
  read_PIR();
  read_light();
  if(page_flag == 0){
  showTempAndHum();
  page_flag = 1;
  }
  else{
  showLight();
  page_flag = 0;
  }

  send_sensor_data();

  delay(1000);
}
void showTempAndHum(){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hum:");
    lcd.print(humidity);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print(temprature);
    lcd.write(byte(6));
}
void showLight(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Light:");
  lcd.print(light_value);
  lcd.print("LUX");  
}
