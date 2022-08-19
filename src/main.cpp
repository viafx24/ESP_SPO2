
#include <Arduino.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"

// Interrupt pin
const byte oxiInt = 19; // pin connected to MAX30102 INT

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;

String Data;


void millis_to_hours(uint32_t ms, char* hr_str);

void setup() {

  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

  Serial.begin(115200);

  maxim_max30102_init();  //initialize the MAX30102
  old_n_spo2=0.0;

  // while(Serial.available()==0)  //wait until user presses a key
  // {
  //   Serial.println(F("Press any key to start conversion"));
  //   delay(1000);
  // }
  // uch_dummy=Serial.read();

  timeStart=millis();
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
     
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts

    // here is the switch IR/RED due to hardware defect (Guillaume)
    maxim_max30102_read_fifo((aun_ir_buffer + i), (aun_red_buffer + i)); //read from MAX30102 FIFO

  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
  elapsedTime=millis()-timeStart;

  elapsedTime/=1000; // Time in seconds

  if(ch_hr_valid && ch_spo2_valid) { 

    Data=String(String(elapsedTime) + ',' + String(n_spo2) + ','  + String(n_heart_rate) + ',');

    Serial.println(Data);

    old_n_spo2=n_spo2;
  }
}