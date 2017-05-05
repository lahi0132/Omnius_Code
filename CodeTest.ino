/*
Libraries
=====================
*/
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

int COMS =1; //don't print stuff out if =0; print stuff if we ==1
/*
Sensor Pin Constants
=====================
*/
int Buzzer = 3; //Kevin
int Button = 7; //Kevin
int PIR1 = 6; //David
int PIR2 = 4; //David
int PIR3 = 8;  // David
int Neopixel = 9;//David
int IR_LED = 7; //Kevin
int IR_diode =  3;//Kevin
int TMP36 = 1; //Kevin
int led = 13; // test led to note change on pin 12.
int Light_Sensor = 0; //Kevin
int Mic = 6; //Kevin

// SPi LINES not wokring
// RESET variables
long count = 0;
long threshold_Reset = 5000;

// I2C LINES
//Acceleromter = SDA, SCL

//mic setup:
float threshold_MIC = 40;
int Mic_sensor = 6;
int delta_mic = 0;
float mic_avg =0;
int micADC =0;


//Neopixel Setup
int numberPixels = 1;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numberPixels, Neopixel, NEO_GRB + NEO_KHZ800);

//Temperature sensor variables:
int tempADC = 0; //analogRead of temperature
float tempV = 0.0; //Temperature in Volts, will be converted to C
float tempC = 0.0; // temperature in celcius
float tempF =0.0;
float temp_avg = 0.0;
float threshold_temp =1; //ADC value
int V_cc =5;
int CorrectionC = 3; //correction value;
int delta_temp = 0;

//Accelerometer sensor variables
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
long acc_avg_x, acc_avg_y, acc_avg_z, acc_avg_GyX; // these are the initial positions from accelerometer data
long delta_X, delta_Y, delta_Z, delta_GyX, delta_GyY, delta_GyZ  ;
int CountX; // trigger count
long threshold_ACC_x = 16500;
long threshold_ACC_y = 16500;
long threshold_ACC_z = 16500;
long threshold_Gy_x = 16500; // this needs to be calibrated. not sure if actually the right value.
long threshold_Gy_y = 16500;
long threshold_Gy_z = 16500;

//Photoresistor variables:
long LightADC = 0;
long delta_light = 0;
float light_avg = 0.0;
int threshold_Light =80;


//IR beambreak variables:
float IR_avg =0.0;
long delta_IR = 0.0;
int threshold_IR = 3; ;

void setup(){
  Serial.begin(9600);
  Serial.println("Start Combined Code");

  pinMode(led, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(IR_diode, OUTPUT);
  pinMode(PIR3, INPUT);

  //Acceleromter Start:
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  initializeACC();
  initializeLight(); //get the starting light
  initializeTmp36();
  initializeBeamBreak(); //need a mode switch on this
  initializeMic();
  //probably will go in a function to start it off.
  //neopixel start
  pixels.begin(); // This initializes the NeoPixel.

}

void loop(){
  tmp36Read(255,50,255); //Works
  accRead(0,255,0); //works
  PhotoresistorRead(0,0,255); //works
  PIR3Read(0,255,0);
  micRead(255,255,255);
  beamBreakRead(140,255,255);

  micADC = analogRead(Mic);
  delta_mic = abs(mic_avg - micADC);
  Serial.println(delta_mic);
  //delay(1000);

  count++;
 if (count >= threshold_Reset) {
if(COMS ==1){
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  Serial.println("********* ====== ****** ====********* ====== ****** ====********* ====== ****** ====");
  Serial.println(" \t \t \tRESET!!");
  Serial.println("********* ====== ****** ====********* ====== ****** ====********* ====== ****** ====");
  delay(1000);
}
initializeACC();
initializeLight(); //get the starting light
initializeTmp36();
initializeBeamBreak(); //need a mode switch on this
initializeMic();
   count = 0;
}
pixels.setPixelColor(0, pixels.Color(0 ,0,0));
pixels.show();
}

int PIR3Read(int r, int g, int b){
  if(digitalRead(PIR3) == 1 || digitalRead(PIR2) || digitalRead(PIR1)){
    flashLed(r,g,b);
    return 1;
  }
  return 0;
}



int beamBreakRead(int r, int g, int b){
delta_IR = abs(IR_avg - analogRead(IR_diode));

if(stateChange()){
flashLed(r,g,b);
return 1;
}
return 0;

}

void flashLed(int r, int g, int b){ //in the even of something changed, we will flash this led on board
digitalWrite(led, HIGH);
delay(45);
digitalWrite(led, LOW);
delay(45);

NeopixelON(0,r,g,b);
buz(40);
}

int PhotoresistorRead(int r, int g, int b){
  LightADC = analogRead(Light_Sensor);
if(COMS == 1){
  Serial.println();
  Serial.print(" \t \t Light Value   ");
  Serial.print(LightADC);
  Serial.print("\t");
  Serial.print("average light ");
  Serial.print(light_avg);
  Serial.println();
}
  // int delta_light = abs(LightADC-light_avg);
  // if(delta_light > threshold_Light){
  //   return 1;
  // }
  // else
  // return 0;
  //return LightADC; //maybe do a return value? not sure yet.
  if(stateChange()){
    flashLed(r, g, b);
    return 1;
  }
  return 0;
}

void initializeMic(){
  for(int i=0; i<10; i++)
  {
   mic_avg += analogRead(Mic);
  }
  mic_avg /= 10;
}



void initializeACC() {
  // get inital values for the accelerometer
  for (int i = 0; i < 10; i++) {
    acc_avg_x += Wire.read() << 8 | Wire.read();
        acc_avg_y += Wire.read() << 8 | Wire.read();
            acc_avg_z += Wire.read() << 8 | Wire.read();
              acc_avg_GyX += Wire.read() << 8 | Wire.read();
  }
  acc_avg_x /= 10; //find average
    acc_avg_y /= 10; //find average
      acc_avg_z /= 10; //find average
      acc_avg_GyX /= 10;
}

void initializeLight(){
  for(int i=0; i<10; i++){
    light_avg += analogRead(Light_Sensor);
  }
      light_avg /= 10;
}

void initializeTmp36(){
  for(int i=0; i<10; i++){
    temp_avg += analogRead(TMP36);
  }
      temp_avg /= 10;
}



void initializeBeamBreak(){
  digitalWrite(IR_LED, HIGH);
  delay(1000 ); // requires a warmup period
  for(int i=0; i<10; i++){
    IR_avg += analogRead(IR_diode);
  }//df
      IR_avg/= 10;
}

//mic Checks
int micRead(int r, int g, int b){
  if(stateChange()){
          flashLed(r,g,b);
        //  delay(10000);
          return 1;
  }
  return 0;
}



int stateChange() {
//   //initializeACC();
//Acceleromter Checks
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();

  delta_X = abs(acc_avg_x - AcX); //change value
  delta_Y = abs(acc_avg_y - AcY); //change value
  delta_Z = abs(acc_avg_z - AcZ);
  delta_GyX = abs(acc_avg_GyX - GyX);
//Light Checks
  delta_light = abs(light_avg - analogRead(Light_Sensor));


//Temp Checks
  delta_temp = abs(temp_avg - analogRead(TMP36));

//IR beam Break checks
  delta_IR = abs(IR_avg - analogRead(IR_diode));

   if (delta_X > threshold_ACC_x || delta_Y > threshold_ACC_y || delta_Z > threshold_ACC_z || delta_GyX > threshold_Gy_x) {
     Serial.println(delta_GyX);
    CountX ++;//incriment
    if(COMS ==1 ){
      Serial.print("delta acc");
      Serial.println(delta_X);
    }
    return 1;
  }
  if(delta_light > threshold_Light){
    if(COMS ==1 ){
      Serial.print("delta light");
      Serial.println(delta_light);
        }
        return 1;
  }
  if(delta_temp > threshold_temp){
    if(COMS ==1 ){
      Serial.println();
      Serial.print("delta tmp ");
      Serial.print(delta_temp);
      Serial.print("\t temp_avg ");
      Serial.println(temp_avg);
    }
        return 1;
        }
  if(delta_IR > threshold_IR){
    if(COMS ==1 ){
      Serial.println();
      Serial.print("delta IR ");
      Serial.print(delta_IR);
      Serial.print("\t IR_avg ");
      Serial.println(IR_avg);
    }
      return 1;
    }
micADC = analogRead(Mic);
delta_mic = abs(mic_avg - micADC);
Serial.println(delta_mic);
    if(delta_mic>threshold_MIC){
      if(COMS ==1){
        Serial.println("mic triggereD");
        Serial.println(delta_mic);
      }
      return 1;
    }

else
return 0;
 }//end of State Change

int accRead(int r, int g, int b){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
 AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 if(COMS == 1){
  Serial.println();
  Serial.print(AcX);
  Serial.print("\t");
  Serial.print(" Delta:   ");
  Serial.print(delta_X);
  Serial.print("\t \tThe trigger Count is: \t");
  Serial.print(CountX);
  Serial.print(AcY);
      }
 if (stateChange()) // checks to see if a state has change
      {
          flashLed(r,g,b);
            return 1;
            }
            return 0;
}



int tmp36Read(int r, int g, int b){
  tempADC = analogRead(TMP36);
  tempV = ((float)tempADC/1023.0)*((float)V_cc); // V_cc is 5v,
  tempC = ((tempV - .5)/ .010) + CorrectionC;
  tempF = tempC * (9.0/5.0) +32.0;
  if(COMS == 1){
    Serial.println(tempF);
    Serial.print("\t °F");
    Serial.print(tempC);
    Serial.print("\t °C");
   }
   if(stateChange()){
     flashLed(255,0,0);
     return 1;
   }
    return 0;
    //  Serial.println();
    }

void NeopixelON(int led_index, int r, int g, int b){
  pixels.setPixelColor(led_index, pixels.Color(r,g,b)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(200);
  pixels.setPixelColor(led_index, pixels.Color(r,g,b));
  pixels.show();
  delay(200);
}

void buz(int time){
  for(int i=0; i<=10000; i++){
    digitalWrite(Buzzer, HIGH);
    delayMicroseconds(time);
    digitalWrite(Buzzer, LOW);
    delayMicroseconds(time);
  }
}
