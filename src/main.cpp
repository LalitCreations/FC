#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <SD.h>
#include <SimpleKalmanFilter.h> //EXPERIMENTAL!!! idk how well it works
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


//====== Baro ======
Adafruit_BMP085 baro;
SimpleKalmanFilter pressureEstimate(1, 1, 1.1); //EXPERIMENTAL!!!
double base_temp, bmp_temp;
float gnd_alt , base_press , est_alt, alt, press;
const float sea_level_press = 0.00;
float p_alt,c_alt,d_alt;


//====== SD card ======
String file_name = "data_file.csv";
File data_file;


//====== Pin ======
const int SD_pin = 1;
const int buzzer = 2;
const int led[3] = {3,4,5}; //(R,G,B)
const int pyro_1 = 6;
const int pyro_2 = 7;
const int imu_pin = 8;
const int lora_cs = 9;
const int lora_dio0 = 10; //pin should have digital interrupt
const int lora_reset = 11;
const int gps_rx = 2;
const int gps_tx = 3;


//====== IMU ======
Adafruit_ICM20649 imu;
float g_x,g_y,g_z,a_x,a_y,a_z,a_net,f_velo;
float i_velo = 0;

//====== Misc =====
int state;
int liftoff_threshold;
int chute_deployment_threshold;
unsigned long liftoff_detection_time = 0;
unsigned long elapsed_time = 0;
int launch = 0;
int land = 0;
int pyro = 0;


// ====== GPS ======
const int gps_baud = 9600;
TinyGPSPlus gps;
SoftwareSerial gps_serial(gps_rx, gps_tx);
float lat,lon,gps_alt;


void led_buzz(int led_state){
  digitalWrite(led[0], LOW);
  digitalWrite(led[1], LOW);
  digitalWrite(led[2], LOW);
  //state 1 ERROR
  if (led_state == 1){
    while (true) {
      digitalWrite(led[0], HIGH);
      digitalWrite(led[1], LOW);
      digitalWrite(led[2], LOW);
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
      delay(500);
    }
  } else if(led_state == 2){ //standby
    digitalWrite(led[0], LOW);
    digitalWrite(led[1], HIGH);
    digitalWrite(led[2], LOW);
  } else if (led_state==3){
    digitalWrite(led[0], LOW);
    digitalWrite(led[1], HIGH);
    digitalWrite(led[2], HIGH);
  } else if (led_state==4){
    digitalWrite(led[0],HIGH);
    digitalWrite(led[1],HIGH);
    digitalWrite(led[2],HIGH);
  } else if (led_state==5){
    digitalWrite(led[0],HIGH);
    digitalWrite(led[1],HIGH);
    digitalWrite(led[2],HIGH);      
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
  }
}

void setup_sd() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_pin)) {
    Serial.println("SD initialization failed!");
  } else {
    Serial.println("SD initialization done.");   
    led_buzz(1);
  }

  data_file = SD.open(file_name, FILE_WRITE);
  if (data_file) {

    data_file.print("Time");
    data_file.print(" ,");
    data_file.print("Altitude");
    data_file.print(" ,");
    data_file.print("Pressure");
    data_file.print(" ,");
    data_file.print("Est_Alt");
    data_file.print(" ,");
    data_file.print("BMP_temp");
    data_file.print(" ,");
    data_file.print("Ax");
    data_file.print(" ,");
    data_file.print("Ay");
    data_file.print(" ,");
    data_file.print("Az");
    data_file.print(" ,");
    data_file.print("Gx");
    data_file.print(" ,");
    data_file.print("Gy");
    data_file.print(" ,");
    data_file.print("Gz");
    data_file.print(" ,");
    data_file.print("Velo");
    data_file.print(" ,");
    data_file.print("Launch");
    data_file.print(" ,");
    data_file.print("Land");
    data_file.print(" ,");
    data_file.print("Pyro");
    data_file.print(" ,");
    data_file.print("Lat");
    data_file.print(" ,");
    data_file.print("Lon");
    data_file.print(" ,");
    data_file.println("GPS_Alt");
    
    data_file.close();
  } else {
    Serial.println("error opening data_file.csv");
    led_buzz(1);
  }

}

void setup_baro() {
  Serial.print("Initializing Baro...");
  if (baro.begin()) {
    Serial.println("BMP Initialization done!");
    for (int i = 0; i < 20; i++) { //averages ground pressure,altitude,temperature
        base_temp += baro.readTemperature() * 0.05;
        gnd_alt += baro.readAltitude(sea_level_press) * 0.05;
        base_press += baro.readPressure() * 0.05;
    }
  }else {
    Serial.println("Baro initialization failed!");
    led_buzz(1);
  }
}

void setup_imu () {
  Serial.print("Initializing IMU...");
  if (imu.begin_SPI(imu_pin)) {
    // if (imu.begin_I2C()) {
    // if (imu.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    imu.setAccelRange(ICM20649_ACCEL_RANGE_30_G); //4g,8g,16g,30g
    imu.setGyroRange(ICM20649_GYRO_RANGE_500_DPS); //500,1000,2000,4000 dps=> degree per second
    imu.setGyroRateDivisor(255);
    imu.setAccelRateDivisor(4095);
    Serial.println("IMU initialization done!");
  } else {
    Serial.println("IMU initialization Failed!");
    led_buzz(1);
  }
}

void setup_lora(){
  LoRa.setPins(lora_cs, lora_reset, lora_dio0);
  Serial.print("Initializing LoRa...");
  if (LoRa.begin(433E6)) { //LoRa.begin(Freq) 433mhz
    Serial.println("LoRa initialization done!");    
  } else {
    Serial.println("LoRa initialization failed!"); 
    led_buzz(1);
  }
}


void get_alt(){
  alt = baro.readAltitude(sea_level_press) - gnd_alt;
  press = baro.readPressure();
  bmp_temp = baro.readTemperature();
  est_alt = pressureEstimate.updateEstimate(alt); //EXPERIMENTAL!!!
}

void get_imu() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
  g_x = gyro.gyro.x;
  g_y = gyro.gyro.y;
  g_z = gyro.gyro.z;
  a_x = accel.acceleration.x;
  a_y = accel.acceleration.y;
  a_z = accel.acceleration.z;
  a_net = sqrt(pow(a_x,2) + pow(a_y,2) + pow(a_z,2));
}

void calc_velo(){ //v = at + u
  get_imu();
  f_velo = a_net*liftoff_detection_time + i_velo;
  f_velo = i_velo;
}

void get_gps() {
  if (gps_serial.available()){
    gps.encode(gps_serial.read());
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      gps_alt = gps.altitude.meters();
    }
  }

}

void data_store() { 
  get_alt();
  get_imu();
  get_gps();
  calc_velo();
  data_file = SD.open(file_name, FILE_WRITE);
  if (data_file) {
    data_file.print(elapsed_time = millis());
    data_file.print(" ,");
    data_file.print(alt);
    data_file.print(" ,");
    data_file.print(press);
    data_file.print(" ,");
    data_file.print(est_alt);
    data_file.print(" ,");
    data_file.print(bmp_temp);
    data_file.print(" ,");
    data_file.print(a_x);
    data_file.print(" ,");
    data_file.print(a_y);
    data_file.print(" ,");
    data_file.print(a_z);
    data_file.print(" ,");
    data_file.print(a_y);
    data_file.print(" ,");
    data_file.print(g_x);
    data_file.print(" ,");
    data_file.print(g_y);
    data_file.print(" ,");
    data_file.print(g_z);
    data_file.print(" ,");
    data_file.print(f_velo);
    data_file.print(" ,");
    data_file.print(launch);
    data_file.print(" ,");
    data_file.print(land);
    data_file.print(" ,");
    data_file.print(pyro);   
    data_file.print(" ,");
    data_file.print(lat); 
    data_file.print(" ,");
    data_file.print(lon); 
    data_file.print(" ,");
    data_file.println(gps_alt); 
    data_file.close();
  } 
  else {
    Serial.println("Error writing to data_file.csv");
  }
  delay(0); 
}


float delta_alt() {                       
  c_alt = baro.readAltitude(sea_level_press);
  float deltaAlt = c_alt - p_alt;
  p_alt = c_alt;
  return (deltaAlt);
}


void send_rf_packet(){
  LoRa.beginPacket();
  LoRa.print(liftoff_detection_time);
  LoRa.print(" ,");
  LoRa.print(lat);
  LoRa.print(" ,");
  LoRa.print(lon);
  LoRa.print(" ,");
  LoRa.print(alt);
  LoRa.print(" ,");
  LoRa.print(f_velo);
  LoRa.print(" ,");
  LoRa.print(pyro);
  LoRa.print(" ,");
  LoRa.println(state);
  LoRa.endPacket();
}

void setup() {
  Serial.begin(9600);
  setup_sd();
  setup_lora();
  setup_baro();
  setup_imu();
  pinMode(led[0],OUTPUT); 
  pinMode(led[1],OUTPUT);
  pinMode(led[2],OUTPUT);
  pinMode(buzzer,OUTPUT);
  state = 0;
}

void loop() {

  while (state == 0){ //idle
    get_imu();
    data_store();
    led_buzz(2);
    if (a_y > liftoff_threshold){
      liftoff_detection_time = millis();
      Serial.println("Liftoff confirmed!");
      launch =1;
      state = 1;
    }
  }

  while (state == 1) { //liftoff
    data_store();
    send_rf_packet();
    led_buzz(3);
    if (delta_alt() <= 0 || liftoff_detection_time >= chute_deployment_threshold) {
      state = 2;
      digitalWrite(pyro_1, HIGH);
      Serial.println("Chutes Deployed!!");
      pyro = 1;
    }
  }

  while (state == 2) { //under chutes
    data_store();
    send_rf_packet();
    led_buzz(4);
    float delta = abs(delta_alt());
    if(delta <= 0.17){
      Serial.println("Landed!");
      land = 1;
      state = 3;
    }
  }

  while (state == 3){
    led_buzz(5);
    get_gps();
    get_imu();
    get_alt();
    send_rf_packet();
  }
  

}
