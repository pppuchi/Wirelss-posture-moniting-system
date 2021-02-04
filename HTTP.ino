#define pulsePin A0
#include <ESP8266WiFi.h>  // The library to use the functions of WiFi module esp8266
#include <Wire.h>         // I2C communication library.



const int MPU_ADDR =      0x68; // The definition of the address of the sensor mpu6050 (0x68)

const int WHO_AM_I =      0x75; // Registration of identification of device.

const int PWR_MGMT_1 =    0x6B; // Record of configuration of the energy management

const int GYRO_CONFIG =   0x1B; // Registration of the gyroscope configuration

const int ACCEL_CONFIG =  0x1C; // The record of the accelerometer configuration

const int ACCEL_XOUT =    0x3B; // Record of reading of the X axis of the accelerometer


const int scl_pin = D5; // The definition of the SCL I2C pin.
const int sda_pin = D6; // The definition of the pin I2C SDA



bool led_state = false;



//  variables to store the raw data of the accelerometer

int16_t AcXR, AcYR, AcZR, TmpR, GyXR, GyYR, GyZR; //raw values

float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Required values

const float accScale = -2.0 / 32768.0; //
const float gyroScale = -250.0 / 32768.0; //


const char* SSID = "pikachu";//"FinalYearProject";

const char* PASSWORD = "huawei12345";//"lhs970228";


WiFiClient client;

const char* IpApiHost = "ip-api.com";   

const int channelID = 699908;

String writeAPIKey = "EM8H4N3LLQ41PO6H"; 

const char* server = "api.thingspeak.com";

const int postingInterval = 30 * 1000; 


int rate[10];                    

unsigned long sampleCounter = 0; 

unsigned long lastBeatTime = 0;  

unsigned long lastTime = 0, N;

int BPM = 0;

int IBI = 0;

int P = 512;

int T = 512;

int thresh = 512;  

int amp = 100;                   

int Signal;

boolean Pulse = false;

boolean firstBeat = true;          

boolean secondBeat = true;

boolean QS = false;    





void initI2C() 

{

  //Serial.println("---inside initI2C");

  Wire.begin(sda_pin, scl_pin);

}


//function that writes a given value into a given record

void writeRegMPU(int reg, int val)

{
  
  Wire.beginTransmission(MPU_ADDR);     // Initiates communication with the address of the mpu6050
  Wire.write(reg);                      // Send the record to "reg"

  Wire.write(val);                      // Writes the value in the register

  Wire.endTransmission(true);           // Ends the transmission

}



//Function that reads a given record

uint8_t readRegMPU(uint8_t reg)

{

  uint8_t data;

  Wire.beginTransmission(MPU_ADDR);     // initiates communication with the MPU6050 address

  Wire.write(reg);                      // sends the record to fegister

  Wire.endTransmission(false);          // ends transmission but continues with I2C open (sends STOP and START)

  Wire.requestFrom(MPU_ADDR, 1);        // configures to receive 1 byte from the record chosen above.
  
  data = Wire.read();                   // reads byte and saves in 'data'

  return data;                          //returns 'data'

}



//function that searches the sensor at address 0x68
void findMPU(int mpu_addr)

{

  Wire.beginTransmission(MPU_ADDR);

  int data = Wire.endTransmission(true);



  if(data == 0)

  {

    Serial.print("Device found at address: 0x");

    Serial.println(MPU_ADDR, HEX);

  }

  else 

  {

    Serial.println("Device is not found!");

  }

}




//function that verifies whether the sensor responds and is active

void checkMPU(int mpu_addr)

{

  findMPU(MPU_ADDR);

    

  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75

  

  if(data == 104) 

  {

    Serial.println("MPU6050 Device responded OK!");



    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B



    if(data == 64) Serial.println("MPU6050 in SLEEP mode! (64)");

    else Serial.println("MPU6050 in ACTIVE mode!"); 

  }

  else Serial.println("Check Device - MPU6050 NOT Available!");

}




//sensor initialization function

void initMPU()

{

  setSleepOff();

  setGyroScale();

  setAccelScale();

}





//Function to configure the sleep bit

void setSleepOff()

{

  writeRegMPU(PWR_MGMT_1, 0); // write 0 in the power management register (0x68), placing the sensor in ACTIVE mode

}



/* function to configure the gyro scales    
 *  gyroscope scale register: 0x1B [4: 3]



    FS_SEL  Full Scale Range   Sensitivity

      0        ± 250 °/s      131 LSB/°/s

      1        ± 500 °/s      65.5 LSB/°/s

      2        ± 1000 °/s     32.8 LSB/°/s

      3        ± 2000 °/s     16.4 LSB/°/s

*/

void setGyroScale()

{

  writeRegMPU(GYRO_CONFIG, 0);

}



/* function to configure the accelerometer scales    
 *  accelerometer scale record: 0x1C [4: 3]

Full Scale Range* LSB Sensitivity=32768

    AFS_SEL   Full Scale Range    Sensitivity

      0           ± 2g            16384 LSB/g

      1           ± 4g            8192 LSB/g

      2           ± 8g            4096 LSB/g

      3           ± 16g           2048 LSB/g

*/

void setAccelScale()

{

  writeRegMPU(ACCEL_CONFIG, 0);

}



/* The function that reads the data 'raw' (raw data) of the sensor
There are 14 bytes in total and they 2 bytes for each axis and 2 bytes to temperature:



  0x3B 59 ACCEL_XOUT[15:8]

  0x3C 60 ACCEL_XOUT[7:0]

  0x3D 61 ACCEL_YOUT[15:8]

  0x3E 62 ACCEL_YOUT[7:0]

  0x3F 63 ACCEL_ZOUT[15:8]

  0x40 64 ACCEL_ZOUT[7:0]



  0x41 65 TEMP_OUT[15:8]

  0x42 66 TEMP_OUT[7:0]



  0x43 67 GYRO_XOUT[15:8]

  0x44 68 GYRO_XOUT[7:0]

  0x45 69 GYRO_YOUT[15:8]

  0x46 70 GYRO_YOUT[7:0]

  0x47 71 GYRO_ZOUT[15:8]

  0x48 72 GYRO_ZOUT[7:0]

   

*/

void readRawMPU()

{  

  Wire.beginTransmission(MPU_ADDR);       // initiates communication with the MPU6050 address
  
  Wire.write(ACCEL_XOUT);                       // sends the record with which to work, starting with record 0x3B (ACCEL_XOUT_H)

  Wire.endTransmission(false);            // ends transmission but continues with I2C open (sends STOP and START)

  Wire.requestFrom(MPU_ADDR, 14);         // configures to receive 14 bytes starting from the register chosen above (0x3B)


  AcXR = Wire.read() << 8;                 // reads the most significant byte first

  AcXR |= Wire.read();                     // then reads the least significant bit

  AcYR = Wire.read() << 8;

  AcYR |= Wire.read();

  AcZR = Wire.read() << 8;

  AcZR |= Wire.read();



  TmpR = Wire.read() << 8;

  TmpR |= Wire.read();



  GyXR = Wire.read() << 8;

  GyXR |= Wire.read();

  GyYR = Wire.read() << 8;

  GyYR |= Wire.read();

  GyZR = Wire.read() << 8;

  GyZR |= Wire.read(); 

AcX=AcXR*accScale;
AcY=AcYR*accScale;
AcZ=AcZR*accScale;
Tmp=TmpR/340.00+36.53;  //equation for temperature in degrees C from datasheet
GyX=GyXR*gyroScale;
GyY=GyYR*gyroScale;
GyZ=GyZR*gyroScale;

  Serial.print("AcX = "); Serial.print(AcX);

  Serial.print(" | AcY = "); Serial.print(AcY);

  Serial.print(" | AcZ = "); Serial.print(AcZ);

  Serial.print(" | Tmp = "); Serial.print(Tmp);

  Serial.print(" | GyX = "); Serial.print(GyX);

  Serial.print(" | GyY = "); Serial.print(GyY);

  Serial.print(" | GyZ = "); Serial.println(GyZ);

if(AcZ<=-0.58){
  Serial.print(" | Sitting\n");
  }
 else if (AcZ<=0.73){
  Serial.print(" | Standing\n");
  }
  else {
    if (BPM<100||BPM>60){
    Serial.print(" | Lying\n");
    }
    else {Serial.print(" | Fall\n");}
    }
  
  led_state = !led_state;

  digitalWrite(LED_BUILTIN, led_state);         // flashes the NodeMCU LED at each sensor reading
  delay(500);                                        

}



/*
 * function that connects the NodeMCU to the Wifi network  
 * SSID and PASSWORD must be indicated in variables

 */

void reconnectWiFi() 

{

  if(WiFi.status() == WL_CONNECTED)

    return;



  WiFi.begin(SSID, PASSWORD);



  while(WiFi.status() != WL_CONNECTED) {

    delay(100);

    Serial.print(".");

  }



  Serial.println();

  Serial.print("Successfully connected to the network: ");

  Serial.println(SSID);

  Serial.print("IP obtained: ");

  Serial.println(WiFi.localIP());  

}



void initWiFi()

{

  delay(10);

  Serial.print("Connecting to the network: ");

  Serial.println(SSID);

  Serial.println("Wait");



  reconnectWiFi();

}



void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);



  Serial.println("\nStarting WiFi setup\n");

  initWiFi();

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

  }

  Serial.println("\nStarting MPU6050 setup\n");

  initI2C();

  initMPU();

  checkMPU(MPU_ADDR);

  

  Serial.println("\nConfiguration Complete, Starting Loop\n");  

}

void readPulse() {



  Signal = analogRead(pulsePin);              

  sampleCounter += 2;                           

  int N = sampleCounter - lastBeatTime;   



  detectSetHighLow();



  if (N > 250) {  

    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) )

      pulseDetected();

  }



  if (Signal < thresh && Pulse == true) {  

    Pulse = false;

    amp = P - T;

    thresh = amp / 2 + T;  

    P = thresh;

    T = thresh;

  }



  if (N > 2500) {

    thresh = 512;

    P = 512;

    T = 512;

    lastBeatTime = sampleCounter;

    firstBeat = true;            

    secondBeat = true;           

  }

}



void detectSetHighLow() {



  if (Signal < thresh && N > (IBI / 5) * 3) {

    if (Signal < T) {                       

      T = Signal;                         

    }

  }



  if (Signal > thresh && Signal > P) {    

    P = Signal;                           

  }                                       



}



void pulseDetected() {

  Pulse = true;                           

  IBI = sampleCounter - lastBeatTime;     

  lastBeatTime = sampleCounter;           



  if (firstBeat) {                       

    firstBeat = false;                 

    return;                            

  }

  if (secondBeat) {                    

    secondBeat = false;                

    for (int i = 0; i <= 9; i++) {   

      rate[i] = IBI;

    }

  }



  word runningTotal = 0;                   



  for (int i = 0; i <= 8; i++) {          

    rate[i] = rate[i + 1];            

    runningTotal += rate[i];          

  }



  rate[9] = IBI;                      

  runningTotal += rate[9];            

  runningTotal /= 10;                 

  BPM = 60000 / runningTotal;         

  QS = true;

  if (client.connect(server, 80)) {

String str_AcX = String(AcX);
String str_AcY = String(AcY);
String str_AcZ = String(AcZ);
String str_Tmp = String(Tmp);
String str_GyX = String(GyX);
String str_GyY = String(GyY);
String str_GyZ = String(GyZ);
String body = "api_key="+writeAPIKey+"&field1="+BPM+"&field2="+str_AcX+"&field3="+str_AcY+"&field4="+str_AcZ+"&field5="+str_Tmp+"&field6="+str_GyX+"&field7="+str_GyY+"&field8="+str_GyZ; 


    client.println("POST /update HTTP/1.1");

    client.println("Host: api.thingspeak.com");

    client.println("User-Agent: ESP8266 (nothans)/1.0");

    client.println("Connection: close");

    client.println("X-THINGSPEAKAPIKEY: " + writeAPIKey);

    client.println("Content-Type: application/x-www-form-urlencoded");

    client.println("Content-Length: " + String(body.length()));

    client.println("");

    client.print(body);

  }

  client.stop();

  delay(postingInterval);                              

}

void loop() {

  readRawMPU();    // reads sensor data

  if (QS == true) {

   Serial.println("BPM: "+ String(BPM));

   QS = false;

   } else if (millis() >= (lastTime + 2)) {

     readPulse(); 

     lastTime = millis();

   }     
  
  delay(100);  

}
