#include <ESP8266WiFi.h>  // biblioteca para usar as funções de Wifi do módulo ESP8266

#include <Wire.h>         // biblioteca de comunicação I2C

//成功读取3个数据 15S更新

const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)

const int WHO_AM_I =      0x75; // registro de identificação do dispositivo

const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia

const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio

const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro

const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro



const int sda_pin = D6; // definição do pino I2C SDA

const int scl_pin = D5; // definição do pino I2C SCL



bool led_state = false;



// variáveis para armazenar os dados "crus" do acelerômetro

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 



// Definições da rede Wifi

const char* SSID = "Password 8 eights";

const char* PASSWORD = "lhs970228";



// endereço IP local do Servidor Web instalado na Raspberry Pi 3

// onde será exibida a página web

String apiKey = "CO989RDEMD7CLP18";



// servidor que disponibiliza serviço de geolocalização via IP    

const char* server = "api.thingspeak.com";   



WiFiClient client;




void initI2C() 

{

  //Serial.println("---inside initI2C");

  Wire.begin(sda_pin, scl_pin);

}



/*

 * função que escreve um dado valor em um dado registro

 */

void writeRegMPU(int reg, int val)

{

  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050

  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar

  Wire.write(val);                      // escreve o valor no registro

  Wire.endTransmission(true);           // termina a transmissão

}



/*

 * função que lê de um dado registro

 */

uint8_t readRegMPU(uint8_t reg)

{

  uint8_t data;

  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050

  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar

  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)

  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima

  data = Wire.read();                   // lê o byte e guarda em 'data'

  return data;                          //retorna 'data'

}



/*

 * função que procura pelo sensor no endereço 0x68

 */

void findMPU(int mpu_addr)

{

  Wire.beginTransmission(MPU_ADDR);

  int data = Wire.endTransmission(true);



  if(data == 0)

  {

    Serial.print("Dispositivo encontrado no endereço: 0x");

    Serial.println(MPU_ADDR, HEX);

  }

  else 

  {

    Serial.println("Dispositivo não encontrado!");

  }

}



/*

 * função que verifica se o sensor responde e se está ativo

 */

void checkMPU(int mpu_addr)

{

  findMPU(MPU_ADDR);

    

  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75

  

  if(data == 104) 

  {

    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");



    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B



    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");

    else Serial.println("MPU6050 em modo ACTIVE!"); 

  }

  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");

}



/*

 * função de inicialização do sensor

 */

void initMPU()

{

  setSleepOff();

  setGyroScale();

  setAccelScale();

}



/* 

 *  função para configurar o sleep bit  

 */

void setSleepOff()

{

  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE

}



/* função para configurar as escalas do giroscópio

   registro da escala do giroscópio: 0x1B[4:3]

   0 é 250°/s



    FS_SEL  Full Scale Range

      0        ± 250 °/s      0b00000000

      1        ± 500 °/s      0b00001000

      2        ± 1000 °/s     0b00010000

      3        ± 2000 °/s     0b00011000

*/

void setGyroScale()

{

  writeRegMPU(GYRO_CONFIG, 0);

}



/* função para configurar as escalas do acelerômetro

   registro da escala do acelerômetro: 0x1C[4:3]

   0 é 250°/s



    AFS_SEL   Full Scale Range

      0           ± 2g            0b00000000

      1           ± 4g            0b00001000

      2           ± 8g            0b00010000

      3           ± 16g           0b00011000

*/

void setAccelScale()

{

  writeRegMPU(ACCEL_CONFIG, 0);

}



/* função que lê os dados 'crus'(raw data) do sensor

   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:



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

  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050

  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)

  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)

  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)



  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo

  AcX |= Wire.read();                     // depois lê o bit menos significativo

  AcY = Wire.read() << 8;

  AcY |= Wire.read();

  AcZ = Wire.read() << 8;

  AcZ |= Wire.read();



  Tmp = Wire.read() << 8;

  Tmp |= Wire.read();



  GyX = Wire.read() << 8;

  GyX |= Wire.read();

  GyY = Wire.read() << 8;

  GyY |= Wire.read();

  GyZ = Wire.read() << 8;

  GyZ |= Wire.read(); 



  Serial.print("AcX = "); Serial.print(AcX);

  Serial.print(" | AcY = "); Serial.print(AcY);

  Serial.print(" | AcZ = "); Serial.print(AcZ);

  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);

  Serial.print(" | GyX = "); Serial.print(GyX);

  Serial.print(" | GyY = "); Serial.print(GyY);

  Serial.print(" | GyZ = "); Serial.println(GyZ);



  led_state = !led_state;

  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor

  delay(50);                                        

}



/*

 * função que conecta o NodeMCU na rede Wifi

 * SSID e PASSWORD devem ser indicados nas variáveis

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

  Serial.print("Conectado com sucesso na rede: ");

  Serial.println(SSID);

  Serial.print("IP obtido: ");

  Serial.println(WiFi.localIP());  

}



void initWiFi()

{

  delay(10);

  Serial.print("Conectando-se na rede: ");

  Serial.println(SSID);

  Serial.println("Aguarde");

  reconnectWiFi();

}






void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);



  Serial.println("\nIniciando configuração WiFi\n");

  initWiFi();

 WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

  }

  Serial.println("\nIniciando configuração do MPU6050\n");

  initI2C();

  initMPU();

  checkMPU(MPU_ADDR);



}



void loop() {

  readRawMPU();    // lê os dados do sensor


  if (client.connect(server,80)) { // "184.106.153.149" or api.thingspeak.com
String str_sensor2 = String(AcX);
String str_sensor3 = String(AcY);
String str_sensor4 = String(AcZ);
String str_sensor5 = String(Tmp);
String str_sensor6 = String(GyX);
String str_sensor7 = String(GyY);
String str_sensor8 = String(GyZ);
String postStr = "api_key="+apiKey+"&field2="+str_sensor2+"&field3="+str_sensor3+"&field4="+str_sensor4+"&field5="+str_sensor5+"&field6="+str_sensor6+"&field7="+str_sensor7+"&field8="+str_sensor8; 


client.print("POST /update HTTP/1.1\n");
client.print("Host: api.thingspeak.com\n");
client.print("Connection: close\n");
client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
client.print("Content-Type: application/x-www-form-urlencoded\n");
client.print("Content-Length: ");
client.print(postStr.length());
client.print("\n\n");
client.print(postStr);

  delay(100);  
}
}
