// Usa MAX30100
#include <Kalman.h>
#include <Wire.h> // Biblioteca da comunicacao I2C
#include <Adafruit_ADS1015.h> //Library of external adc
#include "esp32-hal-cpu.h"
#include <WiFi.h>
#include "ArduinoJson.h"
#include "HttpServer.h"
#include "MAX30100_PulseOximeter.h"
#include <EEPROM.h>


// Comunicação bluetooth
//BluetoothSerial ESP_BT; //Object for Bluetooth
//int incoming;


//Reinicia sensores
int temp_0;
int acel_0;
int oxi_0;
int temp_reinit;
int acel_reinit;
int acel_global;
int oxi_reinit;

// Sensor de temperatura
Adafruit_ADS1115 ads(0x48); // define the types of external adc 
float temp = 0.0; // variable to store temperature value
float temp_acel;
float temp_envia;
float temperature_oxi = 32.0;



int LED_Conexao = 5;
int LED_Sensor = 33;
//int LED_Conexao = 2;

//Frequência respiratória
//Filtro passa alta e baixa ara contagem da frequência respiratória
float EMA_a = 0.3;			//initialization of EMA alpha
int EMA_S = 0;				//initialization of EMA S
float EMA_a_H = 0.1;
int EMA_H = 0;
//Variáveis para contagem da frequência respiratória
int t=0;
int vezes_resp = 3500;
float buffer_respiracao[3500];
long delta_resp;
long lastBeat_resp;
int frequencia_int = 15;


// Mensagem
float mensagem[7];


// Acelerômetro
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
/* IMU Data */
const int MPU1=0x68;  // Endereco I2C do MPU-6050 numero 1
const int MPU2=0x69;  // Endereco I2C do MPU-6050 numero 2
float AcX1, AcY1, AcZ1, Tmp1, GyX1, GyY1, GyZ1; // Leituras do MPU-6050 1
float AcX2, AcY2, AcZ2, Tmp2, GyX2, GyY2, GyZ2; // Leituras do MPU-6050 2
//float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Variaveis finais, que serao enviadas ao programa
float aceleracaox;
float aceleracaoy;
float aceleracaoz;
float const_calib = 16071.82;
float const_gravid = 9.81;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
int16_t tempRaw;

float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;


//Batimentos Cardiacos e Oxímetro

#define REPORTING_PERIOD_MS 100
float heart_rate_value_one = 0;
float oxi_value_one = 0;

// PulseOximeter is the higher level interface to the sensor
// it offers:
// * beat detection reporting
// * heart rate calculation
// * SpO2 (oxidation level) calculation
PulseOximeter pox;
 
uint32_t tsLastReport = 0;
 
// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
//Serial.println("Beat!");
}


void acelerometro()
	{
 //valor_mioeletrico = analogRead(pinosensor_muscular);
 
 Wire.beginTransmission(MPU1);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
  
  Wire.requestFrom(MPU1,14,true);  // request a total of 14 registers
  AcX1 = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY1 = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1 = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX1 = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1 = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ1 = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU2,14,true);  // request a total of 14 registers
  AcX2 = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY2 = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ2 = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  float temp_acel_2 = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX2 = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY2 = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ2 = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);

  accX = AcX2 - AcX1;
  accY = AcY2 - AcY1;
  accZ = AcZ2 - AcZ1;

  gyroX = GyX1 - GyX2;
  gyroY = GyY1 - GyY2;
  gyroZ = GyZ1 - GyZ2;

if (t<25){
accX = 16000;
accY = 16000;
accZ = 16000;
gyroX = 500;
gyroY = 100;
gyroZ = 100;
}

  float dt = (float)(millis() - timer) / 1000; // 1000000 Calculate delta time
  timer = millis();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  float roll  = atan2(accY, accZ) * RAD_TO_DEG;
  float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  float roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  float pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  float gyroXrate = gyroX / 131.0; // Convert to deg/s
  float gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complementary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  


aceleracaox = (( accX * const_gravid) / const_calib);
aceleracaoy = (( accY * const_gravid) / const_calib);
aceleracaoz = (( accZ * const_gravid) / const_calib);
                     
  int sensorValue = kalAngleX*kalAngleX;//-compAngleX;              //read the sensor value using ADC
  EMA_S = (EMA_a*sensorValue) + ((1-EMA_a)*EMA_S);  //run the EMA
 // EMA_S_2 = (EMA_a_2*sensorValue) + ((1-EMA_a_2)*EMA_S_2);  //run the EMA
//  EMA_H = (EMA_a_H*EMA_S) + ((1-EMA_a_H)*EMA_H);  //run the EMA
 // EMA_H_2 = (EMA_a_H_2*EMA_S_2) + ((1-EMA_a_H_2)*EMA_H_2);  //run the EMA
  int saida_filt = EMA_S; //EMA_S - EMA_H;
 // int saida_filt_2 = EMA_S_2 - EMA_H_2;
 // Serial.print("  "); Serial.print(saida_filt_2);  Serial.print("  ");
 // Serial.print("  "); Serial.print(EMA_S_2);  Serial.print("  ");
 Serial.println(saida_filt);  //Serial.print("  ");
 // Serial.print("  "); Serial.print(EMA_S);  Serial.println("  ");
 // Serial.print("  "); Serial.print(0);  Serial.print("  ");

  float acel_vetor = sqrt(pow(aceleracaox, 2) + pow(aceleracaoy, 2) + pow(aceleracaoz, 2));
  temp_acel = (tempRaw/340.00)+30.53; //+36.53
  buffer_respiracao[t]= saida_filt;

float maior_frente = 0.0;
float media_frente = 0.0;
float maior_tras = 0.0;
float media_tras = 0.0;
float media_media = 0.0;
float maior_media = 0.0;
float limiar = 0.0;

//Conta Respiração
vezes_resp = 3500; // Aproximadamente 30s
t++;
if(t>=vezes_resp){
     int meio = t/2;
  // Serial.print(" Inicia calculo dos picos ");
  // Serial.print(" meio  "); Serial.print(meio);

//Procura para frente
for (int i = meio ; i < (meio+200) ; i++){
  if(maior_frente < buffer_respiracao[i]){
    maior_frente = buffer_respiracao[i];
  }
  media_frente = media_frente + buffer_respiracao[i];
   //  Serial.print(" procura frente "); Serial.print(i);
  } // fim do for
  media_frente = media_frente/200;
  
//Procura para trás
for (int i = meio; i < (meio+200); i++){
  if(maior_tras < buffer_respiracao[i])
    maior_tras = buffer_respiracao[i];
   media_tras = media_tras + buffer_respiracao[i];
   //Serial.print(" procura tras ");
} // fim do for
  media_tras = media_tras/200;

maior_media = (maior_frente + maior_tras)/2;
media_media = (media_frente + media_tras)/2;
limiar = ((maior_media - media_media)/2)+media_media; 
    //Serial.println("  ");
    //Serial.print(maior_frente); Serial.print("  "); Serial.print(maior_tras); Serial.print("  "); Serial.print(maior_media); Serial.print("  "); 
    //Serial.print(media_frente); Serial.print("  "); Serial.print(media_tras); Serial.print("  "); Serial.print(media_media); Serial.print("  "); 
    //Serial.print(limiar); Serial.print("  ");
int conta_pico = 0;
int borda_subida = 1;
int pos_pico = 0;
int dif_pos = 0;
int grava_pos = 0;
// Contagem
for (int i = 50; i < t; i++){

  if((limiar < buffer_respiracao[i]) && (!borda_subida)){
   pos_pico = i;
  }

  dif_pos = i - pos_pico;
  
  if ((limiar > buffer_respiracao[i]) && (!borda_subida) && (dif_pos >80))
  borda_subida = 1;

   /*
  if((limiar > buffer_respiracao[i]) && (!borda_subida)){
  borda_subida = 1;
   }
  */
  //Serial.print(" zera borda ");
  if((limiar < buffer_respiracao[i]) && (borda_subida)){
    conta_pico++;
    borda_subida = 0;
   }
} // fim do for

    
delta_resp = millis() - lastBeat_resp;
lastBeat_resp = millis();
long delta_resp_s = delta_resp/1000;
float frequencia_resp = (60*conta_pico)/delta_resp_s;


if(frequencia_resp<3)
{
  acel_reinit++;
}else{
  acel_reinit=0;
  digitalWrite(LED_Sensor, LOW);
  }


if(frequencia_resp<10)
{
  frequencia_resp = 0;
  mensagem[0] = 0;
  if (frequencia_resp<3)
  {
    frequencia_resp = -acel_reinit;
    mensagem[0] = -acel_reinit;
  }
}





t=0; 
Wire.begin();
EMA_S = 0;
compAngleX = 0;
//Serial.print("  "); Serial.print(delta_resp_s); Serial.print("  ");
Serial.print("  "); Serial.print(conta_pico); Serial.print("  "); Serial.println(frequencia_resp);
acel_global = conta_pico;
  mensagem[0] = frequencia_resp; //  kalAngleY; acel_vetor;

}

return;
}






//Configuraão do wifi
const char * password = "Monitor@r";
char ssid[32] = "MonitorAR_";
WiFiServer Server(80);
bool station = false;

void setup() {
  pinMode(LED_Conexao, OUTPUT); 
  pinMode(LED_Sensor, OUTPUT);
  digitalWrite(LED_Sensor, HIGH);

	Serial.begin(115200);
	Serial.println("ok");
	IPAddress local_ip(192,168,1,4);
	IPAddress gateway(192,168,1,1);
	IPAddress subnet(255,255,255,0);
	delay(5000);

	ssid[10] = random(65,90);
	ssid[11] = NULL;
	boolean result = WiFi.softAP(ssid, "Monitor@r");
	if(result == true)
	{	
		WiFi.softAPConfig(local_ip, gateway, subnet);
		//WiFi.config(local_ip, dns, gateway, subnet);
		Serial.println("Ready");
		Serial.print("Soft-AP IP address = ");
		Serial.println(WiFi.softAPIP());
		Server.begin();
		Serial.printf("Web server started, open %s in a web browser\n", WiFi.softAPIP().toString().c_str());
	}
	else{
		Serial.println("Falha on start AP");
	}

	int tenta = 0;
	mensagem[0] = 0.0;
	mensagem[1] = 0.0;
	mensagem[2] = 0.0;
	mensagem[3] = 0.0;
	mensagem[4] = 0.0;
	mensagem[5] = 0.0;
	mensagem[6] = 0.0;

  Wire.begin();
  
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);  // registrador PWR_MGMT_1
  Wire.write(0);     // zera registrador, acordando o MPU-6050
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);  // registrador PWR_MGMT_1
  Wire.write(0);     // zera registrador, acordando o MPU-6050
  Wire.endTransmission(true);


	// Batimentos Cardíacos e Oxímetro
	delay(100);
	if (!pox.begin()) {
		Serial.println("FAILED");
		//for(;;;);              			//Não pode parar o código aqui por isso
	}
	else {
		Serial.println("SUCCESS");
	}
 
	// The default current for the IR LED is 50mA and it could be changed
	// by uncommenting the following line. Check MAX30100_Registers.h for all the
	// available options.
	pox.setIRLedCurrent(MAX30100_LED_CURR_40_2MA);
	//pox.setIRLedCurrent(MAX30100_LED_CURR_20_8MA );
	// Register a callback for the beat detection
	Serial.print("Heart rate:");
	pox.setOnBeatDetectedCallback(onBeatDetected);
	tsLastReport = 0;

for (int g = 0; g < 1000; g++)
{  
pox.update();
if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
  pox.getHeartRate();
  pox.getSpO2();
Serial.print("Heart rate:");
Serial.print(pox.getHeartRate());
Serial.print("bpm / SpO2:");
Serial.print(pox.getSpO2());
Serial.println("%");
 
tsLastReport = millis();
}
delay(10);
}


delay(100);
ads.begin(); // start adc to read sensor value over I2C 

} // fim setup


void envia();

void loop() {
	Serial.println("Loop initialized");
	mensagem[0] = 0.0;
	mensagem[1] = 0.0;
	mensagem[2] = 0.0;
	mensagem[3] = 0.0;
	mensagem[4] = 0.0;
	mensagem[5] = 0.0;
	mensagem[6] = 0.0;
	
	//Roda uma vez 
	//Alocar memória para o objeto
	const size_t CAPACITY = JSON_OBJECT_SIZE(6);	//+ JSON_ARRAY_SIZE(1);
	StaticJsonDocument<CAPACITY> doc;
	String JsonData;
	// Criar objeto
	JsonObject Data = doc.to<JsonObject>();
 	// JsonArray Aceleration = doc.createNestedArray("Aceleration_errado")[1];
	WiFiClient client = Server.available();

	int64_t timer_acelerometro = millis();
	int64_t timer_oximetro = millis();
	int64_t timer_stop_http = millis();
	int64_t timer_medias = millis();

	bool client_conected = false;

	float len_med_heart, len_med_oxi;
	float buffer_heart= 0;
	float buffer_oxi = 0;
	float med_heart_rate_value;
	float med_oxi_value;
  Serial.println("Before While-Loop");
	while(1){
    
		delay(2);
		client = Server.available();

		if (millis() - timer_acelerometro >= 20){
			timer_acelerometro = millis();
			acelerometro();
		}

		if (client_conected && ((millis() - timer_stop_http) > 500)){
			client.stop();
			client_conected = false;
      digitalWrite(LED_Conexao, LOW);
      digitalWrite(LED_Sensor, HIGH);
		}

		if (client && !client_conected)
		{	
			if (client.available()){
				//Serial.println("new client");
				Data["Aceleration"] = mensagem[0];//mensagem[5];
				Data["temperature"] = mensagem[1];//mensagem[1];
				//atualiza os demais campos do objeto com seus respectivos valores
				Data["bmp_instant"] = mensagem[3];//mensagem[2];
				Data["bmp_med"] = mensagem[2];//mensagem[3];
				Data["saturation_med"] = mensagem[4];
				Data["valid_saturation"] = 500;
				Data["batery"] = 1;
				JsonData = "";

				serializeJson(doc, JsonData);
				   
        digitalWrite(LED_Conexao, HIGH);

				manageRequest("data", &client, JsonData); //esta função trata a requisição de envia a resposta
				timer_stop_http = millis();
				client_conected = true;
				//Serial.println(doc.memoryUsage());
         timer = millis(); // acelerometro
         timer_acelerometro = millis();
			}
		}


		if (millis() - timer_oximetro > 0){
			//Batimentos Cardíacos e Oxímetro
			//MAX30100

      //  for (int g = 0; g < 1000; g++)
     //   {  
        //delay(20);
        pox.update();
        if ((millis() - tsLastReport) > REPORTING_PERIOD_MS) {
          heart_rate_value_one = pox.getHeartRate();
          oxi_value_one = pox.getSpO2();
          oxi_value_one = oxi_value_one + 2;
          //Serial.print("heart_rate_value_one: "); Serial.println(heart_rate_value_one);
          //Serial.print("oxi_value_one: "); Serial.println(oxi_value_one);
          
          //if (heart_rate_value_one < 40 || oxi_value_one < 70){
          //  oxi_desconected ++; 
          //}
          
          if (heart_rate_value_one > 40 && heart_rate_value_one < 200){
           len_med_heart += 1;
           buffer_heart += heart_rate_value_one;
           //Serial.print("heart_rate_value_one: "); Serial.println(heart_rate_value_one);
          }
        
          if (oxi_value_one > 70 && oxi_value_one < 101){
            len_med_oxi += 1;
            buffer_oxi += oxi_value_one;
            //Serial.print("oxi_value_one: "); Serial.println(oxi_value_one);
          }
         tsLastReport = millis();
         }
         delay(20);
         acelerometro();
      //  } // fim do for
      timer_oximetro = millis();
		}// fim de coleta de medições do oxímetro e dos batimentos cardiacos

		if (millis() - timer_medias > 15000){
			timer_medias = millis();

			if (len_med_oxi > 10){
				med_oxi_value = buffer_oxi/len_med_oxi;
			}
			else{
				//nehuma leitura válida foi efetuada
				med_oxi_value = 0;
			}


       if (len_med_heart > 10){
        med_heart_rate_value = buffer_heart/len_med_heart;
      }
      else{
        //nehuma leitura válida foi efetuada
        med_heart_rate_value = 0;
        med_oxi_value = 0;
      }
			
			buffer_heart = 0;
			len_med_heart = 0;
			buffer_oxi = 0;
			len_med_oxi = 0;

			mensagem[2] = med_heart_rate_value;
			mensagem[3] = med_heart_rate_value;
			mensagem[4] = med_oxi_value;
			mensagem[5] = med_oxi_value;
     // Serial.print("heart_rate_value_one: "); Serial.println( mensagem[2]);
     // Serial.print("oxi_value_one: "); Serial.println(mensagem[4]);
     // Serial.print("Acel: "); Serial.println(mensagem[0]);  Serial.print("Picos : "); Serial.println(acel_global); 
     // Serial.print("Error acel: "); Serial.println(acel_reinit);
     // Serial.print("Temperatura LM35  "); Serial.println(temp);
     // Serial.print("Temperatura acel ");  Serial.println(temp_acel);
			envia();

      //Revisa a ligação entre os sensores e a placa
     // if (mensagem[1]){
     //    acel_0++; 
     // }else{
     //    acel_0 = 0;
     // }

       if (mensagem[2] == 0 || mensagem[4] == 0){
         oxi_0++; 
      }else{
         oxi_0 = 0;
      }

      if (0){ //REINICIA ACELERÔMETRO
        Wire.begin();
        t = 0; // reinicia buffer do acelerômetro
        lastBeat_resp = millis();  // reinicia contador do acelerometro       
        Serial.println("acel reinit");
        temp_0 = 2;// reinicia temp pois depende de wire.h
        oxi_0 = 3; // reinia oxi pois depende de wire.h
        acel_reinit = 0;
      }       
      if (temp_0 > 1){ //REINICIA LM35
        ads.begin();
        Serial.println("ads reinit");
      }
      if (oxi_0 > 2){ //REINICIA OXÍMETRO
       if (!pox.begin()) {
        Serial.println("FAILED oxi reinit");
        }
       else {
        Serial.println("SUCCESS oxi reinit");
        pox.setIRLedCurrent(MAX30100_LED_CURR_40_2MA);
        Serial.print("Heart rate:");
        pox.setOnBeatDetectedCallback(onBeatDetected);
        tsLastReport = 0;

        for (int g = 0; g < 1000; g++)
        {  
          pox.update();
          if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
            pox.getHeartRate();
            pox.getSpO2();
            tsLastReport = millis();
          }
          delay(10);
        }
        t = 0; // reinicia buffer do acelerômetro
        lastBeat_resp = millis(); // reinicia contador do acelerometro
       }

      } // fim da reinicialização do oxímetro


        


      
		}// fim da média

		int bateria = 60;
		mensagem[6] = bateria;

	}   // fim do while-loop

} 	    // fim do loop


void envia()
{ 
	// LM35
	int16_t adc0; // 16 bit interger to store output of analog channel zero
	adc0 = ads.readADC_SingleEnded(0); // read ANO values
	temp = (adc0 * 0.1875)/1000; // convert ADC value into voltage
	temp = temp * 100; // converts voltage into temperature 10mv=1C
  if (temp < 2){
   temp_0++; 
  }else{
    temp_0 = 0;
  }
       

	if(temp > 32){
		temp_envia = temp;
	} else{
		//temp_envia = (temp_acel+temperature_oxi)/2;
		temp_envia = temp_acel;
		}

 if(temp_envia < 45){
  char str[6];
  // convert float to fprintf type string
// format 5 positions with 3 decimal places
//
dtostrf(temp_envia, 3, 1, str );
float actual = atof(str);
    mensagem[1] = actual;
 }



}
