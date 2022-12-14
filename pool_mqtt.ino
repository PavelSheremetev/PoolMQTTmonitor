#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/

#define MQTT_RECONNECTIO_MS_TIME 5000


///////////////////////////////////////////////////////////////////////////
#define TdsSensorPin A0
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

WiFiManager wm;
///////////////////////////////////////////////////////////////////////////

WiFiClient espClient;
PubSubClient mqttClient(espClient);

long lastReconnectAttempt = 0;
const char * mqttServer = "192.168.13.235";

char id[16]; //Create a Unique AP from MAC address
void createID() {
  //uint64_t chipid=ESP.getEfuseMac() ;//The chip ID is essentially its MAC address(length: 6 bytes).
 // uint32_t chipid=ESP.getChipId();//The chip ID is essentially its MAC address(length: 6 bytes).
 // uint16_t chip = (uint16_t)(chipid>>40);

uint32_t chip = 0;
for(int i=0; i<17; i=i+8) {
  chip |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
}
  
  snprintf(id,16,"Robonomics-%04X",chip); 
}

char ConnectionTopic[37] = "";
char TDSTopic[32] = "";

void createTOPICS() {
//  uint32_t chip=ESP.getChipId();

uint32_t chip = 0;
for(int i=0; i<17; i=i+8) {
  chip |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
}
  snprintf(ConnectionTopic,37,"Robonomics/%08X/connection/status",chip);
  snprintf(TDSTopic,32,"Robonomics/%08X/sensors/TDS",chip);
}

void callback(char *topic, byte *payload, unsigned int payloadLenght)
{
  Serial.println("recibiendo informacion desde el topico:");
  Serial.println(topic);
}

boolean reconnectMqtt()
{
  
  // Attempt to connect
  if (mqttClient.connect(id, "mqtt", "mqtt"))
  { 
    // Once connected, publish an announcement...
    mqttClient.publish( ConnectionTopic, "1");
    //mqttClient.publish( "connection/status", "1");
    // ... and resubscribe
    mqttClient.subscribe("inTopic");
  }
  return mqttClient.connected();
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void setup()
{
  Serial.begin(115200);

  pinMode(TdsSensorPin,INPUT);

  createID();
  createTOPICS();

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "", 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "", 6);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
    
  wm.setDarkMode(true);
  // put your setup code here, to run once:

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP



  bool res;
  res = wm.autoConnect(id); // password protected ap
  
  if (!res)
  {
    Serial.println("Failed to connect");
    // ESP.restart();
  }
  else
  {
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!mqttClient.connected())
  {

    long now = millis();
    if (now - lastReconnectAttempt > MQTT_RECONNECTIO_MS_TIME)
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnectMqtt())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  else
  {
    // Client connected


   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");

      char result[8]; // Buffer big enough for 7-character float
      dtostrf(tdsValue, 6, 0, result);

      Serial.print("TDS Value:");
      Serial.print(result);
      Serial.println("ppm");
      mqttClient.publish(TDSTopic, result );
      
   }
    mqttClient.loop();
  }
}
