#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BMP085.h>

#define QAIR_A 16
#define QAIR_B 17
#define DHT_PWR 4
#define DHT_DATA 23
#define DHTTYPE DHT11 
#define TOKEN_TB "5hS7riBz8EOuFzo8uGjQ"

const char* ssid     = "SSID";
const char* password = "PASS";
static int Wconectado = 0;
const char* mqtt_server = "190.2.22.61";
DHT dht(DHT_DATA, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BMP085 bmp;

void EventoWifi(WiFiEvent_t event)
{
    Serial.printf("[Evento-Wifi] evento: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi conectedo");
      Serial.println("IP : ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi se pedio la conexion...");
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 inicio cliente..");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 cliente conectado a AP");
      break;
    default:      
      Serial.println("Evento WiFi desconocido.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(EventoWifi);
  WiFi.begin(ssid, password);
}

void envia_mqtt(float t, float h, float hic, float pres, float qair){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando CIOR ThingsBoard node ...");
      if ( client.connect("Sensor", TOKEN_TB, NULL) ) {
        Serial.println( "[OK]" );
      } else {
        Serial.print( "[ERROR] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
        
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String presion = String(pres);
      String qaire = String(qair);
            
      String payload = "{";
      payload += "\"temperatura\":";
      payload += temperatura;
      payload += ",";
      payload += "\"humedad\":";
      payload += humedad;
      payload += ",";
      payload += "\"stermica\":";
      payload += stermica;
      payload += ",";
      payload += "\"presion\":";
      payload += presion;
      payload += ",";
      payload += "\"qaire\":";
      payload += qaire;
      payload += "}";

      char attributes[200];
      payload.toCharArray( attributes, 200 );
      int rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      Serial.print( "Atributos : ");
      Serial.println( attributes );
    }
  }
}

float qaire(){
  float qair=digitalRead(QAIR_B) * 2 + digitalRead(QAIR_A) ;
  return qair;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  pinMode(21,INPUT_PULLUP);
  pinMode(22,INPUT_PULLUP);
  pinMode(QAIR_A,INPUT);
  pinMode(QAIR_B,INPUT);             
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  delay(400);
  
  client.setServer(mqtt_server, 1884);
  
  dht.begin();
  if (!bmp.begin()) {
  Serial.println("No se encuentra sensor de Presion!");
  }
  Serial.println("Fin Setup()");
     
}

void loop() {
  // put your main code here, to run repeatedly:
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }else{
    
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  Serial.print("Pressure = ");
  float pres=bmp.readPressure();
  Serial.print(pres);
  Serial.println(" Pa");
  float qair=qaire();
  envia_mqtt(t,h,hic,pres,qair);
 }
 delay(60000); 
}

