# IOT

https://espressif.github.io/arduino-esp32/package_esp32_index.json


#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ThingSpeak.h>

// Credenciais da Rede WiFi
const char* ssid = "linksys";  // REDE
const char* password = "";     // SENHA

// Configurações do Broker MQTT
const char* mqtt_server = "test.mosquitto.org"; // Broker MQTT
const int mqtt_port = 1883;

// Configurações do ThingSpeak
unsigned long myChannelNumber = 2914561; // Substitua com seu Canal ID
const char* myWriteAPIKey = "LTR9OSFKNU0ETJSH";

// Número do grupo (alterar conforme seu grupo)
const int GROUP_NUMBER = 1;

// Definição dos pinos
#define TRIG_PIN 27    // Pino de trigger do sensor ultrassônico
#define ECHO_PIN 33    // Pino de echo do sensor ultrassônico
#define TEMP_PIN 22    // Pino do sensor de temperatura
#define PUMP_PIN 23    // Pino que controla a bomba (LED)

// Limites de acionamento
const float LEVEL_THRESHOLD = 30.0; // Limite de nível em cm para ligar a bomba

// Variáveis globais
float localLevel = 0;      // Nível medido localmente
float localTemp = 0;       // Temperatura medida localmente
float avgLevel = 0;        // Média dos níveis recebidos
float avgTemp = 0;         // Média das temperaturas recebidas 
int numTempReadings = 0;   // Contador de leituras de temperatura
int numLevelReadings = 0;
float sumLevel = 0;        // Soma acumulada dos níveis
float sumTemp = 0;         // Soma acumulada das temperaturas

// Objetos para comunicação
WiFiClient espClient;                // Cliente WiFi
PubSubClient client(espClient);      // Cliente MQTT
OneWire oneWire(TEMP_PIN);           // Protocolo OneWire
DallasTemperature sensors(&oneWire); // Sensor de temperatura

// Função para conectar ao WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// Função de callback para mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  // Converte o payload para String
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  
  // Verifica se a mensagem é de nível e atualiza a média
  if (topicStr.equals("GRP" + String(GROUP_NUMBER) + "\\NIVEL")) {
    sumLevel += message.toFloat();
    numLevelReadings++;
    avgLevel = sumLevel / numLevelReadings;
  } 
  // Verifica se a mensagem é de temperatura e atualiza a média
  else if (topicStr.equals("GRP" + String(GROUP_NUMBER) + "\\TEMPERATURA")) {
    sumTemp += message.toFloat();
    numTempReadings++;
    avgTemp = sumTemp / numTempReadings;
  }
}

// Função para reconectar ao broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("conectado");
      
      // Inscreve nos tópicos de nível e temperatura
      client.subscribe(("GRP" + String(GROUP_NUMBER) + "\\NIVEL").c_str());
      client.subscribe(("GRP" + String(GROUP_NUMBER) + "\\TEMPERATURA").c_str());
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

// Função para ler o nível do sensor ultrassônico
float readUltrasonicLevel() {
  // Envia pulso para trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Mede o tempo de eco
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Converte para distância em cm
  float distance = duration * 0.034 / 2;
  
  return distance;
}

// Função para ler os sensores locais
void readLocalSensors() {
  // Lê a temperatura
  sensors.requestTemperatures();
  localTemp = sensors.getTempCByIndex(0);
  
  // Lê o nível
  localLevel = readUltrasonicLevel();
  
  Serial.print("Temperatura Local: ");
  Serial.print(localTemp);
  Serial.println(" °C");
  
  Serial.print("Nível Local: ");
  Serial.print(localLevel);
  Serial.println(" cm");
}

// Função para controlar a bomba baseado no nível
void controlPump() {
  if (localLevel > LEVEL_THRESHOLD) {
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("Bomba LIGADA - Nível acima do limite");
  } else {
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("Bomba DESLIGADA - Nível abaixo do limite");
  }
}

// Função para publicar dados dos sensores via MQTT
void publishSensorData() {
  String levelTopic = "GRP" + String(GROUP_NUMBER) + "\\NIVEL";
  String tempTopic = "GRP" + String(GROUP_NUMBER) + "\\TEMPERATURA";
  
  client.publish(levelTopic.c_str(), String(localLevel).c_str());
  client.publish(tempTopic.c_str(), String(localTemp).c_str());
  
  Serial.println("Dados dos sensores publicados no MQTT");
}

// Função para enviar dados ao ThingSpeak
void sendToThingSpeak() {
  // Define os campos no ThingSpeak
  ThingSpeak.setField(1, localTemp);    // Temperatura local
  ThingSpeak.setField(2, avgTemp);      // Temperatura média
  ThingSpeak.setField(3, localLevel);   // Nível local
  ThingSpeak.setField(4, avgLevel);     // Nível médio
  
  // Envia os dados
  int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
  if (response == 200) {
    Serial.println("Dados enviados ao ThingSpeak");
  } else {
    Serial.println("Problema ao enviar para ThingSpeak. Código HTTP " + String(response));
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configura os pinos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  // Inicializa os sensores
  sensors.begin();
  
  // Conecta ao WiFi
  setup_wifi();
  
  // Configura o MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Inicializa o ThingSpeak
  ThingSpeak.begin(espClient);
}

void loop() {
  // Verifica conexão MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Lê os sensores locais
  readLocalSensors();
  
  // Controla a bomba baseado no nível
  controlPump();
  
  // Publica dados no MQTT
  publishSensorData();
  
  // Envia dados ao ThingSpeak
  sendToThingSpeak();
  
  // Reinicia as médias a cada ciclo (para demonstração)
  sumLevel = 0;
  sumTemp = 0;
  numLevelReadings = 0;
  numTempReadings = 0;
  
  delay(15000); // Aguarda 15 segundos
}
