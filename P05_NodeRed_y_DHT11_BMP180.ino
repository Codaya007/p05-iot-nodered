#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

#define DHTPIN 5       // Pin para data DHT11
#define DHTTYPE DHT11  // Tipo de sensor DHT
#define LED_PIN 4      // Pin para LED

#define CHANNEL_LED "led"
#define CHANNEL_PARAMS "ambientales"

const char* ssid = "RED-ViVi";
const char* password = "Ladeantes";
// const char* ssid = "Internet_UNL";
// const char* password = "UNL1859WiFi";

// Datos del broker MQTT
const char* mqttServer = "20.82.114.131";  // IP del broker MQTT
const int mqttPort = 1883;                 // Puerto MQTT

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;

long timeLastPayloadSend = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// Función para conectarse a WiFi
void setup_wifi() {
  delay(10);
  // Conexión WiFi
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado al WiFi");
}

// Función de callback para cuando se recibe un mensaje del topic "led"
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el topic: ");
  Serial.println(topic);

  // Solo nos interesa el topic "led"
  if (strcmp(topic, "led") == 0) {
    // Analizar el mensaje recibido (por ejemplo, encender/apagar el LED)
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    Serial.println("Mensaje: " + message);

    if (message == "on") {
      // Encender el LED
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (message == "off") {
      // Apagar el LED
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

// Función para conectar al servidor MQTT
void reconnect() {
  // Intentar conectar mientras no esté conectado
  while (!client.connected()) {
    Serial.print("Conectando al MQTT...");

    // Intentar conectar al broker con un identificador único
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado al broker MQTT");

      // Suscribirse al canal "led" para recibir comandos
      client.subscribe("led");
    } else {
      Serial.print("Falló la conexión, rc=");
      Serial.print(client.state());
      Serial.println(" Volveré a intentar en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  // Inicializar puerto serie
  Serial.begin(9600);
  dht.begin();

  // Configuración del LED. Inicialmente apagado
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Conectarse al WiFi
  setup_wifi();

  // Configurar cliente MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // Valido que el sensor BMP esté conectado
  while (!bmp.begin()) {
    Serial.println("No se pudo encontrar el sensor BMP180.");
    delay(1000);
  }

  Serial.println("Sensor BMP180 configurado.");
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float p = bmp.readPressure();

  Serial.println("=====================================================");
  Serial.println("Temperatura: " + String(t));
  Serial.println("Humedad: " + String(h));
  Serial.println("Presión: " + String(p));

  // Conectar al broker MQTT si no está conectado
  if (!client.connected()) {
    reconnect();
  }

  // Procesar mensajes entrantes
  client.loop();

  delay(1000);

  long now = millis();

  if (now - timeLastPayloadSend > 3000) {
    timeLastPayloadSend = now;

    if (isnan(h) || isnan(t) || isnan(p)) {
      Serial.println("Error al leer los sensores");

      return;
    }

    // Convertir presión a hPa
    float pHPa = p / 100;

    Serial.println("Presión hPa: " + String(pHPa));
    Serial.println("=====================================================");


    // Formamos el JSON para enviarlo al canal
    String payload = String("{\"temp\":") + t + ",\"humidity\":" + h + ",\"press\":" + pHPa + "}";

    // Publicamos datos en broker
    boolean success = client.publish("ambientales", payload.c_str());

    if (success) {
      Serial.println("Datos enviados");
    } else {
      Serial.println("Error al enviar a MQTT");
    }
  }
}
