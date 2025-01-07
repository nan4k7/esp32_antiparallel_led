#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>


// Pin Definitions
#define LED_A 2
#define LED_B 3
#define LED_ALT 0
#define WIFI_SWITCH_PIN 1  // GPIO para el interruptor Wi-Fi
#define FORCEON_SWITCH_PIN -1 //GPIO para el interruptor de encendido forzado

unsigned long currentMillis;

// Timing Variables
unsigned long previousMillis = 0;
const long interval = 10;  // 150Hz = 6.67ms per cycle
bool cycle = false;
bool running = false;
bool firstRun = false;
bool altLedFlag = false;
bool forceOnFlag = false;
bool forceOn = false;
int currentHour = -1;
int currentMin = -1;

int startHourDef = 19;
int endHourDef = 4;
int pwmPowerDef = 5;

// Wi-Fi
const char* ssid = "Fibergarch 2.4";
const char* password = "Hipolito480";
const char* hostname = "esp32-led-candlelight";
bool wifiAlwaysOn = false;

//NTP
unsigned long lastNTPUpdate = 0;                 // Almacena el tiempo de la última actualización NTP
const unsigned long ntpUpdateInterval = 21600000; // 6 horas en milisegundos (6 * 3600 seg * 1000 ms)
const unsigned long ntpUpdateIntervalRetry = 30000;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", (-3 * 3600), (ntpUpdateInterval - 5000));  // UTC, syncs every (ntpUpdateInterval - 5s)
bool timeClientEnabled = false;


// Preferences for Storing Configuration
Preferences preferences;
int startHour;
int endHour;
int pwmPower;

// Web Server
AsyncWebServer server(80);
bool serverRunning = false;  // Estado del servidor HTTP

// Wi-Fi Control Functions
void connectWiFi() {
  int retryCount = 0;
  int maxRetries = 120;

  setCpuFrequencyMhz(80);

  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // Activa el modo de bajo consumo en Wi-Fi


  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi conectando");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      
      retryCount++;

      if (retryCount > maxRetries) {
        Serial.println("\nFallo en la conexión WiFi, reiniciando...");
        delay(1000);
        ESP.restart();
      }
    }

    Serial.print("\nWiFi conectado, ");
    Serial.print(WiFi.getHostname());
    Serial.print(" (");
    Serial.print(WiFi.localIP());
    Serial.println(")");
  }
}

void disconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi desconectado");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    setCpuFrequencyMhz(40);
  }
}

void initTimeClient() {
  if (!timeClientEnabled) {
    timeClient.begin();
    timeClientEnabled = true;

    Serial.println("TimeClient iniciado");
  }
}

void endTimeClient() {
  if (timeClientEnabled) {
    timeClient.end();
    timeClientEnabled = false;

    Serial.println("TimeClient finalizado");
  }
}

void startServer() {
  if (!serverRunning) {
    server.begin();
    serverRunning = true;

    Serial.println("Servidor HTTP iniciado");
  }
}

void stopServer() {
  if (serverRunning) {
    server.end();
    serverRunning = false;

    Serial.println("Servidor HTTP detenido");
  }
}

void updateLocalTime() {
  currentMin = timeClient.getMinutes();
  currentHour = timeClient.getHours();
}

void runCycle() {
  if (currentMillis - previousMillis >= interval) {
    analogWrite(LED_A, cycle ? (pwmPower * 2.55) : LOW);
    analogWrite(LED_B, cycle ? LOW : (pwmPower * 2.55));

    previousMillis = currentMillis;
    cycle = !cycle;
  }
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(80);

  if (LED_ALT >= 0)
    altLedFlag = true;
  if (FORCEON_SWITCH_PIN >= 0)
    forceOnFlag = true;

  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);
  
  if (forceOnFlag)
    pinMode(FORCEON_SWITCH_PIN, INPUT_PULLUP);

  analogWrite(LED_A, LOW);
  analogWrite(LED_B, LOW);
  
  if (altLedFlag) {
    pinMode(LED_ALT, OUTPUT);
    digitalWrite(LED_ALT, LOW);
  }

  preferences.begin("led-config", true);
  startHour = preferences.getInt("startHour", startHourDef);
  endHour = preferences.getInt("endHour", endHourDef);
  pwmPower = preferences.getInt("pwmPower", pwmPowerDef);
  preferences.end();

  btStop();

  delay(1000);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <body>
        <h2>LED On/Off Time Configuration</h2>
        <form action="/set-times" method="post">
          Start Hour: <input type="number" name="startHour" min="0" max="23" value=")rawliteral";
    html += String(startHour);
    html += R"rawliteral("><br>
          End Hour: <input type="number" name="endHour" min="0" max="23" value=")rawliteral";
    html += String(endHour);
    html += R"rawliteral("><br>
          Power: <input type="number" name="pwmPower" min="0" max="100" value=")rawliteral";
    html += String(pwmPower);
    html += R"rawliteral("><br>    
          <input type="submit" value="Save">
        </form>
      </body>
      </html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  server.on("/set-times", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("startHour", true) && request->hasParam("endHour", true)) {
      startHour = request->getParam("startHour", true)->value().toInt();
      endHour = request->getParam("endHour", true)->value().toInt();
      pwmPower = request->getParam("pwmPower", true)->value().toInt();

      preferences.begin("led-config", false);
      preferences.putInt("startHour", startHour);
      preferences.putInt("endHour", endHour);
      preferences.putInt("pwmPower", pwmPower);
      preferences.end();

        Serial.println("Configuración guardada");

        request->redirect("/");
    }
    else {
      request->send(400, "text/plain", "Invalid Input");
    }
  });

  Serial.println("Setup completado");
}

void loop() {
  currentMillis = millis();

  // Leer el estado del interruptor
  wifiAlwaysOn = digitalRead(WIFI_SWITCH_PIN) == HIGH;
  
  if (forceOnFlag)
    forceOn = digitalRead(FORCEON_SWITCH_PIN) == HIGH;    

  if ((!wifiAlwaysOn && (currentMillis - lastNTPUpdate >= ntpUpdateInterval || lastNTPUpdate == 0)) || wifiAlwaysOn) {
    connectWiFi();
    startServer();

    initTimeClient();
    timeClient.update();
    currentHour = timeClient.getHours();

    if ((currentMillis - lastNTPUpdate >= ntpUpdateInterval || lastNTPUpdate == 0)) {
      Serial.println("Hora NTP actualizada");
    }

    if (!wifiAlwaysOn) {
      disconnectWiFi();
      //endTimeClient();
      stopServer();
    }

    lastNTPUpdate = currentMillis;  // Actualizar el tiempo de la última sincronización
  }
  else {
    disconnectWiFi();
    //endTimeClient();
    stopServer();  // Asegurar que el servidor esté apagado si Wi-Fi está desconectado
  }

  //Serial.println(currentHour);

  bool mismoDia = startHour <= endHour;


  if (!timeClient.isTimeSet()) {
    analogWrite(LED_A, LOW);
    analogWrite(LED_B, LOW);

    if (altLedFlag) {
      digitalWrite(LED_ALT, LOW);
    }

    if (currentMillis - lastNTPUpdate >= ntpUpdateIntervalRetry) {
      Serial.println("NO TIME");

      timeClient.update();
      updateLocalTime();

      lastNTPUpdate = currentMillis;
    }
  }
  else {
    updateLocalTime();

    if (forceOn || (mismoDia && (currentHour >= startHour && currentHour < endHour)) || (!mismoDia && (currentHour >= startHour || currentHour < endHour))) {
      if (!running || !firstRun) {
        running = true;
        firstRun = true;

        Serial.println("LED encendido");
      }

      runCycle();

      if (altLedFlag) {
        digitalWrite(LED_ALT, HIGH);
      }      
    }
    else {
      analogWrite(LED_A, LOW);
      analogWrite(LED_B, LOW);

      if (altLedFlag) {
        digitalWrite(LED_ALT, LOW);
      }

      if (running || !firstRun) {
        running = false;
        firstRun = true;

        Serial.println("LED apagado fuera del rango de tiempo");

        if (!wifiAlwaysOn) {
          esp_sleep_enable_timer_wakeup(15 * 1000000);
          esp_light_sleep_start(); 
        }
      }
    }
  }
}