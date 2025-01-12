#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include <NTPClient.h>
#include <AsyncTCP.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <Preferences.h>


// Pin Definitions
#define LED_A 2
#define LED_B 3
#define LED_ALT 0
#define WIFI_SWITCH_PIN 1      // GPIO para el interruptor Wi-Fi
#define FORCEON_SWITCH_PIN -1  //GPIO para el interruptor de encendido forzado
#define POTPOWER_PIN -1        // GPIO para el potenciómetro (-1 para desactivar)
#define DEVNAME "esp32-led-candlelight"

#define WIFI_SSID "Fibergarch 2.4"
#define WIFI_PASS "Hipolito480"


unsigned long currentMillis;
unsigned long otaProgMillis = 0;
unsigned long previousMillis = 0;

const long interval = 10;  // 150Hz = 6.67ms per cycle
bool cycle = false;
bool running = false;
bool firstRun = false;
bool altLedFlag = false;
bool forceOnFlag = false;
bool potPinFlag = false;
bool forceOn = false;
int currentHour = -1;
int currentMin = -1;

int startHourDef = 19;
int endHourDef = 4;
int pwmPowerDef = 5;

// Wi-Fi
bool wifiAlwaysOn = false;

// NTP
unsigned long lastNTPUpdate = 0;                   // Almacena el tiempo de la última actualización NTP
const unsigned long ntpUpdateInterval = 21600000;  // 6 horas en milisegundos
const unsigned long ntpUpdateIntervalRetry = 30000;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", (-3 * 3600), (ntpUpdateInterval - 5000));
bool timeClientEnabled = false;

// Preferences
Preferences preferences;
int startHour;
int endHour;
int pwmPower;

// Web Server
AsyncWebServer server(80);
bool serverRunning = false;  // Estado del servidor HTTP
bool otaRunning = false;

// Wi-Fi Control Functions
void connectWiFi() {
  int retryCount = 0;
  int maxRetries = 120;

  setCpuFrequencyMhz(80);

  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(DEVNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // Activa el modo de bajo consumo en Wi-Fi

  const int LISTEN_INTERVAL = 10;

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = WIFI_SSID,
      .password = WIFI_PASS,
      .listen_interval = LISTEN_INTERVAL,
    },
  };

  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  esp_wifi_set_max_tx_power(40);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi conectando");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

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

void startOTA() {
  if (!otaRunning) {
    ElegantOTA.begin(&server);  // Start ElegantOTA
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    otaRunning = true;

    Serial.println("Servidor OTA iniciado");
  }
}

void stopOTA() {
  if (otaRunning) {
    otaRunning = false;

    Serial.println("Servidor OTA detenido");
  }
}

void onOTAStart() {
  Serial.println("OTA update iniciado!");
}

void onOTAProgress(size_t current, size_t final) {
  if (millis() - otaProgMillis > 1000) {
    otaProgMillis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finalizado successfully!");
  } else {
    Serial.println("Error durante el update OTA!");
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

void updatePWMPowerFromPot() {
  if (potPinFlag) {
    int potValue = analogRead(POTPOWER_PIN) / 10;  // Leer valor (0-4095)

    int pwmPowerTmp = map(potValue, 0, 409, 0, 100);  // Escalar a 0-100

    const int threshold = 5;

    if (abs(pwmPower - pwmPowerTmp) >= threshold) {
      pwmPower = pwmPowerTmp;
      preferences.putInt("pwmPower", pwmPower);

      Serial.println("Potencia LED: " + String(pwmPower) + "%");
    }
  }
}

void runCycle() {
  updatePWMPowerFromPot();

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

  if (LED_ALT >= 0) altLedFlag = true;
  if (FORCEON_SWITCH_PIN >= 0) forceOnFlag = true;
  if (POTPOWER_PIN >= 0) potPinFlag = true;

  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);

  if (forceOnFlag) pinMode(FORCEON_SWITCH_PIN, INPUT_PULLUP);
  if (potPinFlag) pinMode(POTPOWER_PIN, INPUT);

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
          Power: <input type="number" name="pwmPower" min="0" max="100" )rawliteral";

    // Usamos una variable para el atributo disabled
    html += (potPinFlag ? R"rawliteral(disabled="disabled")rawliteral" : "");
    html += R"rawliteral( value=")rawliteral";
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

      preferences.begin("led-config", false);
      preferences.putInt("startHour", startHour);
      preferences.putInt("endHour", endHour);

      if (!potPinFlag) {
        pwmPower = request->getParam("pwmPower", true)->value().toInt();
        preferences.putInt("pwmPower", pwmPower);
      }

      preferences.end();

      Serial.println("Configuración guardada");

      request->redirect("/");
    } else {
      request->send(400, "text/plain", "Invalid Input");
    }
  });
  Serial.println("Setup completado");
}

void loop() {
  currentMillis = millis();

  wifiAlwaysOn = digitalRead(WIFI_SWITCH_PIN) == HIGH;
  if (forceOnFlag) forceOn = digitalRead(FORCEON_SWITCH_PIN) == HIGH;

  if ((!wifiAlwaysOn && (currentMillis - lastNTPUpdate >= ntpUpdateInterval || lastNTPUpdate == 0)) || wifiAlwaysOn) {
    connectWiFi();
    startOTA();
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
      stopOTA();
      stopServer();
    }

    lastNTPUpdate = currentMillis;  // Actualizar el tiempo de la última sincronización
  } else {
    disconnectWiFi();
    //endTimeClient();
    stopOTA();
    stopServer();  // Asegurar que el servidor esté apagado si Wi-Fi está desconectado
  }

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
  } else {
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
    } else {
      if (running || !firstRun) {
        running = false;
        firstRun = true;

        analogWrite(LED_A, LOW);
        analogWrite(LED_B, LOW);

        if (altLedFlag) {
          digitalWrite(LED_ALT, LOW);
        }

        Serial.println("LED apagado fuera del rango de tiempo");
      }

      if (!wifiAlwaysOn) {
        esp_sleep_enable_timer_wakeup(10 * 1000000);
        esp_light_sleep_start();
      }
    }
  }

  ElegantOTA.loop();
}