#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <PID_v1.h>

ESP8266WebServer httpd(80);

double targetTemp, tempMijoteuse, heatPercent;
double Kp = 25.00, Ki = 0.00, Kd = 43.00;
unsigned long lastTempRead = 0ul, lastPIDCycle = 0ul;
byte nombreDecimal = 2;
bool currentHeatCycleHandled = false;

PID PIDcontroller(&tempMijoteuse, &heatPercent, &targetTemp, Kp, Ki, Kd, DIRECT);

const int maxHeatPercent = 100;
unsigned long windowStartTime, expectedHeatCycle = 0ul;

// Déclaration des méthodes
void ReadTemp();
void HandleFileRequest();
void ComputePID();
String GetContentType(String);


void setup()
{
    Serial.begin(115200);
    Serial.println("Creation du AP...");

    WiFi.softAP("Xavcd", "Toto123!");
    
    Serial.println(WiFi.softAPIP());

    LittleFS.begin();

    httpd.on("/getTemperature", HTTP_GET, []() {
        String response = "{\"temperature\": " + String(tempMijoteuse, nombreDecimal) + "}";
        httpd.send(200, "application/json", response);
    });
    httpd.onNotFound(HandleFileRequest);
    httpd.begin();

    // Initialiser le PID
    windowStartTime = millis();
    PIDcontroller.SetOutputLimits(0, maxHeatPercent);
    PIDcontroller.SetMode(AUTOMATIC);
    PIDcontroller.SetSampleTime(1250);

    pinMode(D1, OUTPUT);
    digitalWrite(D1, LOW);
    targetTemp = 43.60;

    Serial.println("Setup done!");
}

void loop() 
{
    // Lire la température
    ReadTemp(); 

    // Kill switch si la température est trop proche de 50
    if (tempMijoteuse > 49.00)
    {
        digitalWrite(D1, LOW); // Désactiver le relais de chauffage.
        Serial.println("Turned killswitch on. Press reset button to restart.");
        while(true) {}
    }

    ComputePID();

    httpd.handleClient(); // Appeler le plus souvent possible
}

String GetContentType(String filename)
{
    struct Mime
    {
        String extension, type;
    } mimeType[] = {
        {".html", "text/html"}};

    for (unsigned int i = 0; i < sizeof(mimeType) / sizeof(Mime); i++)
    {
        if (filename.endsWith(mimeType[i].extension))
            return mimeType[i].type;
    }

    return "application/octet-stream";
}

void HandleFileRequest()
{
  String filename = httpd.uri();

  if (filename.endsWith("/"))
    filename = "index.html";

  if (LittleFS.exists(filename))
  {
    File file = LittleFS.open(filename, "r");
    httpd.streamFile(file, GetContentType(filename));
    file.close();
  }
  else
    httpd.send(404, "text/plain", "404 : Not Found");
}

void ReadTemp() 
{
    // Éviter de lire la température trop souvent pour ne pas faire de conflit avec le wifi.
    if (millis() - lastTempRead > 100ul)  
    {
        lastTempRead = millis();
        int input = analogRead(A0);
        double voltage = input * 5.0 / 1023.0;
        double resistance = 10000.0 * voltage / (5.0 - voltage);
        double tempKelvin = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3977.0);
        tempMijoteuse = tempKelvin - 273.15; // Température en celsius
    }
}

void ComputePID()
{
    PIDcontroller.Compute();
    
    if ((millis() - lastPIDCycle <= expectedHeatCycle) && !currentHeatCycleHandled)
    {
        currentHeatCycleHandled = true;
        if (digitalRead(D1) == LOW)
        {
            digitalWrite(D1, HIGH); // Activer l'élément chauffant.
            // Serial.println("Turned relay on."); // Debug
        }
    }
    else if ((millis() - lastPIDCycle >= expectedHeatCycle) && heatPercent != 100.00 && currentHeatCycleHandled)
    {
        currentHeatCycleHandled = false;
        if (digitalRead(D1) == HIGH)
        {
            digitalWrite(D1, LOW); // Désactiver l'élément chauffant.
            // Serial.println("Turned relay off."); // Debug
        }
    }

    if (millis() - lastPIDCycle >= 1000ul)
    {
        lastPIDCycle = millis();
        expectedHeatCycle = heatPercent * 10ul;
        Serial.print("Température courante : ");
        Serial.print(tempMijoteuse);
        Serial.print("\t\t Pourcentage de chauffage : ");
        Serial.println(heatPercent);
        /* Simulation très simple pour tester le PID Controller
        tempMijoteuse += 5 * (heatPercent / 100);
        if (heatPercent == 0.00)
            tempMijoteuse -= 0.5;
        */
    }
}