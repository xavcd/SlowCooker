#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

ESP8266WebServer httpd(80);
double tempMijoteuse = 25.00;
byte nombreDecimal = 2;

// Déclaration des méthodes
void ReadTemp();
void HandleFileRequest();
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



    Serial.println("Setup done!");
}

void loop() 
{
    //ReadTemp(); Uncomment when plugged into crockpot
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
    int input = analogRead(A0);
    double voltage = input * 5.0 / 1023.0;
    double resistance = 10000.0 * voltage / (5.0 - voltage);
    double tempKelvin = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3977.0); // B (beta) = 3977
    tempMijoteuse = tempKelvin - 273.15;
}