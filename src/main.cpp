#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

ESP8266WebServer httpd(80);
double tempMijoteuse = 0.00;
byte decimalAmount = 2;
String htmlRoot = "";

// Déclaration des méthodes
void HandleRoot();
void ReadTemp();

void setup()
{
  Serial.begin(115200);
  Serial.println("Creation du AP...");
  WiFi.softAP("xavcd", "Toto123!");

  Serial.println(WiFi.softAPIP());

  httpd.on("/", HandleRoot);
  httpd.begin();
}

void loop() 
{
    ReadTemp();
    httpd.handleClient(); // Appeler le plus souvent possible
}

void HandleRoot()
{
    htmlRoot =  "<html>" +
    htmlRoot +=     "<body>" +
    htmlRoot +=         "Temperature: " + String(tempMijoteuse, decimalAmount) + 
    htmlRoot +=     "</body>" +
    htmlRoot += "</html>";
  
    httpd.send(200, "text/html", htmlRoot);
}

void ReadTemp() 
{
    int input = analogRead(A0);
    double voltage = input * 5.0 / 1023.0;
    double resistance = 10000.0 * voltage / (5.0 - voltage);
    double tempKelvin = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3977.0); // B (beta) = 3977
    tempMijoteuse = tempKelvin - 273.15;
}