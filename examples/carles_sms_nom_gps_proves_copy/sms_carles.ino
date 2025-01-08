/**
 * @file      SendSMS.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-11-16
 * @note      Not support T-SIM7672
 * *  Although the manual of SIM7672G states that it has the functions of making voice calls and sending text messages, 
 * *  the current firmware does not support it.
 */

#include "utilities.h"
#include <esp_adc_cal.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "utils.h"

#define INTERRUPTOR_PIN 32 

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

//define SMS_TARGET  "+34622090133"
const char* ssid = "Banyoles-Servidor";
const char* password = "11112222";


WiFiServer server(80);

#define TINY_GSM_USE_GPRS true
// Your GPRS credentials, if any
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
const char serverFirebase[]   = "34.140.81.157";
const char resource[]         = "/submit_data01.php";
const int  port               = 80;

#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif

int flag = 0;

String header;
String deviceName = "Cap nom assignat";

struct gpsMessage{
  String numberGPS;
  String valorLat;
  String valorLon;
};



// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define SMS_TARGET_ONE  "+34622090133" //Change the number you want to send sms message
#define SMS_TARGET_TWO  "+34669123526"
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
HttpClient    http(client, serverFirebase, port);


bool sms_enviat = false;
bool despert = false;
bool llum = false;
const int voltageLimit = 100;
bool primerLlum = false;
int startOff = 0;

unsigned long previousMillisLCD = 0;
const long intervalLCD = 10000;

unsigned long previousMillisThingspeak = 0;
const long intervalThingspeak = 180000;

bool res_one = false;
bool res_two = false;
bool res_three = false;
bool res_fourth = false;
uint16_t battery_voltage;
char buf[256];




gpsMessage loopGPS()
{
    Serial.println("=========================");
    Serial.print("Set GPS Mode : ");
    Serial.println(1);

    //modem.setGPSMode(1); // modificat últim

    GPSInfo info;
    gpsMessage message;

    Serial.println("Requesting current GPS/GNSS/GLONASS location");

    for (;;) {
        if (modem.getGPS_Ex(info)) {

            Serial.println();

            Serial.print("FixMode:");
            Serial.println(info.isFix);

            Serial.print("Latitude:");
            Serial.print(info.latitude, 7);
            if (info.latitude > 0){
                Serial.println(" N");
            } else {
                Serial.println(" S");
            }
            Serial.print("Longitude:");
            Serial.print(info.longitude, 7);
            if (info.longitude > 0){
                Serial.println(" E");
            } else {
                Serial.println(" W");
            }
            Serial.print("Speed:");
            Serial.println(info.speed);
            Serial.print("Altitude:");
            Serial.println(info.altitude);

            Serial.println("Visible Satellites:");
            Serial.print(" GPS Satellites:");
            Serial.println(info.gps_satellite_num);
            Serial.print(" BEIDOU Satellites:");
            Serial.println(info.beidou_satellite_num);
            Serial.print(" GLONASS Satellites:");
            Serial.println(info.glonass_satellite_num);
            Serial.print(" GALILEO Satellites:");
            Serial.println(info.galileo_satellite_num);

            Serial.println("Date Time:");
            Serial.print("Year:");
            Serial.print(info.year);
            Serial.print(" Month:");
            Serial.print(info.month);
            Serial.print(" Day:");
            Serial.println(info.day);
            Serial.print("Hour:");
            Serial.print(info.hour);
            Serial.print(" Minute:");
            Serial.print(info.minute);
            Serial.print(" Second:");
            Serial.println(info.second);

            Serial.print("Course:");
            Serial.println(info.course);
            Serial.print("PDOP:");
            Serial.println(info.PDOP);
            Serial.print("HDOP:");
            Serial.println(info.HDOP);
            Serial.print("VDOP:");
            Serial.println(info.VDOP);

            /* String gps_raw = modem.getGPSraw();
            Serial.print("GPS/GNSS Based Location String:");
            Serial.println(gps_raw); */

            message.numberGPS = String(info.gps_satellite_num);
            message.valorLat  = String(info.latitude, 7) + (info.latitude > 0 ? "N" : "S");
            message.valorLon = String(info.longitude, 7) + (info.longitude > 0 ? "E" : "W");

            break;
        } else {
            Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
            Serial.print(".");
            delay(15000L);
        }
    }
    return message;
}





void setup()
{
  Serial.begin(115200);
  pinMode(INTERRUPTOR_PIN, INPUT_PULLUP);

  if (digitalRead(INTERRUPTOR_PIN) == LOW){
    flag = 1;
  }
  
#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

    // Set modem reset pin ,reset modem
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  // Turn on modem
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);


  // Set ring pin input
  // pinMode(MODEM_RING_PIN, INPUT_PULLUP); es pot eliminar si no es reben trucades

  // Set modem baud
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  Serial.println("Start modem...");
  delay(3000);

  

  while (!modem.testAT()) {
      delay(10);
  }

   // Wait PB DONE
  delay(10000);

  
  if (!SPIFFS.begin(true)){
      Serial.println("Error muntant SPIFFS!");
      return;
    }

  File file = SPIFFS.open("/nom_dispositiu.txt", "r");
  if (!file) {
    File newFile = SPIFFS.open("/nom_dispositiu.txt", "w");
    if (newFile) {
      newFile.println(deviceName);
      newFile.close();
    }
  } else {
    deviceName = file.readString();
    deviceName.trim();
    file.close();
  }

  
  if (flag == 1){
     WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    server.begin();

  } else if (flag == 0){
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    battery_voltage = esp_adc_cal_raw_to_voltage(analogRead(BOARD_BAT_ADC_PIN), &adc_chars) * 2;
    snprintf(buf, 256, "Battery:%umV", battery_voltage);
    

    Serial.print("Init success, start to send message to  ");
    Serial.println("Nom del dispositiu: " + deviceName);
    Serial.println(SMS_TARGET_ONE);
    Serial.println(SMS_TARGET_TWO);
    //String imei = modem.getIMEI();
    //res_one = modem.sendSMS(SMS_TARGET_ONE, deviceName + " V:" +  String(battery_voltage));
    //res_two = modem.sendSMS(SMS_TARGET_TWO, deviceName + " V:" +  String(battery_voltage));
    res_one = true;
    res_two = true;

    Serial.print("Send sms_one message ");
    Serial.println(res_one ? "OK" : "fail");
    Serial.print("Send sms_two message ");
    Serial.println(res_two ? "OK" : "fail");


    // Mirar de canviar el modem.
    DBG("Initializing modem...");
    if (!modem.init()) {
        DBG("Failed to restart modem, delaying 10s and retrying");
        return;
    }

    Serial.print("Waiting for network...");
    while (!modem.waitForNetwork()){
      Serial.print(".");
      delay(1000);
    }
    Serial.println("Network connected");
    

    
    
#if TINY_GSM_USE_GPRS
    // GPRS connection parameters are usually set after network registration
    Serial.print(F("Connecting to "));
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" success");

    if (modem.isGprsConnected()) {
        Serial.println("GPRS connected");
        Serial.println("Checking network mode...");
        int value = modem.getNetworkMode();
        Serial.print("value: ");
        Serial.println(value);
        String values = modem.getNetworkModes();
        Serial.print("values: ");
        Serial.println(values);

        /* modem.sendAT("+CNSMOD?");
        delay(100); */
    }
#endif

    Serial.println("Enabling GPS/GNSS/GLONASS");
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
        Serial.print(".");
    }
    Serial.println();
    Serial.println("GPS Enabled");
    modem.setGPSMode(1); // modificat últim

    // Set GPS Baud to 115200
    modem.setGPSBaud(115200);

  }
}



void loop()
{
  if (flag == 1){
    Serial.println("Has clicat el botó per canviar el nom");
    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                            // If a new client connects,
      Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            if (currentLine.length() == 0) {  // Two newline characters in a row
              // HTTP response
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Handle GPIO state requests
              if (header.indexOf("GET /update-name?name=") >= 0) {
                int startIndex = header.indexOf("name=") + 5;
                int endIndex = header.indexOf(" ", startIndex);
                if (endIndex == -1) endIndex = header.length();
                String newName = header.substring(startIndex, endIndex);
                newName.trim(); // Remove spaces
                newName = decodeURIComponent(newName);
                deviceName = newName; // Update the global variable
                Serial.println("Nou nom rebut: " + deviceName);

                File file = SPIFFS.open("/nom_dispositiu.txt", "w");
                if (file) {
                  file.println(deviceName);
                  file.close();
                } else {
                  Serial.println("Error obrint el fitxer per desar el nom");
                }
              }

              // Web Page Content
              client.println("<!DOCTYPE html><html>");
              client.println("<head>");
              client.println("<meta charset=\"UTF-8\">"); // Afegeix la codificació UTF-8
              client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              client.println("<style>");
              client.println("html, body { height: 100%; margin: 0; display: flex; justify-content: center; font-family: Helvetica; text-align: center;}");
              client.println(".container { display: flex; flex-direction: column; align-items: center; margin-top: 20px; }");  // Afegit margin-top
              client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
              client.println("text-decoration: none; font-size: 30px; margin: 10px; cursor: pointer;}");
              client.println("input[type=text] { width: 200px; height: 40px; font-size: 16px; margin-bottom: 20px; }");
              client.println(".form-container { display: block; margin-top: 20px; }");  // Contenidor per separar elements verticalment
              client.println("</style></head>");
              client.println("<body>");
              client.println("<div class=\"container\">");
              client.println("<h1>Banyoles Servidor</h1>");
              
              // Mostrar nom actual i formulari per actualitzar-lo
              client.println("<p><h2>Nom actual: " + deviceName + "</h2></p>");

              client.println("<div class=\"form-container\">");
              client.println("<input type=\"text\" id=\"newName\" placeholder=\"Nou nom...\" required>");
              client.println("<button class=\"button\" onclick=\"sendName()\">Canviar nom</button>");
              client.println("</div>");
              
              client.println("</div>");

              // JavaScript per gestionar l'actualització del nom
              client.println("<script>");
              client.println("function sendName() {");
              client.println("  const newName = document.getElementById('newName').value;");
              client.println("  if (newName) {");
              client.println("    const xhr = new XMLHttpRequest();");
              client.println("    xhr.open('GET', `/update-name?name=${encodeURIComponent(newName)}`, true);");
              client.println("    xhr.send();");
              client.println("    xhr.onload = function () {");
              client.println("      if (xhr.status === 200) {");
              client.println("        alert('Nom actualitzat: ' + newName);");
              client.println("        location.reload();");
              client.println("      } else {");
              client.println("        alert('Error enviant el nom.');");
              client.println("      }");
              client.println("    };");
              client.println("  } else {");
              client.println("    alert('Si us plau, introdueix un nom.');");
              client.println("  }");
              client.println("}");
              client.println("</script>");

              client.println("</body></html>");
              client.println();
              break;
            } else {
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }
        }
      }
      header = "";
      client.stop();
      Serial.println("Client disconnected.");
    }   
  } else {
    Serial.println("S'envien els missatges al mòbil");
    if (!res_one){
    //connectModem();
      if (testAT()){  
        int rssi = checkCSQ();
        Serial.print("rssi: " + String(rssi));
        if (rssi > 9 && rssi < 33){
          Serial.println("Bona senyal");
          //bool resPrimer = modem.sendSMS(SMS_TARGET_ONE, String("A5 atrapat - V: ") +  String(battery_voltage));
          bool resPrimer = modem.sendSMS(SMS_TARGET_ONE, deviceName +  String(battery_voltage));
          if (resPrimer){
            Serial.println("sms enviat amb èxit");
            res_one = true;
            //primerLlum = true;
            //modem.poweroff();
            delay(3000);
          } else {
          Serial.println("sms no enviat amb èxit");
          }
        }
      }
    }
    if (!res_two){
      if (testAT()){  
        int rssi = checkCSQ();
        Serial.print("rssi: " + String(rssi));
        if (rssi > 9 && rssi < 33){
          Serial.println("Bona senyal");
          //bool resSegon = modem.sendSMS(SMS_TARGET_TWO, String("A5 atrapat - V: ") +  String(battery_voltage));
          bool resSegon = modem.sendSMS(SMS_TARGET_ONE, deviceName +  String(battery_voltage));
          if (resSegon){
            Serial.println("sms enviat amb èxit");
            res_two = true;
            //primerLlum = true;
            //modem.poweroff();
            delay(3000);
          } else {
          Serial.println("sms no enviat amb èxit");
          }
        }
      }
    }
    unsigned long startTime = millis();
    gpsMessage testGPS = loopGPS();
    Serial.print("Number GPS: ");
    Serial.println(testGPS.numberGPS);
    Serial.print("Latitude: ");
    Serial.println(testGPS.valorLat);
    Serial.print("Longitud: ");
    Serial.println(testGPS.valorLon);
    unsigned long endTime = millis();
    unsigned long timeGetGPS = endTime - startTime;
    Serial.print("Has trigat tant mil·lisegons");
    Serial.println(timeGetGPS);

    String data1 = String(testGPS.valorLat);    // Latitud
    String data2 = String(testGPS.valorLon);    // Longitud
    String data3 = deviceName;                  // Nom del dispositiu
    String data4 = String(battery_voltage);
    
    // acabat de posar per enviar les dades a Firebase.
    if (modem.isGprsConnected()) {
      Serial.println("Enviant dades a Firebase...");
      String url = "http://34.140.81.157//submit_data01.php?data1=" + data1 + "&data2=" + data2 + "&data3=" + data3 + "&data4=" + String(1);

      http.beginRequest();
      http.get(url);
      http.endRequest();
  
      int statusCode = http.responseStatusCode();
      String response = http.responseBody();

      Serial.print("Codi de resposta de Firebase: ");
      Serial.println(statusCode);
      Serial.print("Resposta: ");
      Serial.println(response);

      if (statusCode == 200) {
        Serial.println("Dades enviades correctament a Firebase.");
      } else {
        Serial.println("Error enviant les dades a Firebase.");
      }
    } else {
      Serial.println("GPRS no connectat, no es poden enviar dades a Firebase.");
    }

    if (res_one && res_two) {
      Serial.println("Has enviat els dos missatge i hauries d'entrar en poweroff");
      delay(5000);
      modem.poweroff();
      Serial.println("Apagant el mòdem...");
      delay(2000);
      Serial.println("Entrant en mode somni profund");
      esp_deep_sleep_start();
    }
  }
  delay(1000);
}




int checkCSQ(){
  unsigned long startTime = millis();
  debugger.print("AT+CSQ\r\n");
  long int timeout = millis() + 3000;
  while (debugger.available() == 0){
    if (millis() > timeout){
      Serial.println("Temps espera esgotat");
      return -1; // retorna el valor -1 de la funció
    }
  }
  String response = debugger.readString();
  unsigned long endTime = millis();
  unsigned long responseTime = endTime - startTime;
  int index = response.indexOf("+CSQ:");
  Serial.print("Temps de respost CSQ: ");
  Serial.print(responseTime);
  Serial.println(" ms");
  Serial.print("Index: ");
  Serial.println(index);
  if (index != -1){
    String rssiString = response.substring(index + 6  , index + 8);
    Serial.print("rssiString: ");
    Serial.println(rssiString);
    int rssi = rssiString.toInt();
    return rssi;
  } else {
    return -1;
  }
}

/**bool sendSMS(String value){
  //String imei = modem.getIMEI();
  bool res = modem.sendSMS(SMS_TARGET_ONE, value);

  Serial.print("Send sms message ");
  Serial.println(res ? "OK" : "fail");
  if (!res){
    Serial.print("SMS no enviat");
    return false;
  } else {
    return true;
  }
}**/

/**void actualitzarLCD(float voltage){
  //float voltage = voltageSensor.getRmsVoltage();
  lcd.setCursor(0,1);
  lcd.print("V:");
  lcd.setCursor(3,1);
  lcd.print(voltage);
  Serial.print("Voltage: ");
  Serial.println(voltage);
}**/
/** 
void connectModem(){
  Serial.println("Connectant mòdem");
  //digitalWrite(BOARD_PWRKEY_PIN, LOW);
  //delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(13000);
}
*/

bool testAT(){
  Serial.println("Testejant AT command");
  SerialAT.println("AT");
  delay(1000);
  String response = "";
  while (SerialAT.available()) {
    char c = SerialAT.read();
    response += c;
  }

  // Mostrar la resposta per a debugging
  Serial.println("Resposta del mòdem: " + response);

  // Comprovar si la resposta conté "OK"
  if (response.indexOf("OK") != -1) {
    return true;  // La resposta conté "OK", el test és exitós
  } else {
    return false; // La resposta no conté "OK", el test ha fallat
  }
}