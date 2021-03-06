/*
  Library for NO2 and CO Sensor reading.
  Created by Marcel Belledin, October 27, 2016.
*/

#include <SPI.h>
#include <WiFi101.h>
#include <FlashStorage.h>
#include "config.h"

// feather m0 config
#define en A0
#define pre A1
#define vred A2
#define vnox A3
#define in_type INPUT
int resolution = 1023; // resolution steps of the ADC


// ap storage config
typedef struct {
  boolean valid;
  char ssid_ap[100];
  char pass_ap[100];
} ap;

FlashStorage(my_ap, ap);

// Create a "Access Point" variable and call it "owner"
ap owner;

// local ap config
char ssid[] = "isGenerated"; // created AP name
char pass[] = "1234567890"; // AP password (needed only for WEP, must be exactly 10 or 26 characters in length)
String ssid_ap;
String pass_ap;
int keyIndex = 0; // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

// data push config
const int port = 80;
const char* host = "influx.datacolonia.de";
char data[256];
char board[7];
char api_key[13];
bool ap_connectable = false;
bool ap_created = false;

// influxdb
bool pingable = false;
WiFiClient influx_client;

// sensor config
float board_volt = 3.3; // input voltage
float vout_ox = 0; // output voltage
float vout_co = 0; // output voltage
float r5 = 3900; // RLOAD_OX in Ohm
float r7 = 360000; // RLOAD_CO in Ohm


// vars for ppm calculation
float ratio_ox = 0;
float ppm_ox = 0;
float ratio_co = 0;
float ppm_co = 0;
float r0_ox = 900; // assumed resistance in fresh air... to be calibrated
float r0_co = 200; // assumed resistance in fresh air... to be calibrated
float r_ox = 0; // calculated resistance
float r_co = 0; // calculated resistance
int a_ox;
int a_co;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  WiFi.setPins(8,7,4,2);
  WiFi.maxLowPowerMode(); // go into power save mode
  //sensor:
  pinMode(pre, OUTPUT);
  pinMode(en, OUTPUT);
  // Read the content of "my_ap" into the "owner" variable
  owner = my_ap.read();
  Serial.println(owner.ssid_ap);

  if (owner.valid) {
    connect_ap();
  }else{
    create_ap();
  }
}

void create_ap() {
  Serial.println("Access Point Web Server");
  WiFi.end();

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(ssid, "%02X%02X%02X%02X%02X%02X",mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 5 seconds for connection:
  delay(5000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWifiStatus();
  ap_created = true;
}

void get_new_ssid() {
 // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    bool new_owner = false;
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<html><h1>OpenAir Sensor Config</h1><form method='get' action='setting'><label>SSID (Name des Hotspots): </label><input name='ssid' length=32><label>Passwort: </label><input name='pass' length=64><input type='submit'></form>");
            client.print("Hinweis: Wir ben&ouml;tigen den WLAN Zugang, um die Sensormessungen an unsere Datenbank zu senden. Nachdem das Ger&auml;t die Daten empfangen hat, testet es die Verbindung. ");
            client.print("Ihr Browser zeigt dabei eine Fehlerseite...Wenn die Daten nach wenigen Minuten unter <a href=\"http://openair.codingcologne.de\">openair.codingcologne.de</a> nicht erscheinen, melden sie sich. (siehe Anleitung)");
            client.print("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.startsWith("GET /setting")) {
          if (currentLine.endsWith(" HTTP")) {
            String s=currentLine;
            int firstIndex = s.indexOf("ssid=");
            int secondIndex = s.indexOf('&', firstIndex+1);
            int thirdIndex = s.indexOf("pass=", secondIndex+1);
            int endIndex = s.indexOf(" HTTP", thirdIndex+1);
            ssid_ap = escapeParameter(s.substring(firstIndex+5, secondIndex));
            pass_ap = escapeParameter(s.substring(thirdIndex+5, endIndex)); // To the end of the string
            Serial.println("params:");
            Serial.println(ssid_ap);

            // Fill the "owner" structure with the data entered by the user...
            owner.valid = true;
            ssid_ap.toCharArray(owner.ssid_ap, 100);
            pass_ap.toCharArray(owner.pass_ap, 100);

            // ...and finally save everything into "my_ap"
            my_ap.write(owner);
            ap_created = false;
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<html><h1>Wir versuchen zu verbinden...</h1>");
            client.print("Die Verbindung wird gleich getrennt... <a href=\"http://192.168.1.1\"> Falls keine Daten auf openair.datacolonia.de erscheinen die Anleitung und diesem Link folgen...</a>");
            client.print("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            client.stop();
            delay(10000);
            connect_ap();
          }
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println(ip);
}

void connect_ap() {
  WiFi.end();
  int attempts = 0;
  while ( (status != WL_CONNECTED) && (attempts < 5) ) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(owner.ssid_ap);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(owner.ssid_ap, owner.pass_ap);
    attempts += 1;
    // wait 5 seconds for connection:
    delay(5000);
  }
  printWifiStatus();
  //
  if (attempts > 4) {
    ap_connectable = false;
    WiFi.end();
    create_ap();
  }else {
    ap_connectable = true;
    pingable = true; // is assumed
  }
}

char* get_board() {
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(board, "%02X%02X%02X", mac[2], mac[1], mac[0]);
  return board;
}

char* get_api_key() {
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(api_key, "%02X%02X%02X%02X%02X%02X",mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  return api_key;
}

void send_to_influx() {

  Serial.println(data);
  if (!influx_client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  // send request to the server
  influx_client.print("POST /write?db=open_air&u="+String(USER)+"&p="+String(PASS)+ " HTTP/1.1\r\n" +
        "Host: " + host + "\r\n" +
        "Content-Type: application/x-www-form-urlencoded\r\n");
  influx_client.print("Content-Length: ");
  influx_client.println(sizeof(data), DEC);
  influx_client.println("Connection: close\r\n");

  influx_client.write(data, sizeof(data));

  delay(1);

  unsigned long timeout = millis();
  while (influx_client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      influx_client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while(influx_client.available()){
    String line = influx_client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("closing connection");
}

float get_ox_resistance() {
  a_ox = analogRead(vnox);
  vout_ox = (board_volt / resolution) * a_ox; // Calculates the Voltage
  r_ox = ((board_volt - vout_ox) * r5)/vout_ox; // Calculates the resistance
  return isnan(r_ox)?-1:r_ox;
}

float get_co_resistance() {
  a_co = analogRead(vred);
  vout_co = (board_volt / resolution) * a_co; // Calculates the Voltage
  r_co = ((board_volt - vout_co) * r7)/vout_co; // Calculates the resistance
  return isnan(r_co)?-1:r_co;
}
String escapeParameter(String param) {
  param.replace("+", " ");
  param.replace("%21", "!");
  param.replace("%23", "#");
  param.replace("%24", "$");
  param.replace("%26", "&");
  param.replace("%27", "'");
  param.replace("%28", "(");
  param.replace("%29", ")");
  param.replace("%2A", "*");
  param.replace("%2B", "+");
  param.replace("%2C", ",");
  param.replace("%2F", "/");
  param.replace("%3A", ":");
  param.replace("%3B", ";");
  param.replace("%3D", "=");
  param.replace("%3F", "?");
  param.replace("%40", "@");
  param.replace("%5B", "[");
  param.replace("%5D", "]");

  return param;
}

void loop() {
  if (pingable) {
    data[0] = (char)0;
    digitalWrite(en, HIGH);
    delay(10000);
    // read the value from the sensor:
    digitalWrite(pre, 1);
    delay(30000);
    Serial.println(get_ox_resistance());
    Serial.println(get_co_resistance());
    digitalWrite(pre, 0);
    // influxDB data format: key,tag_key=param field=param
    snprintf(data, sizeof data,
      "r_no,id=%s,mac=%s value=%.2f \n"
      "sens_no,id=%s,mac=%s value=%d \n"
      "r_co,id=%s,mac=%s value=%.2f \n"
      "sens_co,id=%s,mac=%s value=%d \n",
      get_board(), get_api_key(), r_ox,
      get_board(), get_api_key(), a_ox,
      get_board(), get_api_key(), r_co,
      get_board(), get_api_key(), a_co);

    send_to_influx();
    WiFi.maxLowPowerMode(); // go into power save mode
    delay(60000*15);
  } else {
    if ((ap_connectable == false) && (ap_created == true)) {
      get_new_ssid();
    }
  }
}