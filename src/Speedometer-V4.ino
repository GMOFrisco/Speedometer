// Messwagen kompakt für die Modellbahn
// basierend auf dem Sketch "Fidget Spinner counter and tachometer" von
// 27.08.2017 Pawel A. Hernik
// source code for the video:
// https://youtu.be/42qNfPOYlR8

// Und auch aus Anregungen aus dem Stummi Forum
// und selbstverständlich eigene Bedürfnisse
/*
 Teile:
 - Hall sensor 3144
 - Minimagnet auf Radachse Magnet
 - OLED SSD3106 von Alibaba
 - ESP8266 von Alibaba  (genaue Bezeichnung Wemos D1 mini)

 3144 pinout from front:
 1 - VCC
 2 - GND
 3 - DATA (0 when close to magnet)

            Caboose      The Chiller     Southern
 drad       9,59mm        10,45mm       9,45mm        10,4mm
 Umfang     30,1278mm   32,8296mm     29,688mm        32,672mm
 Massstab   1:87      1:87        1:87
 Es gibt eine Messung pro Umdrehung die dann an den PC gesendet wird
 Die Zeitdauer wird mit dem µs Timer berechnet, das gibt ausreichend Genaugkeit
 Für die Messwerte wird double als Zahlenformat verwendet um die entsprechende Auflösung zu erhalten
 Optimierungen für eine bessere Funktion:
 1. Ersten Meßwert nach Stillstand bzw. Connecting verwerfen. (kann man auch im PC)
 2. Meßwertfilterung oder Mittelwertbildung (ebenfalls im PC) = realisiert, dynamisch
 3. Einstellungen von der Gegenseite ermöglichen, z.B. Raddurchmesser ( man könnte auch nur die Zetdauer schicken, dann ist
                                      aber keine Anzeige mehr im Wagen möglich als Standalone)
 4. Messung der zurückgelegten Wegstrecke, bereits integriert und funktioniert
 5. Einsatz eines Zugkraftmesswagens zusätzlich. Mit Arduino ok Umsetzung hier in Arbeit


 Optionen:
 - Über den Eingang D7 kann man einstellen ob nur gemessen und angezeigt wird.
   legt man den Eingang D7 auf GND, dann überspringt die Software alles was mit Wlan zu tun hat.
   Es wird dann nur im Display die Scale Miles per Hour angezeigt. Und seit kurzem auch der zurückgelegte Weg.

 Die PC Software wird noch ständig weiterentwickelt. Da stehen folgende Optionen an:
 - Senden des Raddurchmessers
 - Lesen der Decoderpro Dateien
 - Schreiben der Decoderpro Dateien
 - Steuern der Lok über Loconet (Locobuffer). Das Protokoll ist noch nicht vollständig verstanden.
 - Umschreiben Lokadresse, Consistadresse.


 */
#define USEHW 1  // 0 - use fast softi2c, 1 - use hw implementation on A4/A5

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdint.h>

#include "OLED_SoftI2C.h"
#include "term8x14_font.h"
//#include "Speed_8.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 		//https://github.com/tzapu/WiFiManager

#include <EEPROM.h>
#include <HX711.h>

HX711 scale;//(D0, D5);

// initial passwordless AP name
char apname[] = "Messwagen";      // to connect take 192.168.4.1 in a Browser

//char pc_ip[] = "192.168.0.42";
int port = 2055;

// zeit um eingestellten AP zu probieren in Sekunden
const int TRY_AP_CONNECT_SECONDS = 180;
// Zeit labview server zu probieren in Sekunden
// 0=unendlich
const int TRY_LABVIEW_CONNECT_SECONDS = 180;


//WiFiClient client;
WiFiServer server(port);
//WifiClient* client_global; // TODO: for sharing with other tasks later
boolean alreadyConnected = false; // whether or not the client was connected previously

const int ledPin = D4;             // Build in LED zur Kontrolle
const int nurDisplay  = D6;         // Eingang für Anwahl nur Disply ohne PC und WiFi
const int ledblink = D8;           // Blinkled zur Kontrolle ob Firmware läuft
const int hallPin = D7;            // pin 3 = int 0

const double Radumfang = 32.609;   // Radumfang der Messachse

OLEDSoftI2C oled(0x3c);
char txt[10];
char gettext[20];

// --------------------------------------------------------------------------
volatile unsigned long print_timer; 		// Zeitgeber um die Werte über die serielle Schnittstelle auszugeben
volatile unsigned long timeout_timer; 		// Zeitgeber für Message bei Fahrzeugstillstand
volatile unsigned long blink_timer;     // Blinktimer für Funktionsled
volatile unsigned long cntTime = 0;
volatile unsigned long cnt = 0;
volatile unsigned long deltamicro = 0; 		// Zeit zwischen zwei Flanken bei jeder fallenden Flanke von Pin2
volatile unsigned long microalt = 0; 		// Alter Wert von micro um Zeit zu messen bei jeder Flanke von Pin2
//volatile double scalefactor = (87 * 30.128);  	// Maßstab 1:87 mal Radumfang   Caboose
//volatile double scalefactor = (87 * 32.8296);     // Maßstab 1:87 mal Radumfang The Chiller
//volatile double scalefactor = (87 * 29.688);  	// Maßstab 1:87 mal Radumfang   Southern
volatile double scalefactor = (87 * 32.609);     // Maßstab 1:87 mal Radumfang QOCX 193 RBL
volatile double v_m_s = 0;                        // Meter/Sekunde
volatile double v_km_h = 0;                       // Kilometer/Stunde
volatile double v_smi_h = 0;                      // Scalemiles/Stunde
volatile double length = 0;                       // gefahrene Strecke in Meter seit start
volatile int wifi_send_bit = 0;                   // Wird in der Interuptroutine gesetzt
volatile int ledBit = 0;                          // wird auch in der Interuptroutine gesetzt
volatile int toggle_bit = 0;                      // wechselt in jeder IR
volatile int mitWifi = 0;                         // Variable für nur Messen
volatile int blinki = 0;                          // Variable zur Blinkled Status
char str_temp[32];

volatile double force = 0;
volatile double forceav = 0;
volatile double forceunit = 0;

ICACHE_RAM_ATTR void doCount_ISR()    				     // interrupt wird durch steigende Flanke ausgelöst an Pin 2
{
  cnt++;                      			               // Erhöhe den Zähler um 1
  cntTime = micros();                       	     // hole aktuellen Zeitwert
  deltamicro = cntTime - microalt;        	       // Berechne die Zeitspanne zwischen zwei Pulsen
  microalt = cntTime;               		           // aktueller Zeitwert wird alter Wert für die nächste Berechnung
  v_m_s = (1000 * scalefactor) / deltamicro;       // Geschwindigkeit in scale m/s
  v_km_h = v_m_s * 3.600;                   	     // Geschwindigkeit in scale km/h
  v_smi_h = v_km_h * 0.621371;              	     // Geschwindigkeit in Scalemiles / h
  length = (cnt * Radumfang) / 1000;              // Gefahrene Strecke in Meter
  digitalWrite(ledPin, HIGH);                    	 // Kontroll LED für Interrupt erreicht
  wifi_send_bit = 1;
}

void doForceSetup()
{
  oled.clrScr();                  			// Display löschen
  oled.printStr(0, 0, "Speedometer");       		// Überschrift
  oled.printStr(0, 2, "Force Setup");       		// Überschrift

    wdt_disable();
  //-----
    scale.set_scale(980.f);     // this value is obtained by calibrating the scale with known weights; see the README for details
    scale.tare();				        // reset the scale to 0
    force = scale.read();			// raw reading from the ADC
    forceav = scale.read_average(20);  	// average of 20 readings from the ADC
    forceunit = scale.get_units(5);
    //forceunit = (scale.get_units(5), 1);	// average 5 readings from the ADC minus tare weight (not set) divided					                              // by the SCALE parameter (not set yet)
  //_____
    wdt_enable(1000);
    oled.clrScr();                  			// Display löschen
    oled.printStr(0, 0, "Speedometer");       		// Überschrift
    oled.printStr(0, 2, "Setup done");       		// Überschrift

}

void oled_print_ip()
{
  String ip_str = WiFi.localIP().toString();

  oled.printStr(0, 0, "IP address:\n");       // Auf das Dispaly Zeile 1
  oled.printStr(0, 4, (char*) (ip_str+"\n").c_str());  // Anzeigen auf Display
  oled.printStr(0, 6, "\n");
}

void oled_print_headlines()
{
  oled.clrScr();                  			// Display löschen
  oled.printStr(0, 0, "Speedometer");       		// Überschrift
  print_timer = millis(); // Der Print Timer ist für die Seriellausgabe gedacht
  timeout_timer = millis();
  blink_timer = millis();   // Timer um Blinken einer LED an D7 zu steuern
}


/*void receiveTask()
{
  while(1);
   {
     String gettext = client_global->readStringUntil('\n');
     oled.printStr(0, 2, (char*)gettext.c_str());
   }
}*/

void doReceive(WiFiClient* rcv_client)
{
  String gettext = rcv_client->readStringUntil('\n');
  oled.printStr(0, 2, (char*)gettext.c_str());

//  scalefactor = (gettext, 4);
}
// --------------------------------------------------------------------------

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());

  oled.printStr(0, 0, "\n");
  oled.printStr(0, 2, "Access point:\n");       // Auf das Display Zeile 2
  oled.printStr(0, 4, (char*)(myWiFiManager->getConfigPortalSSID()).c_str());  // Anzeigen auf Display
  oled.printStr(0, 6, "\n");
}

void setup() {
  scale.begin(D0,D5);

  Serial.begin(115200);
  Serial.println("Hello");
  oled.init();
  oled.clrScr();
  oled.setFont(Term8x14PL);
  oled.printStr(0, 2, "Hello\n");

  pinMode(hallPin, INPUT_PULLUP);
  digitalWrite(hallPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(hallPin), doCount_ISR, FALLING); // hall pin on interrupt 0 = pin 7
  digitalWrite(hallPin, HIGH);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(ledblink, OUTPUT);
  pinMode(nurDisplay, INPUT_PULLUP);
  mitWifi = digitalRead(nurDisplay);
  digitalWrite(ledblink, HIGH);

  //wdt_disable();

  if (mitWifi == 0) {
    oled.printStr(0, 0, "Speedometer");       // Überschrift
    oled.printStr(0, 2, "kein Wifi");
    doForceSetup();
  }


  if (mitWifi == 1) {

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  wifiManager.setAPCallback(configModeCallback);

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect(apname)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  server.begin();
  Serial.print("Connected to wifi. My address: ");
  IPAddress myAddress = WiFi.localIP();
  Serial.println(myAddress.toString());
  oled_print_ip();

  Serial.println("Wait for Connection from LabView");
 }

 /*xTaskCreate(receiveTask,      // Task function.
             "receive",        // String with name of task.
             256,              // Stack size in bytes.
             NULL,             // Parameter passed as input of the task
             1,                // Priority of the task.
             NULL);            // Task handle. */

  doForceSetup();

}

// --------------------------------------------------------------------------
void loop() {
//  ESP.wdtFeed();
if (mitWifi == 1) {       // Nur weiter hier wenn D6 auf 1 sonst nur Messen und Anzeigen
      // wait for a client - at least one needs to be connected!
      static WiFiClient client;

  if(client != NULL && client.connected()) // START
  {
      if (print_timer <= (millis() - 9000))         // alle 9 Sekunden wird der Bildschirm gelöscht
        {
          //Serial.begin(115200);
          //Serial.println(print_timer);
          //Serial.println(toggle_bit);
          oled.printStr(0, 0, "Speedometer\n");       // Überschrift
          //oled.setMinCharWd(8);
          print_timer = millis();               // Print Timer neu setzen
        }
        if (toggle_bit == 0)
        {
          oled.printStr(110, 0, "o");
          } else {
          oled.printStr(110, 0, "x");
        }

      // zuerst geht es zur Parameterausgabe an die Serielle Schnittstelle

    //    Serial.println(v_m_s);
        digitalWrite(ledPin, LOW);            // Led wieder aus, wurde in der ISR eingeschaltet
        ledBit = 0;

        if (wifi_send_bit == 1) {             // hier werden die Messungen an Wifi gesendet
            wifi_send_bit = 0;                // Format =
            char sendstring[50] = "";           // SPEED:sm/s;skm/h;smi/h;Strecke;Zugkraft CRLF
            strcat(sendstring, "SPEED:");       // SPEED
            dtostrf(v_m_s, 5, 3, str_temp);
            strcat(sendstring, str_temp);       // m/s
            strcat(sendstring, ";");
            dtostrf(v_km_h, 5, 3, str_temp);
            strcat(sendstring, str_temp);       // km/h
            strcat(sendstring, ";");
            dtostrf(v_smi_h, 5, 3, str_temp);
            strcat(sendstring, str_temp);       // mi/h
            strcat(sendstring, ";");
            dtostrf(length, 5, 3, str_temp);
            strcat(sendstring, str_temp);       // Strecke in m
            strcat(sendstring, ";");
            dtostrf(forceunit, 5, 3, str_temp);
            strcat(sendstring, str_temp);       // Zugkraft in g
            strcat(sendstring, "\r\n");
//
            client.print(sendstring);         // Schreiben an PC
//            client_global = &client;
//            doReceive(&client);               // Lesen vom PC


            timeout_timer = millis();           // Timeouttimer wieder zurücksetzen
            if (toggle_bit == 0)
            {
              toggle_bit = 1;
            } else  {
              toggle_bit = 0;
            }
          } else {
              if (timeout_timer <= (millis() - 12000))
              {                                   // wenn der Wagen steht oder <0,5 sm/h fährt, dann nach jeweils 12 Sekunden "No-Data" schicken
                char sendstring[50] = "";         // String  Array setzen
                strcat(sendstring, "No-Data");        // Erzeuge "No-Data"
                strcat(sendstring, "\r\n");         // und ein abschließendes CR als Ende Kennung
                client.print(sendstring);         // nun in den Äther damit
              }
          }

            //forceunit = (scale.get_units(5), 1);
            if (scale.is_ready()) {
                forceunit = scale.get_units(1); // average of 3 readings
                dtostrf(forceunit, 10, 3, str_temp);
                oled.printStr(0, 2, (char*)(String(str_temp)+" g\n").c_str());           // kg

            }
            //dtostrf(v_m_s, 8, 3, str_temp);         // Ausgabe der Messwerte auf Display
            //oled.printStr(0, 2, (char*)(String(str_temp)+" sm/sec\n").c_str());          // m/s
            dtostrf(length, 10, 3, str_temp);        //
            oled.printStr(0, 4, (char*)(String(str_temp)+" m\n").c_str());              // m
            //dtostrf(v_km_h, 8, 3, str_temp);        //
            //oled.printStr(0, 4, (char*)(String(str_temp)+" skm/h\n").c_str());          // km/h
            dtostrf(v_smi_h, 8, 3, str_temp);       //
            oled.printStr(0, 6, (char*)(String(str_temp)+" smi/h\n").c_str());          // mi/h
                              // Fertig
  } else {
    client = server.available();
    if(client) {
      oled_print_headlines();
    } else {
      oled_print_ip();
      delay(500);
    }
  }
// Ab hier wenn "kein WiFi" es wird nur gemessen und angezeigt
} else {
    if (wifi_send_bit == 1) {             // Neue Daten sind da
    wifi_send_bit = 0;
    timeout_timer = millis();
    if (toggle_bit == 0) {
      toggle_bit = 1;
      } else  {
        toggle_bit = 0;
      }
    }
    if (toggle_bit == 0)
    {
      oled.printStr(110, 0, "o");
    } else {
      oled.printStr(110, 0, "x");
    }
//  dtostrf(v_m_s, 8, 3, str_temp);         // Ausgabe der Messwerte auf Display
//  oled.printStr(0, 2, (char*)(String(str_temp)+" sm/sec\n").c_str());          // m/s
//  dtostrf(v_km_h, 8, 3, str_temp);        //
//  oled.printStr(0, 4, (char*)(String(str_temp)+" skm/h\n").c_str());          // km/h
    dtostrf(length, 10, 3, str_temp);        //
    oled.printStr(0, 4, (char*)(String(str_temp)+" m\n").c_str());          // m
    dtostrf(v_smi_h, 8, 3, str_temp);       //
    oled.printStr(0, 6, (char*)(String(str_temp)+" smi/h\n").c_str());          // mi/h


    //forceunit = (scale.get_units(5), 1);
    if (scale.is_ready()) {
        forceunit = scale.get_units(1); // average of 3 readings
        dtostrf(forceunit, 10, 3, str_temp);
        oled.printStr(0, 2, (char*)(String(str_temp)+" g\n").c_str());           // kg

    //    Serial.printf("read:\t %s\n", str_temp);
    }

    if (blink_timer <= (millis() - 1000))
    {
        if (blinki == 0)
        {
          digitalWrite(ledblink, HIGH);
          blinki = 1;
        } else {
          digitalWrite(ledblink, LOW);
          blinki = 0;
        }
        blink_timer = millis();
    }
  } // ENDE von "kein Wifi"
}
// Fertig, es läuft jetzt so wie vorgesehen
