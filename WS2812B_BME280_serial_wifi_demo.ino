/*
 * Credits / inspiracje i fragmenty kodu zaczerpniete z:
 * - serwer WWW: https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
 * - obsługa paskow LED WS2812B: https://github.com/adafruit/Adafruit_NeoPixel
 * - obsluga zintegrowanego czujnika BME280: https://github.com/adafruit/Adafruit_BME280_Library
 */


#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>

//enter SSID and password to your local Wi-Fi
const char* ssid = "esp";
const char* password = "haslo8266";

#define NEOPIXEL_PIN       12
#define NUMPIXELS          8    //enter the number of pixels (LEDs in your stripe)

#define BME_SDA            21
#define BME_SCL            22


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
TwoWire I2C_BME280 = TwoWire(1);

WiFiServer server(80);
String header;  //request will be stored here

unsigned long currentTime = millis();
unsigned long previousTime = 0; 
const long timeoutTime = 2000;

//temperature, pressure, and humidity are stored here
float T,P,H;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 250
uint8_t LED_r = 0;
uint8_t LED_g = 150;
uint8_t LED_B =1 ;

void startServer()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}


void processServer(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /3/red") >= 0) {
              Serial.println("Red color");
              LED_r = 150;
              LED_g = 0;
              LED_b = 0;
            } else if (header.indexOf("GET /3/green") >= 0) {
              Serial.println("Green color");
              LED_r = 0;
              LED_g = 150;
              LED_b = 0;
            } else if (header.indexOf("GET /3/blue") >= 0) {
              Serial.println("Blue color");
              LED_r = 0;
              LED_g = 0;
              LED_b = 150;
            }
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #C0C0C0; border: none; color: black; padding: 5px 5px;");
            client.println("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer;}");
            client.println(".buttonr {background-color: #EE0000;}");
            client.println(".buttong {background-color: #00EE00;}");
            client.println(".buttonb {background-color: #A0A0FF;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>Temperature " + String(T) + " deg. C</p>");
            client.println("<p>Humidity " + String(H) + " %RH</p>");
            client.println("<p>Pressure " + String(P) + " hPa</p>");
            // If the output26State is off, it displays the ON button       
            client.println("<p><a href=\"/3/red\"><button class=\"button buttonr\">RED</button></a></p>");
            client.println("<p><a href=\"/3/green\"><button class=\"button buttong\">GREEN</button></a></p>");
            client.println("<p><a href=\"/3/blue\"><button class=\"button buttonb\">BLUE</button></a></p>");

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}



void setup() {
  delay(2);
  Serial.begin(115200);
  delay(2);
  //Serial.begin(115200,SERIAL_8N1,U0RX,U0TX,false,1000);  //console
  while(!Serial);    // time to get serial running
  Serial.println("WS2812B, BME280, and Web server test");
  if( I2C_BME280.begin(BME_SDA,BME_SCL,100000) == false ) Serial.println("I2C_BME280.begin returned false"); else Serial.println("I2C_BME280.begin returned true");
  pixels.begin();

  unsigned status;
  
  status = bme.begin(0x76, &I2C_BME280);  //try 0x77 or 0x76
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      while (1) delay(10);
  }
  else
  {
    Serial.println("bme.begin returned true");
  }

  startServer();
  
}

int seqNb = 0;
unsigned long blinkCntr_last;
const unsigned long BLINK_TIME = 250;

void handleLedBlink()
{
    blinkCntr_last = millis();
    Serial.println("blink!");
    pixels.clear();
  
    if(seqNb<NUMPIXELS) {
      seqNb++;
    }
    else
    {
      seqNb = 0;
    }
    pixels.setPixelColor(seqNb, pixels.Color(LED_r, LED_g, LED_b));
    pixels.show();
}


void loop() {
  processServer();
  if( (millis() - blinkCntr_last) > BLINK_TIME )
  {
    handleLedBlink();
  
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
    T = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(T);
    Serial.println(" *C");
  
    Serial.print("Pressure = ");
  
    P=bme.readPressure() / 100.0F;
    Serial.print(P);
    Serial.println(" hPa");
  
    H=bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(H);
    Serial.println(" %");
  }
}
