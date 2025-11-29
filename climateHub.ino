#include <Wire.h>
#include "HT_SSD1306Wire.h"
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <WiFi.h>

// MUST MATCH SENSOR SETTINGS 
#define RF_FREQUENCY            915E6 // Hz  (same as sensor)
#define TX_OUTPUT_POWER         14     // dBm (for ACKs, if used)
#define LORA_BANDWIDTH          0     // [0: 125 kHz] most range
#define LORA_SPREADING_FACTOR   10    // [SF7..SF12] longer airtime = more range
#define LORA_CODINGRATE         1     // [1: 4/5,  (1-4)] lowest error correction setting

#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYMBOL_TIMEOUT     0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON    false

#define BUFFER_SIZE 256
#define RX_BUFFER_SIZE 256 //max bytes per packet
char rxBuffer[RX_BUFFER_SIZE]; //full space 


static RadioEvents_t radioEvents; //for lora to call when arrival


void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
//rssi is the signal strenth and snr is the signal to noise ratio



const char* SSID = "mac26";
const char* PASS = "12345678";

String sensor1Data = "";
String sensor2Data = "";




WiFiServer http(80);   // port 80, normal http


void setup()
{
  Serial.begin(115200);
  delay(500);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE); //set up board

  radioEvents.RxDone = OnRxDone; //packet received, call onrxdone function
  

  Radio.Init(&radioEvents); //initialize lora with radioEvents struct so it knows what to call
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  //lora settings from earlier, passing them in to configure receiving

  Radio.Rx(0); //now we listen to receive continuously




 //start Wi-Fi AP
  WiFi.mode(WIFI_AP); // Put ESP32 into AP mode
  WiFi.softAP(SSID,PASS);//start it

  IPAddress ip = WiFi.softAPIP(); //get ip address of AP
  Serial.println("WiFi started.");

  Serial.print("Open at: http://");
  Serial.print(ip);
  Serial.println("/s1");
  Serial.print("Open at: http://");
  Serial.print(ip);
  Serial.println("/s2");


  http.begin(); //start http server
  Serial.println("HTTP server started.");
} //end setup


void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Radio.Sleep(); //temp pause radio

  if (size >= RX_BUFFER_SIZE) size = RX_BUFFER_SIZE-1; //cant be larger than what we allocated
  
  memcpy(rxBuffer, payload, size);
  //now rxBuffer holds raw payload data and its size

  //DEBUGGING
  rxBuffer[size] = '\0'; //to print as string

  Serial.println(rxBuffer);
  Serial.print("Received (");
  Serial.print(size);
  Serial.print(" bytes), RSSI=");
  Serial.print(rssi);
  Serial.print(" dBm\nData:\n");
  Serial.println(rxBuffer);


  String msg = String(rxBuffer);

  //which sensor this came from, indexOf will return index where sensor 'num' starts
  if (msg.indexOf("Sensor 1") != -1) sensor1Data = msg;
  else if (msg.indexOf("Sensor 2") != -1) sensor2Data = msg;

  Radio.Rx(0); //back to listening

}



void loop()
{
  Radio.IrqProcess(); //continuously check for packets to call rx 

  // handle connected device
  WiFiClient client = http.available();
  if (!client) return; //reloop (still grab lora data)
  
  // Wait until http request is made from device 
  while (client.connected() && !client.available())
  {
    Radio.IrqProcess(); //still get packets
    delay(1);
  }
  

  //read first line of request (like "GET / HTTP/1.1")
  String requestLine = client.readStringUntil('\r');
  client.readStringUntil('\n'); // consume '\n'

  String body;
  // Check path in the request line
  if (requestLine.startsWith("GET /s1")) body = sensor1Data; //sensor1 url
  else if (requestLine.startsWith("GET /s2")) body = sensor2Data; //sensor2
  else
  {
    // default- show both at each update of both
    body += "Sensor 1:\n" + sensor1Data + "\n\n";
    body += "Sensor 2:\n" + sensor2Data + "\n";
  }


  // send minimal HTTP response with auto-refresh + payload
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();  // end of headers

  // auto-refresh + preformatted payload
  client.print("<meta http-equiv='refresh' content='3'>");
  client.print("<pre>");
  client.print(body);  
  client.print("</pre>");


  delay(10);
  client.stop();
}



