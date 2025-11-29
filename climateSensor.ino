
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "MQ135.h"

// Initialize the OLED display
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#include "Arduino.h"
#include "LoRaWan_APP.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define TX_BUFFER_SIZE 256
char txBuffer[TX_BUFFER_SIZE]; // data transmission memory space

#define MQPIN 5
#define DHTPIN 6
#define DHTTYPE    DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);
MQ135 sensorMQ = MQ135(MQPIN);

// MATCH HUB
#define RF_FREQUENCY            915E6 // Hz  (same as hub)
#define TX_OUTPUT_POWER         14     // dBm (for ACKs)
#define LORA_BANDWIDTH          0     // [0: 125 kHz] most range
#define LORA_SPREADING_FACTOR   10    // [SF7..SF12] longer airtime = more range
#define LORA_CODINGRATE         1     // [1: 4/5,  (1-4)] lowest error correction setting

#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYMBOL_TIMEOUT     0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON    false



static RadioEvents_t radioEvents;  //tell LoRa which functions to call from below

void OnTxDone(void);
void OnTxTimeout(void);



String buildPayload(String fah, String celcius, String humidity, String conc, String status, String time)
{
  String payload = "";

  payload += "---------------Sensor 1--------------- \n";

  //fahrenheit line
  payload += fah + " (F)\n";

  // Celsius
  payload += celcius + " (C)\n";

  // humidity
  payload += humidity + " % humidity\n";

  // ppm gas conc
  payload += "Gas conc (ppm): " + conc + "\n";

  // status string
  payload += status + "\n";

  // Time
  payload += "time (s): " + time + "\n";

  payload += "------------------------------------ \n";
  return payload;
}




uint32_t delayMS;

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}



void OnTxDone(void) {
  Serial.println("LoRa TX done");
  Radio.Sleep();   //sleep until next send
}

void OnTxTimeout(void) {
  Serial.println("LoRa TX timeout");
  Radio.Sleep();
}



void setup() {
  Serial.begin(115200);
  
  dht.begin(); // Initialize device.
  delay(50);


  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  //set TX callbacks
  //radioEvents.TxDone = OnTxDone;
  //radioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&radioEvents); //initialize lora with radioEvents struct so it knows what to call
  Radio.SetChannel(RF_FREQUENCY);

  // Configure as transmitter
  Radio.SetTxConfig(
    MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,    
    LORA_FIX_LENGTH_PAYLOAD_ON,true,0, 0, LORA_IQ_INVERSION_ON, 3000 // 3000 ms timeout
  );



  sensor_t sensor;
  dht.temperature().getSensor(&sensor);// Print temperature sensor details.
  dht.humidity().getSensor(&sensor);// Print humidity sensor details.
  delayMS = sensor.min_delay / 1000;// Set delay between sensor readings based on sensor details.


  VextON();
  delay(50);


  display.init();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.display();
}


void loop() {

  Radio.IrqProcess();   // let LoRa run

  display.clear();

  // reading time from start in seconds
  long time = millis() / 1000;
  display.drawString(0,50, "time (s): ");
  display.drawString(45,50, String(time));

  // Delay between measurements.
  delay(delayMS);


  float ppm = sensorMQ.getPPM(); //get air quality readings
  // parts per million (so 'x' molecules per 1 million particles sensed)


  float ppmPercent = 0.01*ppm;  //this means 500 ppm = 5%
  String ppmString = String(ppmPercent) + " %";

  Serial.print("Gas Conc (ppm): ");
  Serial.println(ppm);
  Serial.println(ppmString);  

  display.drawString(0,30, "Gas Conc (ppm): ");
  display.drawString(80,30, ppmString);

  String status = "";
  if (ppm >= 0 && ppm <= 1200)   status = "(SAFE)";
  else if (ppm > 1200 && ppm <=2000)  status = "(MILD)";
  else if (ppm > 2000 && ppm <= 100000)  status = "(DANGER)";
  else  status = "(ERROR)";
  display.drawString(65,40,status);

  // Get temperature event and print its value.
  sensors_event_t event;
  static float fah=0.0, celc=0.0, humidity=0.0;  


  dht.temperature().getEvent(&event);

  if (isnan(event.temperature)) Serial.println(F("Error reading temperature!"));
  
  else 
  {
    celc = event.temperature;
    Serial.print(F("Temperature (C): "));
    Serial.print(celc);
    Serial.println(F("°C"));

    Serial.print(F("Temperature (F): "));
    fah = ((9.0/5.0)*event.temperature) +32;
    Serial.print(fah);
    Serial.println(F("°F"));


    display.drawString(0,0, String(fah));
    display.drawString(40,0, "(F)");

    display.drawString(0,10, String(event.temperature));
    display.drawString(40,10, "(C)");
    display.display();
  }


  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);

  if (isnan(event.relative_humidity)) Serial.println(F("Error reading humidity!"));

  else
  {
    humidity = event.relative_humidity;
    Serial.print(F("Humidity: "));
    Serial.print(String(humidity)); 
    Serial.println(F("%  humidity"));
    Serial.println("");


    display.drawString(0,20, String(event.relative_humidity));
    display.drawString(40,20, "%  humidity");
    display.display();

  }


  String payload =  buildPayload(String(fah), String(celc), String(humidity), ppmString, status, String(time));


  static unsigned long lastSend = 0;
  unsigned long timeFrame = millis();

  if (timeFrame - lastSend >= 5000 && payload.length() < TX_BUFFER_SIZE) //if 5sec later and fits, send 
  {
    lastSend = timeFrame;

    Serial.println("LoRa TX payload:");
    Serial.println(payload);

    payload.toCharArray(txBuffer, TX_BUFFER_SIZE);  //put payload into buffer space
    Radio.Send((uint8_t*)txBuffer, strlen(txBuffer)); //send over lora
  }
 


} //end loop



