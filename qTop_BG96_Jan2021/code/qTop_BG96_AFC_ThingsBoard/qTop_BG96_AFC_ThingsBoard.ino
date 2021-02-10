/*****************************************************************************
  This is the Demo of using qTop LTE BG96 shield 
  plugged into Feather ESP32 board with ThingsBoard IOT Dashboard.

  Hardware Setup:
    Adafruit Feather ESP32 board;
    qTop LTE BG96 shield (IBT-QTC-AFC-BG96) with LTE and GNSS antennas connected;
    qJam BLE280 sensor module (IBT-QJS-BME280-0);
    Nano SIM Card;
    LiPol Battery.

====================================================================================================
Revision History:

Author                          Date                Revision NUmber          Description of Changes
------------------------------------------------------------------------------------

iotbotscom                02/09/2021               1.0.0                        Initial release
iotbotscom                02/10/2021               1.0.1                        Set "Always On" Mode


*****************************************************************************/

// Modem type
#define TINY_GSM_MODEM_BG96
#include <TinyGsmClient.h>

#include <ArduinoJson.h>
#include <Adafruit_BME280.h>

///////////////////////////////////////////////////////////////////////////////
// Demo modes
#define MODE_ALWAYS_ON      1
#define MODE_ONE_SHOT       2

// Publish delay timeout for "Always On" mode
#define MODE_ALWAYS_ON_DELAY_TIMEOUT  5000 // 5s

// Publish delay (Device Sleep) timeout for "Always On" mode
#define MODE_ONE_SHOT_DELAY_TIMEOUT   60000 // 60s

// Modem HW Pins
#define MODEM_PWR_ON_PIN    13
#define MODEM_ON_PIN        32

// GNSS Defs
#define GNSS_FIELDLEN_MAX  32

// Modem Serial
#if (defined(ESP32))
// ESP32 specific
HardwareSerial serialGSM(2);
#else
// AVR specific
HardwareSerial &serialGSM = Serial1;
#endif

// Uncomment to see StreamDebugger output in serial monitor
#define DUMP_AT_COMMANDS 1

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(serialGSM, Serial);
  TinyGsm modem(debugger);
#else
  // Initialize GSM modem
  TinyGsm modem(serialGSM);
#endif

// Initialize GSM client
TinyGsmClient client(modem);

// BME280 sensor
Adafruit_BME280 bme280;

// GPRS credentials
const char apn[]  = "wholesale";
const char user[] = "";
const char pass[] = "";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
#define CLOUD_TOKEN   "Your Token"
#define CLOUD_SERVER  "thingsboard.cloud"
#define CLOUD_PORT    80

// JSON & HTTP
DynamicJsonDocument jsonbuf(1024);
String reqtstr = "";

// Working Mode
int demo_mode = MODE_ALWAYS_ON;
//int demo_mode = MODE_ONE_SHOT;

// Devices status
bool is_modem_on = false;
bool is_sensor_on = false;
bool is_gnss_ready = false;

// Device GSM Data
int rssi = 0;

// Device Sensor Data
float temperature = 0;
float humidity = 0;
float pressure = 0;
float int_battery = 0;
float ext_battery = 0;

// Device GNSS Data
char quality[1+1];
char gnss_date[6+1];
char gnss_time[12+1];
char latitude[11+1];
char longitude[12+1];
char altitude[25+1];
char hdop[4+1];
char nsat[2+1];
char speed[25+1];
char heading[3+1];

///////////////////////////////////////////////////////////////////////////////
void setup() {

  // Set console baud rate
  Serial.begin(115200);

  Serial.print("\r\n***************************************************************************************************\r\n");
  Serial.print("\r\n **** IOT-BOTS.COM **** \r\n");
  Serial.print("\r\n **** qTop Quectel BG96 Shield ThingsBoard Demo **** \r\n");
  Serial.print("\r\n***************************************************************************************************\r\n");

  Serial.println("\r\n---- Setup Started ----\r\n");

  if (!bme280.begin(BME280_ADDRESS_ALTERNATE)) {
    Serial.println("No BME280 Sensor Found!");
    is_sensor_on = false;
  }
  else {
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_OFF);

    Serial.println("BME280 Sensor Found!");
    is_sensor_on = true;
  }

  pinMode(MODEM_PWR_ON_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_ON_PIN, LOW);

  pinMode(MODEM_ON_PIN, OUTPUT);
  digitalWrite(MODEM_ON_PIN, LOW);

  delay(3000);

  // Modem On Begin
  digitalWrite(MODEM_PWR_ON_PIN, HIGH);

  delay(1000);

  digitalWrite(MODEM_ON_PIN, HIGH);

  delay(1000);

  digitalWrite(MODEM_ON_PIN, LOW);
  // Modem On End

  delay(1000);

  // Set GSM module baud rate
  serialGSM.begin(115200);

  /**/
  Serial.println("\r\n Modem Initializing...");
  if (modem.begin()) {

    // GNSS On
    modem.enableGPS();

    // Get Modem Info
    String modemInfo = modem.getModemInfo();
    Serial.print("Modem: ");
    Serial.println(modemInfo);

    String imei = modem.getIMEI();
    Serial.print("IMEI: ");
    Serial.println(imei);

    String imsi = modem.getIMSI();
    Serial.print("IMSI: ");
    Serial.println(imsi);

    String ccid = modem.getSimCCID();
    Serial.print("CCID: ");
    Serial.println(ccid);

    is_modem_on = true;
  }
  else {
    Serial.println("No Modem Found");
    is_modem_on = false;
  }

  Serial.println("\r\n---- Setup Done ----");
}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
  Serial.println("\r\n---- Loop Cycle ----\r\n");

  if (is_modem_on != true) {
    Serial.println("No Modem Found");
    delay(1000);
    return;
  }

  /* Connect to GSM Network */
  if (!modem.isNetworkConnected()) {
    Serial.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    
    if (modem.isNetworkConnected()) {
    Serial.println(" : Network connected");
    }
  }

  /* Connect to GPRS */
  if (!modem.isGprsConnected()) {
    Serial.print("Connecting to \"");
    Serial.print(apn);
    Serial.print("\"...");
    if (!modem.gprsConnect(apn, user, pass)) {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    
    if (modem.isGprsConnected()) {
      Serial.println(" : GPRS connected");
    }
  }

  /* Connect to Cloud */
  if (!client.connected()) {
  Serial.print("Connecting to \"");
    Serial.print(CLOUD_SERVER);
    Serial.print("\"...");
    if (!client.connect(CLOUD_SERVER, CLOUD_PORT)) {
      Serial.println(" fail");
      delay(10000);
      return;
    }

    if (client.connected()) {
      Serial.println(" : Cloud connected");
    }
  }

  /* Obtain Data to be publised */
  get_data();

  /* Data publising */
  publish_data();

  /* Wait or Sleep */
  if (demo_mode == MODE_ALWAYS_ON) {
    /* Always On mode - just wait and start new Data Obtain-Publish cycle again */
    delay(MODE_ALWAYS_ON_DELAY_TIMEOUT);
  }
  else {
    /* One Shot mode */

    /* Disconnect from Cloud */
    client.stop();
    Serial.println("Cloud disconnected");

    /* Disconnect from GPRS */
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
      Serial.println("GPRS disconnected");
    }

    /* Modem Off */
    modem.poweroff();
    digitalWrite(MODEM_PWR_ON_PIN, LOW);
    is_modem_on = false;
    Serial.println("Modem Off");

    // BME280 : Nothing to be done - sensor works in Forced Mode
    //Serial.println("Turn BME280 Off");

    /* Move Core to Deep Sleep */
    Serial.println("Move core to Deep Sleep");
#if (defined(ESP32))
    // ESP32 specific
    esp_sleep_enable_timer_wakeup(MODE_ONE_SHOT_DELAY_TIMEOUT * 1000);
    esp_deep_sleep_start();
#else
    // AVR specific

#endif
  }
}

void get_data(void )
{
  /* Obtain BME280 reading */
  get_sensor_data();
  
  /* Get RSSI and Battery info */
  get_modem_data();
  
  /* Get GNSS Data, if available */
  get_gnss_data();
}

bool publish_data(void )
{
  String cloud_token(CLOUD_TOKEN);
  String cloud_server(CLOUD_SERVER);
  String jsonstr = "";
  int jsonlen = 0;

  jsonlen = 0;

  /* Compose JSON */
  jsonbuf["Bat.voltage"] = int_battery;
  jsonbuf["Ext.voltage"] = ext_battery;
  jsonbuf["RSSI"] = rssi;

  if (is_sensor_on) {
    jsonbuf["Temperature"] = temperature;
    jsonbuf["Humidity"] = humidity;
    jsonbuf["Pressure"] = pressure;
  }

  if (is_gnss_ready) {
    jsonbuf["latitude"] = latitude;
    jsonbuf["longitude"] = longitude;
    jsonbuf["altitude"] = altitude;
    jsonbuf["speed"] = speed;
    jsonbuf["heading"] = heading;
    jsonbuf["nsat"] = nsat;
    jsonbuf["hdop"] = hdop;

    jsonbuf["date"] = gnss_date;
    jsonbuf["time"] = gnss_time;
  }

  /* Get JSON Str*/
  serializeJson(jsonbuf, jsonstr);
  jsonlen = jsonstr.length();

  /* Compose HTTP POST request */
  reqtstr = "POST /api/v1/" + cloud_token + "/telemetry HTTP/1.1\r\nHost: " + cloud_server + "\r\nAccept: */*\r\nContent-Type: application/json\r\nContent-Length: " + jsonlen + "\r\n\r\n" + jsonstr + "\r\n\r\n";

  Serial.println("\r\nHTTP Request: ");
  Serial.println(reqtstr);

  /* Send HTTP POST request */
  client.print(reqtstr);

  Serial.println("\r\nHTTP Reply: ");

  // Wait for data to arrive
  uint32_t start = millis();
  while (client.connected() && !client.available() &&
         millis() - start < 10000L) {
    delay(100);
  };

  if (client.available()) {
    // Read data
    start = millis();
    while (client.connected() && millis() - start < 5000L) {
      while (client.available()) {
        Serial.write(client.read());
        start = millis();
      }
    }
    Serial.println();

    return true;
  }
  else {
    Serial.println("...No Reply received");

    return false;
  }
}

void get_sensor_data(void )
{
  Serial.println("\r\nBME280 Sensor: ");

  if (is_sensor_on) {
    //get and print temperatures
    Serial.print(" Temp: ");
    Serial.print(temperature = bme280.readTemperature());
    Serial.println("C");//The unit for  Celsius because original arduino don't support special symbols

    //get and print atmospheric pressure data
    Serial.print(" Pressure: ");
    Serial.print(pressure = bme280.readPressure() / 100.0F);
    Serial.println("Pa");

    //get and print humidity data
    Serial.print(" Humidity: ");
    Serial.print(humidity = bme280.readHumidity());
    Serial.println("%");
  }
  else {
    Serial.println("...No Data Available!");
  }
}

void get_modem_data(void )
{
  uint8_t chargeState = 0;
  int8_t percent     = 0;
  uint16_t milliVolts  = 0;

  // Get Battery
  Serial.println("\r\nBattery: ");

  modem.getBattStats(chargeState, percent, milliVolts);
  Serial.print(" Battery, %: ");
  Serial.println((int)percent);
  Serial.print(" Battery, mV: ");
  Serial.println((int)milliVolts);
  int_battery = milliVolts;

  // Get RSSI
  Serial.println("\r\nSignal Quality: ");

  rssi = modem.getSignalQuality();
  Serial.print(" RSSI: ");
  Serial.println(rssi);
}

void get_gnss_data(void )
{
  char pField[32];
  char buf[128];
  unsigned int len = 0;
  unsigned int i;

  // Get GNSS Data
  Serial.println("\r\nGNSS: ");

  String gnss_str = modem.getGPSraw();

  len = gnss_str.length();
  if (len) {
    gnss_str.getBytes((byte *)buf, len);

    //Serial.println(buf);

    if (getfield(buf, pField, 0, GNSS_FIELDLEN_MAX)) {
        strcpy(gnss_time, (char *)&pField);
        gnss_time[6] = 0;
    }

    if (getfield(buf, pField, 1, GNSS_FIELDLEN_MAX)) {
        strcpy(latitude, (char *)pField);
        latitude[10] = latitude[9];
        latitude[9] = ';';
        latitude[11] = 0;
    }

    if (getfield(buf, pField, 2, GNSS_FIELDLEN_MAX)) {
        strcpy(longitude, (char *)pField);
        longitude[11] = longitude[10];
        longitude[10] = ';';
        longitude[12] = 0;
    }

    if (getfield(buf, pField, 3, GNSS_FIELDLEN_MAX)) {
        strcpy(hdop, (char *)pField);
        hdop[3] = 0;
    }

    if (getfield(buf, pField, 4, GNSS_FIELDLEN_MAX)) {
        i = 0;
        len = strlen((char *)pField);
        while(pField[i] != '.' && i < len)
        {
            i++;
        }
        pField[i] = 0;
        strcpy(altitude, (char *)pField);
    }

    if (getfield(buf, pField, 5, GNSS_FIELDLEN_MAX)) {
        strcpy(quality, (char *)pField);
        quality[1] = 0;
    }

    if (getfield(buf, pField, 6, GNSS_FIELDLEN_MAX)) {
        i = 0;
        while(pField[i] != '.' && i < len)
        {
            i++;
        }
        pField[i] = 0;

        strcpy(heading, (char *)pField);
    }

    if (getfield(buf, pField, 7, GNSS_FIELDLEN_MAX)) {
        i = 0;
        len = strlen((char *)pField);
        while(pField[i] != '.' && i < len)
        {
            i++;
        }
        pField[i] = 0;

        strcpy(speed, (char *)pField);
    }

    if (getfield(buf, pField, 9, GNSS_FIELDLEN_MAX)) {
        strcpy(gnss_date, (char *)pField);
        gnss_date[6] = 0;
    }

    if (getfield(buf, pField, 10, GNSS_FIELDLEN_MAX)) {
        pField[2] = 0;
        strcpy(nsat, (char *)pField);
    }

    Serial.print(" Quality : ");
    Serial.println((char *)quality);

    Serial.print(" Date : ");
    Serial.println((char *)gnss_date);

    Serial.print(" Time : ");
    Serial.println((char *)gnss_time);

    Serial.print(" Latitude : ");
    Serial.println((char *)latitude);

    Serial.print(" Longitude : ");
    Serial.println((char *)longitude);

    Serial.print(" Altitude : ");
    Serial.println((char *)altitude);

    Serial.print(" Speed : ");
    Serial.println((char *)speed);

    Serial.print(" Heading : ");
    Serial.println((char *)heading);

    Serial.print(" HDOP : ");
    Serial.println((char *)hdop);

    Serial.print(" Sats In Use : ");
    Serial.println((char *)nsat);

    is_gnss_ready = true;
  }
  else {
    Serial.println(" ...No Data Available!");
    is_gnss_ready = false;
  }
}

bool getfield(char * pBuf, char * pField, int idx, int len)
{
    int i = 0;
    int j = 0;
    int nField = 0;

    if (pBuf == NULL || pField == NULL || len <= 0) {
        return false;
    }

    while (nField != idx && pBuf[i]) {
        if (pBuf[i] == ',') {
            nField++;
        }

        i++;

        if (pBuf[i] == '\0') {
            pField[0] = '\0';
            return false;
        }
    }

    if (pBuf[i] == ',') {
        pField[0] = '\0';
        return false;
    }

    while (pBuf[i] != ',' && pBuf[i] != '*' && pBuf[i]) {
        pField[j] = pBuf[i];
        j++; i++;

        if (j >= len) {
            j = len-1;
            break;
        }
    }
    pField[j] = '\0';

    return true;
}
