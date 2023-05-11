//***********************************************************************************************
//                                    Code Revision 1.10
// Added low-power high-power modes to the code.
//***********************************************************************************************

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  Library Includes  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//BLE libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Uncomment if no 16x2 LCD is used
#include <Wire.h> 
// #include <LiquidCrystal_I2C.h> // no longer used opted for 128x64 OLED DISPLAY
// Library for 128x64 OLED display 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Specific Screen parameters
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// OneWire Protocol for the DS18B20 Temp Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Library for the PT100 MAX31865 to compare values for the DS18B20
#include <Adafruit_MAX31865.h>

// Sensor Library for the capacitive soil sensor (seesaw)
#include <Adafruit_seesaw.h>

// ADC for the PH sensor (16-Bit) 65,536
#include <Adafruit_ADS1X15.h>

//PH sensor libraries
#include <Ezo_i2c.h>                      // Link: https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>                         //include arduinos i2c library

Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
bool reading_request_phase = true;        //selects our phase
uint32_t next_poll_time = 0;              //holds the next time we receive a response, in milliseconds
const unsigned int response_delay = 1000; //how long we wait to receive a response, in milliseconds

float phValue = 0;                         // variable to store the ph values of the sensor
 
//Libraries for the NPK sensor: 
//defining the pins on the board
#define RE 33 //GPIO 4
#define DE 25 // GPIO 0
//Software serial for rx/tx communication on regular pins:
#include <SoftwareSerial.h>
//Array address to store byte information on npk sensor:
//const byte code[]= {0x01, 0x03, 0x00, 0x1e, 0x00, 0x03, 0x65, 0xCD};
const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

byte values[11];
SoftwareSerial mod(3,1); //rx1 tx1 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  Global Constants   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// GPIO where the DS18B20 is connected to
const int oneWireBus = 32; //changed pin 14 to pin 34

//***********************************************
//*  PH = RED     SOIL = GREEN    TEMP = BLUE   *
//***********************************************

const int ph_led = 13; 
const int soil_led = 12;
const int temp_led = 14;  //changed pin 33 to pin 14

// // Ph Sensor Values !!! EACH SENSOR MUST BE CALIBRATED INDIVIDUALLY !!!
// float low_Ph = 3.33;
// float mid_Ph = 2.91;
// float high_Ph = 2.54;

// Define Pins for the LED's to light up when collecting data
int pin;
int pH_pin = 0;
int Soil_pin = 2;
int Temp_pin = 15;

// Defining which ADC we are using in this case its the ads1115
Adafruit_ADS1115 ads1115;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

Adafruit_MAX31865 thermo = Adafruit_MAX31865(5, 23, 19, 18);

 
// Creating the lcd object address 0x3F and 16 columns x 2 rows
// If no LCD Comment out !!! some LCD's have a different address !!! 
// LiquidCrystal_I2C lcd (0x3F, 16,2);  


//BLE global variables
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
bool dataWritten = false;
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
//NPK variables





char output[20];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
 
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
 
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
 
        Serial.println();
        Serial.println("*********");

        dataWritten = true;
      }
    }
};


void  setup () {

  // Start the serial monitor for debugging purposes 
  // Final revision to disable the serial monitor
  Serial.begin(4800);
  setCpuFrequencyMhz(80); // set esp32 clock frequency to 80Mhz for low power mode ~~~~~ LOW-POWER MODE ~~~~~~~~
   // Create the BLE Device
  BLEDevice::init("Eagles ESP32 2");
 
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
 
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
 
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_TX,
                                        BLECharacteristic::PROPERTY_NOTIFY
                                    );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());
 
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                             CHARACTERISTIC_UUID_RX,
                                            BLECharacteristic::PROPERTY_WRITE
                                        );
 
  pRxCharacteristic->setCallbacks(new MyCallbacks());
 
  // Start the service
  pService->start();
 
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  mod.begin(4800);
  Wire.begin();                           //start the I2C for PH sensor and OLED display

  // Setting the data colletion pins as output.
  pinMode(ph_led, OUTPUT);
  pinMode(soil_led, OUTPUT);
  pinMode(temp_led, OUTPUT);
  pinMode(pH_pin, OUTPUT);
  pinMode(Soil_pin, OUTPUT);
  pinMode(Temp_pin, OUTPUT);

  // uncomment for use with pt100
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //thermo.begin(MAX31865_3WIRE);

  // check for display initialization ... no LED as this is not part of the scope and used for 
  // debuggin
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  // void setup() for NPK sensor:
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  // Light up LED to check the sensor presence
  // TODO: add check statement to actually tie the LED to the sensor

  digitalWrite(ph_led, HIGH);
  digitalWrite(soil_led, HIGH);
  digitalWrite(temp_led, HIGH);

  // Serial monitor test for COM port
  Serial.println("Port Connected!");
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 188uV)");
  ads1115.begin(0x48);

}
 
void  loop () {

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pH SECTION Begin   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  pin = pH_pin; // selecting pH led
  blink_Led(); // blinking the pH LED
  // adc set to single ended polling conversion on adc0

  //// beginning of PH sensor code//////////
  if (reading_request_phase)             //if were in the phase where we ask for a reading
  {
    //send a read command. we use this command instead of PH.send_cmd("R");
    //to let the library know to parse the reading
    PH.send_read_cmd();
 
    next_poll_time = millis() + response_delay;         //set when the response will arrive
    reading_request_phase = false;                      //switch to the receiving phase
  }
  else                                                //if were in the receiving phase
  {
    if (millis() >= next_poll_time)                    //and its time to get the response
    {
      receive_reading(PH);                              //get the reading from the PH circuit
 
      reading_request_phase = true;                     //switch back to asking for readings
    }
  }
  //////ending of PH sensor code//////////
  if (phValue >= 20){
    digitalWrite(ph_led, LOW);
  }
  else{
    digitalWrite(ph_led, HIGH);
  }


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pH SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  SOIL SECTION BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  pin = Soil_pin;
  blink_Led();
  //digitalWrite(25, LOW);
  
  int16_t adc1;

  adc1 = ads1115.readADC_SingleEnded(1);
  Serial.println(adc1);

  if (adc1 >15500 | adc1 < 6700 ){
    digitalWrite(soil_led, LOW);    
  }
  else{
    digitalWrite(soil_led, HIGH);
  }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  SOIL SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  TEMP SECTION BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  pin = Temp_pin; // selecting Temp led
  blink_Led(); // selecting Temp led
  // requesting temperature from the DS18B20 Sensor
  sensors.requestTemperatures(); 

  // Uncomment temperatureC to get temperature in Celsius
  //float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0); // requesting temperature in F
  Serial.println(temperatureF);

  if (temperatureF <= -190){
    digitalWrite(temp_led, LOW);
  }
  else{
    digitalWrite(temp_led, HIGH);
  }  
  temperatureF = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  TEMP SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//***********************************************************************************************
//                      PT100 sensor was used for precise temperature measurement 
//                      the variance was about .5 of a degree in comparison with 
//                      the DS18B20 sensor so we opted for the cheaper verssion
//***********************************************************************************************

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pt100 SECTION Begin   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//  uint16_t rtd = thermo.readRTD();
//  float tempf;
//
//  Serial.print("RTD value: "); Serial.println(rtd);
//  float ratio = rtd;
//  ratio /= 32768;
//  //Serial.print("Ratio = "); Serial.println(ratio,8);
//  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
//  tempf = (thermo.temperature(RNOMINAL, RREF))*(1.8) + 32;
//  Serial.print("Temperature = "); Serial.println(tempf);
//
//  // Check and print any faults
//  uint8_t fault = thermo.readFault();
//  if (fault) {
//    Serial.print("Fault 0x"); Serial.println(fault, HEX);
//    if (fault & MAX31865_FAULT_HIGHTHRESH) {
//      Serial.println("RTD High Threshold"); 
//    }
//    if (fault & MAX31865_FAULT_LOWTHRESH) {
//      Serial.println("RTD Low Threshold"); 
//    }
//    if (fault & MAX31865_FAULT_REFINLOW) {
//      Serial.println("REFIN- > 0.85 x Bias"); 
//    }
//    if (fault & MAX31865_FAULT_REFINHIGH) {
//      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
//    }
//    if (fault & MAX31865_FAULT_RTDINLOW) {
//      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
//    }
//    if (fault & MAX31865_FAULT_OVUV) {
//      Serial.println("Under/Over voltage"); 
//    }
//    thermo.clearFault();
//  }
//  Serial.println();
//  delay(1000);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pt100 SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//***********************************************************************************************
//                      comment out entire section if no lcd is connected
//***********************************************************************************************

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   LCD SECTION BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  lcd.setCursor(0,0);
//  lcd.print("Temp   pH   Humd");
//  lcd.setCursor(0,1);
//  lcd.print(temperatureF);
//  lcd.setCursor(6,1);
//  lcd.print(phValue);
//  lcd.setCursor(13,1);
//  //lcd.print();
  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   LCD SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   OLED SECTION BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  //display.println("Enviromental Monitor");
  //display.setCursor(0, 10);
  display.println("Temperature F:");
  display.setCursor(0, 10);
  display.println(temperatureF);
  display.setCursor(0, 20);
  display.println("pH Value:");
  display.setCursor(0, 30);  
  display.println(phValue);
  display.setCursor(0, 40);  
  display.println("Soil Moisture:");
  display.setCursor(0, 50);
  display.println(adc1);
  display.display(); 
  delay(3000);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   NPK SENSOR SECTION BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  byte val1,val2,val3;  //initialize to zero move to global variable
  val1 = nitrogen();
  delay(250);
  val2 = phosphorous();
  delay(250);
  val3 = potassium();
  delay(250);

  display.clearDisplay();
  
 
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("N: ");
  display.print(val1);
  display.setTextSize(1);
  display.print(" mg/kg");
 
  display.setTextSize(2);
  display.setCursor(0, 25);
  display.print("P: ");
  display.print(val2);
  display.setTextSize(1);
  display.print(" mg/kg");
 
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print("K: ");
  display.print(val3);
  display.setTextSize(1);
  display.print(" mg/kg");
  display.display();
  delay(3000);

  if (dataWritten) { 
    setCpuFrequencyMhz(240);      //increases the esp32 clock speed to 240Mhz ~~~~~~~~~~~~HIGH-POWER MODE~~~~~~~~~~~~~~~~~
    
    sprintf(output, "{\"PH\":%f,", phValue);  //pHsensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();

    delay(20);
    sprintf(output, "\"Phosphorus\":%d,", val3 ); //phosphorusSensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();
    
    delay(20);
    sprintf(output, "\"Temp\":%f,", temperatureF); //temp Sensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();

    delay(20);
    sprintf(output, "\"Nitrogen\":%d,",val1 ); //nitrogenSensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();

    delay(20);
    sprintf(output, "\"Potassium\":%d,", val2 ); //potassiumSensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();

    delay(20);
    sprintf(output, "\"Moisture\":%d}", adc1); //moistureSensor
    pTxCharacteristic->setValue(output);
    pTxCharacteristic->notify();

    dataWritten = false;
    delay(20);
    
    }
 
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   NPK SENSOR SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   OLED SECTION END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    END OF LOOP  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  Blink Fucntion  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Custom fucntions to blink the LED depending on the sensor
// 166 milliseconds to total 1 second blink for each sensor 333 millisecond each sequence
// The LED is a common anode LED so the PIN must be pulled low first to light up
// go from HIGH --> LOW if common cathode LED

void blink_Led(){
  
    digitalWrite(pin, LOW); // Set GPIO22 active low
    delay(166); // delay of 166 millisecond
    digitalWrite(pin, HIGH); // Set GPIO22 active high
    delay(166);  // delay of 166 millisecond
    digitalWrite(pin, LOW); // Set GPIO22 active low
    delay(166); // delay of 166 millisecond
    digitalWrite(pin, HIGH); // Set GPIO22 active high
    delay(166);  // delay of 166 millisecond
    digitalWrite(pin, LOW); // Set GPIO22 active low
    delay(166); // delay of 166 millisecond
    digitalWrite(pin, HIGH); // Set GPIO22 active high
    delay(166);  // delay of 166 millisecond
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   NPK SENSOR FUNCTIONS BEGIN   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
byte nitrogen(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(nitro,sizeof(nitro))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial.print(values[i],HEX);
    }
    //Serial.println();
  }
  return values[4];
}
 
byte phosphorous(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(phos,sizeof(phos))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial.print(values[i],HEX);
    }
    //Serial.println();
  }
  return values[4];
}
 
byte potassium(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(pota,sizeof(pota))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial.print(values[i],HEX);
    }
    Serial.println();
  }
  return values[4];
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   NPK SENSOR FUNCTIONS END   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pH FUNCTION SECTION BEGINS   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void receive_reading(Ezo_board &Sensor)                // function to decode the reading after the read command was issued
{
 
  Serial.print(Sensor.get_name());  // print the name of the circuit getting the reading
  Serial.print(": ");
 
  Sensor.receive_read_cmd();                                //get the response data and put it into the [Sensor].reading variable if successful
 
  switch (Sensor.get_error())                          //switch case based on what the response code is.
  {
    case Ezo_board::SUCCESS:
      //Serial.println(Sensor.get_last_received_reading());               //the command was successful, print the reading
      phValue = Sensor.get_last_received_reading();
      //delay(200);
      break;
 
    case Ezo_board::FAIL:
      Serial.print("Failed ");                          //means the command has failed.
      break;
 
    case Ezo_board::NOT_READY:
      Serial.print("Pending ");                         //the command has not yet been finished calculating.
      break;
 
    case Ezo_board::NO_DATA:
      Serial.print("No Data ");                         //the sensor has no data to send.
      break;
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  pH FUNCTION SECTION ENDS  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

