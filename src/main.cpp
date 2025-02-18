#include <Arduino.h>
#include <GyverOLED.h>
//GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
//GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
//GyverOLED<SSD1306_128x64, OLED_BUFFER, OLED_SPI, 8, 7, 6> oled;#include <BLEDevice.h>
#include <ArduinoBLE.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
GyverOLED<SSH1106_128x64> oled;

float n = 112.5;
unsigned int c = 14, c_set = 25, n_set = 150, fixvel = 15;
byte vel = 0;
unsigned long tm = 0;
void oled_print() {
  // for (byte i = 0; i < 8; i += x) {
    oled.setCursorXY(3, 3);
    oled.print("Руч.");
    oled.setCursorXY(46-10, 3);
    oled.print("Факт.");
    oled.setCursorXY(103, 3);
    oled.print("Уст.");
    oled.setCursorXY(3, 18);
    oled.print("Обор.");
    oled.setCursorXY(3, 33);
    oled.print("Цикл.");
    oled.setCursorXY(103, 18);
    oled.print(n_set);
    oled.setCursorXY(103, 33);
    oled.print(c_set);
    oled.setCursorXY(3, 48);
    oled.print("Скорость");
    oled.setScale(2);
    // oled.setCursorXY(33, 31);
    // oled.print(c);
    // oled.setCursorXY(33, 16);
    // oled.print(n,1);
    // oled.setCursorXY(80, 46);
    // oled.print(0);
    // oled.print("%");
    // oled.autoPrintln(true);
    // oled.print("Оборотов: ");
    // oled.setScale(2); oled.print(121.5, 1);
    // oled.setScale(1); oled.print("; макс.: "); oled.println(150.5, 1);
    // oled.print("Циклов: ");
    // oled.setScale(2); oled.print(14);
    // oled.setScale(1); oled.print("; макс.: "); oled.println(25, 1);
  // }
  oled.rect(0,0,126,60,OLED_STROKE);
  oled.fastLineH(15,0,126);
  oled.fastLineH(30,0,126);
  oled.fastLineH(45,0,126);
  oled.fastLineV(30,0,45);
  oled.fastLineV(100,0,45);
  oled.fastLineV(60,45,60);
  // delay(1000);
}

BLEService ledService("19b10000-e8f2-537e-4f6c-d104768a1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEStringCharacteristic nsetCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic csetCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic fixvelCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic regvelCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 10);
BLEDescriptor nsetDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Колличество оборотов");
BLEDescriptor csetDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Колличество циклов");
BLEDescriptor fixvelDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Фикс. скорость");
BLEDescriptor regvelDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Регул. скорость");
// const int ledPin = LED_BUILTIN;

// void blePeripheralConnectHandler(BLEDevice central) {
//   // central connected event handler
//   Serial.print("Connected event, central: ");
//   Serial.println(central.address());
// }

// void blePeripheralDisconnectHandler(BLEDevice central) {
//   // central disconnected event handler
//   Serial.print("Disconnected event, central: ");
//   Serial.println(central.address());
// }

void nsetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  n_set = (unsigned int) nsetCharacteristic.value().toInt();
}

void csetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  c_set = (unsigned int) csetCharacteristic.value().toInt();
}

void fixvelCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  fixvel = (unsigned int) fixvelCharacteristic.value().toInt();
}

void setup() {
  // Serial.begin(9600);  while (!Serial);
  
  // pinMode(ledPin, OUTPUT); // use the LED pin as an output

  // begin initialization
  if (!BLE.begin()) {
    // Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  // set the local name peripheral advertises
  BLE.setLocalName("Рабица 2");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  nsetCharacteristic.addDescriptor(nsetDesc);
  csetCharacteristic.addDescriptor(csetDesc);
  fixvelCharacteristic.addDescriptor(fixvelDesc);
  regvelCharacteristic.addDescriptor(regvelDesc);
  
  // assign event handlers for characteristic
  nsetCharacteristic.setEventHandler(BLEWritten, nsetCharacteristicWritten);
  csetCharacteristic.setEventHandler(BLEWritten, csetCharacteristicWritten);
  fixvelCharacteristic.setEventHandler(BLEWritten, fixvelCharacteristicWritten);
  // set an initial value for the characteristic
  nsetCharacteristic.setValue(String(n_set));
  csetCharacteristic.setValue(String(c_set));
  fixvelCharacteristic.setValue(String(fixvel));
  // add the characteristic to the service
  ledService.addCharacteristic(nsetCharacteristic);
  ledService.addCharacteristic(csetCharacteristic);
  ledService.addCharacteristic(fixvelCharacteristic);
  ledService.addCharacteristic(regvelCharacteristic);

  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  // BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  // BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);



  // start advertising
  BLE.advertise();

  // Serial.println(("Bluetooth® device active, waiting for connections..."));
  oled.init();              // инициализация
  oled.clear();
  oled.setScale(1);
  oled_print();
    oled.update();
  tm = millis();
}

void loop() {
  BLE.poll();
  if ((millis() - tm) >= 300)
  {
    tm = millis();
    n = (float) random(200)*0.5;
    c = (unsigned int) random(50);
    oled.clear();
    oled.setScale(2);
    oled.setCursorXY(33, 31);
    oled.print(c);
    oled.setCursorXY(33, 16);
    oled.print(n,1);
    vel = map(analogRead(36), 0, 4095, 0, 100);
    regvelCharacteristic.setValue(String(vel));
    oled.setCursorXY(80, 46);
    oled.print(vel);
    oled.print("%");
    oled.setScale(1);
    oled_print();
    oled.update();
  }
}