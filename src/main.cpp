#include <Arduino.h>
// #include <Bounce2mcp.h>
// #include <Adafruit_MCP23X17.h>
#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h>
#include <GyverOLED.h>
// GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
// GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
//  GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_BUFFER, OLED_SPI, 8, 7, 6> oled;#include <BLEDevice.h>
#include <ArduinoBLE.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include <driver/pcnt.h>
#include "esp_attr.h"
// inputs
// #include "BLE.h"
/**Bluetooth**/
// BLE BT;
#include "SPIFFS.h"
#include "FS.h"
// #include "FFat.h"
#include <ArduinoJson.h>
enum sx_ins
{
  al_stop = 3,
  main_rot1 = 5,
  main_rot2 = 7,
  main_kar1 = 9,
  main_kar2 = 10,
  main_aut = 12,
  main_cut = 14,
  sens_kar1 = 2,
  sens_kar2 = 15,
  sens_cut = 4
};

// enum mcp_ins
// {
//   al_stop = 14,
//   main_rot1 = 12,
//   main_rot2 = 8,
//   main_kar1 = 6,
//   main_kar2 = 5,
//   main_aut = 3,
//   main_cut = 1,
//   sens_kar1 = 0,
//   sens_kar2 = 15,
//   sens_cut = 13
// };

enum nat_ins
{
  sens_rot = 34
};
// outputs
enum sx_outs
{
  vel_rot = 33,
  FWD_ROT = 6,
  FWD_KAR = 8,
  REV_KAR = 11,
  FWD_CUT = 13
};
// enum outs
// {
//   vel_rot = 33,
//   FWD_ROT = 9,
//   FWD_KAR = 7,
//   REV_KAR = 4,
//   FWD_CUT = 2
// };
SX1509 io, *mcp;
volatile uint16_t intr_src = 0;
volatile bool is_intr = false;
// Adafruit_MCP23X17 mcp;

// My deboncing class
class BounceMcp
{
public:
  BounceMcp();

  // Attach to a MCP object, a pin (and also sets initial state), and set debounce interval.
  void attach(SX1509 *io, int pin);
  // Updates the pin
  // Returns 1 if the state changed
  // Returns 0 if the state did not change
  bool update(uint16_t intr_src);

  // Returns the updated pin state
  bool read();

  // Returns the falling pin state
  bool fell();

  // Returns the rising pin state
  bool rose();

protected:
  uint8_t state, tmp_state = 0;
  uint8_t pin;
  SX1509 *io;
};

BounceMcp::BounceMcp()
    : state(0), pin(0) {};

void BounceMcp::attach(SX1509 *io, int pin)
{
  this->io = io;
  this->pin = pin;
  this->io->pinMode(this->pin, INPUT);
  this->io->debouncePin(this->pin);
  this->state = this->io->digitalRead(this->pin);
  this->tmp_state = this->state;
  this->io->enableInterrupt(this->pin, CHANGE);
}

bool BounceMcp::update(uint16_t intr_src)
{
  if (intr_src & (1 << this->pin))
  {
    this->state = this->io->digitalRead(this->pin);
    return true;
  }
  else
  {
    this->tmp_state = this->state;
    return false;
  }
}

bool BounceMcp::read()
{
  if (this->state == HIGH)
    return true;
  else
    return false;
}

bool BounceMcp::fell()
{
  return (this->state == 0) && (this->tmp_state == 1);
}

bool BounceMcp::rose()
{
  return (this->state == 1) && (this->tmp_state == 0);
}
// Instantiate a Bounce object

BounceMcp deb_al_stop = BounceMcp();
BounceMcp deb_main_rot1 = BounceMcp();
BounceMcp deb_main_rot2 = BounceMcp();
BounceMcp deb_main_kar1 = BounceMcp();
BounceMcp deb_main_kar2 = BounceMcp();
BounceMcp deb_main_aut = BounceMcp();
BounceMcp deb_main_cut = BounceMcp();
BounceMcp deb_sens_kar1 = BounceMcp();
BounceMcp deb_sens_kar2 = BounceMcp();
BounceMcp deb_sens_cut = BounceMcp();

GyverOLED<SSH1106_128x64> oled;
File file;
JsonDocument doc;
float n_set = 15.5, cnt_lim = 0;
int16_t *vel = 0, c_set = 25, it_c = 0, fixvel = 15, prev_vel = 0, regvel = 0;
unsigned long tm = 0;
bool aut_mode_perm = false, cnt_main_rot = false, cnt_aut_rot = false, rot = false, cnt_lim_chng = false, *fwdkar, *revkar, aut_fwd_kar = false,
     aut_rev_kar = false, main_fwd_kar = false, main_rev_kar = false, *cut, cut_main = false, cut_aut = false, fix_vel = false, *cnt_rot, isr_inst = false,
     fwd_rot_out = false, tmp_fwd_rot_out = false, fwd_kar_out = false, tmp_fwd_kar_out = false, rev_kar_out = false, tmp_rev_kar_out = false,
     fwd_cut_out = false, tmp_fwd_cut_out = false;
void oled_print()
{
  // for (byte i = 0; i < 8; i += x) {
  oled.setCursorXY(3, 3);
  oled.print("Руч.");
  oled.setCursorXY(46 - 10, 3);
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
  oled.rect(0, 0, 126, 60, OLED_STROKE);
  oled.fastLineH(15, 0, 126);
  oled.fastLineH(30, 0, 126);
  oled.fastLineH(45, 0, 126);
  oled.fastLineV(30, 0, 45);
  oled.fastLineV(100, 0, 45);
  oled.fastLineV(60, 45, 60);
}

BLEService ledService("19b10000-e8f2-537e-4f6c-d104768a1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEStringCharacteristic nsetCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic csetCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic fixvelCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 10);
BLEStringCharacteristic regvelCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 10);
BLEStringCharacteristic ncntCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 10);
BLEStringCharacteristic ccntCharacteristic("19b10000-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 10);
BLEDescriptor nsetDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Колличество оборотов");
BLEDescriptor csetDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Колличество циклов");
BLEDescriptor fixvelDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Фикс. скорость");
BLEDescriptor regvelDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Регул. скорость");
BLEDescriptor ncntDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Счетчик оборотов");
BLEDescriptor ccntDesc("19b10000-e8f2-537e-4f6c-d104768a1214", "Счетчик циклов");

pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

#define PCNT_H_LIM_VAL n_set
// #define PCNT_L_LIM_VAL     -10
// #define PCNT_THRESH1_VAL    5
// #define PCNT_THRESH0_VAL   0
#define PCNT_INPUT_SIG_IO sens_rot           // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO PCNT_PIN_NOT_USED // Control GPIO HIGH=count up, LOW=count down

xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; // user's ISR service handle
/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct
{
  int unit;        // the PCNT unit that originated an interrupt
  uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

int16_t count = 0, old_count = 0;
pcnt_evt_t evt;
portBASE_TYPE res;

void setvel()
{
  if (((*vel - prev_vel) >= 1) || ((*vel - prev_vel) <= -1))
  {
    prev_vel = *vel;
    ledcWriteTone(0, (uint32_t)map(*vel, 0, 100, 200, 5000));
    regvelCharacteristic.setValue(String(*vel));
  }
}

void pause_cnt()
{
  if (isr_inst)
  {
    isr_inst = false;
    pcnt_counter_pause(pcnt_unit);
    // detachInterrupt(sens_rot);
  }
}

// void ARDUINO_ISR_ATTR sens_rot_isr() {
//     count++;
//     if (count == (cnt_lim - 3))
//     {
//       fix_vel = true;
//       vel = fixvel;
//     }
//     else if (count == (cnt_lim))
//     {
//       *cnt_rot = false;
//       fix_vel = false;
//       count = 0;
//     }
// }

void resume_cnt()
{
  // attachInterrupt(sens_rot, sens_rot_isr, FALLING);
  if (!isr_inst)
  {
    isr_inst = true;
    pcnt_counter_resume(pcnt_unit);
  }
}
/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
  int pcnt_unit = (int)arg;
  pcnt_evt_t evt;
  evt.unit = pcnt_unit;
  /* Save the PCNT event type that caused an interrupt
     to pass it to the main program */
  pcnt_get_event_status((pcnt_unit_t)pcnt_unit, &evt.status);
  xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}

void saveConfig(const char *nameParam, float *value)
{
  doc[nameParam] = *value;
  file = SPIFFS.open("/config.txt", FILE_WRITE);
  serializeJson(doc, file);
  file.close();
}

void saveConfig(const char *nameParam, int16_t *value)
{
  doc[nameParam] = *value;
  file = SPIFFS.open("/config.txt", FILE_WRITE);
  serializeJson(doc, file);
  file.close();
}

void nsetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
{
  if ((nsetCharacteristic.value().toFloat() != n_set) && ((int16_t)nsetCharacteristic.value().toFloat() > 3.0))
  {
    n_set = nsetCharacteristic.value().toFloat();
    saveConfig("n_set", &n_set);
  }
}

void csetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
{
  if (((int16_t)csetCharacteristic.value().toInt() != c_set) && ((int16_t)csetCharacteristic.value().toInt() > 0))
  {
    c_set = (int16_t)csetCharacteristic.value().toInt();
    saveConfig("c_set", (&c_set));
  }
}

void fixvelCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
{
  if ((int16_t)fixvelCharacteristic.value().toInt() != fixvel)
  {
    fixvel = (unsigned int)fixvelCharacteristic.value().toInt();
    saveConfig("fix_v", (&fixvel));
  }
}

static void pcnt_example_init(pcnt_unit_t unit, float lim)
{

  pcnt_config_t pcnt_config = {
      // Set PCNT input signal and control GPIOs
      .pulse_gpio_num = PCNT_INPUT_SIG_IO,
      .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
      .lctrl_mode = PCNT_MODE_KEEP,    // Reverse counting direction if low
      .hctrl_mode = PCNT_MODE_DISABLE, // Keep the primary counter mode if high
      .pos_mode = PCNT_COUNT_DIS,      // Count up on the positive edge
      .neg_mode = PCNT_COUNT_INC,      // Keep the counter value on the negative edge
      .counter_h_lim = 1000,
      // .counter_l_lim = PCNT_L_LIM_VAL,
      .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(unit, 1000);
  pcnt_filter_enable(unit);

  /* Set threshold 0 and 1 values and enable events to watch */
  pcnt_set_event_value(unit, PCNT_EVT_THRES_1, lim * 2);
  pcnt_event_enable(unit, PCNT_EVT_THRES_1);
  pcnt_set_event_value(unit, PCNT_EVT_THRES_0, lim * 2 - 2);
  pcnt_event_enable(unit, PCNT_EVT_THRES_0);
  /* Enable events on zero, maximum and minimum limit values */
  // pcnt_event_enable(unit, PCNT_EVT_ZERO);
  // pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  // pcnt_event_enable(unit, PCNT_EVT_L_LIM);

  /* Install interrupt service and add isr callback handler */
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);
}

void changing_pcnt()
{
  if (!*cnt_rot)
  {
    pcnt_set_event_value(pcnt_unit, PCNT_EVT_THRES_1, cnt_lim * 2);
    if (cnt_lim * 2 > 1)
    {
      vel = &regvel;
      setvel();
      pcnt_set_event_value(pcnt_unit, PCNT_EVT_THRES_0, cnt_lim * 2 - 2);
      pcnt_event_enable(pcnt_unit, PCNT_EVT_THRES_0);
    }
    else
    {
      // fix_vel = true;
      vel = &fixvel;
      setvel();
      pcnt_set_event_value(pcnt_unit, PCNT_EVT_THRES_0, 0);
      pcnt_event_disable(pcnt_unit, PCNT_EVT_THRES_0);
    }
    pcnt_counter_clear(pcnt_unit);
  }
}

void oled_update(void *parameter)
{
  for (;;)
  {
    // oled.clear();
    // oled.setCursorXY(0, 0);
    // oled.setScale(1);
    // oled.print("Об. ");
    // oled.setScale(3);
    // oled.print((float)count / 2, 1);
    // oled.setScale(1);
    // oled.print("/");
    // oled.print(n_set, 1);
    // oled.setCursorXY(0, 31);
    // oled.print("Цк. ");
    // oled.setScale(3);
    // oled.print(it_c);
    // oled.setScale(1);
    // oled.print("/");
    // oled.print(c_set, 1);
    // // oled.setCursorXY(80, 46);
    // // oled.print(*vel);
    // // oled.print("%");
    // // oled.setScale(1);
    // // oled_print();
    // oled.update();
    static int16_t lc_couunt = 0;
    if (lc_couunt != count) {
      lc_couunt = count;
      ncntCharacteristic.setValue(String((float)count / 2));
    }
    static int16_t lc_it_c = 0;
    if (lc_it_c != it_c) {
      lc_it_c = it_c;
      ccntCharacteristic.setValue(String(lc_it_c));
    }
    BLE.poll();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ARDUINO_ISR_ATTR sx_isr()
{
  is_intr = true;
}

void setup()
{
  // Serial.begin(9600);
  // while (!Serial)
  //   ;
  // FFat.format();
  // if (!FFat.begin())
  // {
  //   Serial.println("FFat Mount Failed");
  //   return;
  // }
  // elseF

  // oled.init(); // инициализация
  // oled.clear();
  // oled.setScale(1);
  {
    SPIFFS.begin();
    file = SPIFFS.open("/config.txt", FILE_READ);
    deserializeJson(doc, file);
    n_set = doc["n_set"];
    c_set = doc["c_set"];
    fixvel = doc["fix_v"];
    file.close();
  }
  // pinMode(ledPin, OUTPUT); // use the LED pin as an output

  Wire.begin();

  // // Call io.begin(<address>) to initialize the SX1509. If it
  // // successfully communicates, it'll return 1.
  if (io.begin() == false)
  {
    Serial.println("Failed to communicate. Check wiring and address of SX1509.");
    while (1)
      ; // If we fail to communicate, loop forever.
  }
  mcp = &io;
  //   // begin initialization
  // delay(1000);
  // if (!mcp.begin_SPI(5))
  // {
  //   // Serial.println("Error.");
  //   while (1)
  //     ;
  // }
  // for (int i : Enum.GetValues(typeof(mcp_ins)))
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(5, sx_isr, FALLING);
  // mcp.pinMode(al_stop, INPUT);
  // mcp.pinMode(main_rot1, INPUT);
  // mcp.pinMode(main_rot2, INPUT);
  // mcp.pinMode(main_kar1, INPUT);
  // mcp.pinMode(main_kar2, INPUT);
  // mcp.pinMode(main_aut, INPUT);
  // mcp.pinMode(main_cut, INPUT);
  // mcp.pinMode(sens_kar1, INPUT);
  // mcp.pinMode(sens_kar2, INPUT);
  // mcp.pinMode(sens_cut, INPUT);
  // deb_al_stop.attach(mcp, al_stop, 5);
  // deb_main_rot1.attach(mcp, main_rot1, 5);
  // deb_main_rot2.attach(mcp, main_rot2, 5);
  // deb_main_kar1.attach(mcp, main_kar1, 5);
  // deb_main_kar2.attach(mcp, main_kar2, 5);
  // deb_main_aut.attach(mcp, main_aut, 5);
  // deb_main_cut.attach(mcp, main_cut, 5);
  // deb_sens_kar1.attach(mcp, sens_kar1, 5);
  // deb_sens_kar2.attach(mcp, sens_kar2, 5);
  // deb_sens_cut.attach(mcp, sens_cut, 5);


  io.debounceTime(8);
  deb_main_cut.attach(&io, main_cut);
  deb_al_stop.attach(&io, al_stop);
  deb_main_rot1.attach(&io, main_rot1);
  deb_main_rot2.attach(&io, main_rot2);
  deb_main_kar1.attach(&io, main_kar1);
  deb_main_kar2.attach(&io, main_kar2);
  deb_main_aut.attach(&io, main_aut);
  deb_sens_kar1.attach(&io, sens_kar1);
  deb_sens_kar2.attach(&io, sens_kar2);
  deb_sens_cut.attach(&io, sens_cut);
  pinMode(sens_rot, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(vel_rot, OUTPUT);
  mcp->pinMode(FWD_ROT, OUTPUT);
  mcp->pinMode(FWD_KAR, OUTPUT);
  mcp->pinMode(REV_KAR, OUTPUT);
  mcp->pinMode(FWD_CUT, OUTPUT);
  ledcSetup(0, 20, 12);
  ledcWrite(0, 0);
  ledcAttachPin(vel_rot, 0);
  // for (byte i = 0; i <= 15; i++) {
  //   if (i < 5) mcp.pinMode(i, OUTPUT);
  //   else if (i > 5) mcp.pinMode(i, INPUT);
  // }
  if (!BLE.begin())
  {
    // Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }
  // set the local name peripheral advertises
  BLE.setLocalName("Рабица 1");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);
  nsetCharacteristic.addDescriptor(nsetDesc);
  ncntCharacteristic.addDescriptor(ncntDesc);
  csetCharacteristic.addDescriptor(csetDesc);
  ccntCharacteristic.addDescriptor(ccntDesc);
  fixvelCharacteristic.addDescriptor(fixvelDesc);
  regvelCharacteristic.addDescriptor(regvelDesc);

  // assign event handlers for characteristic
  nsetCharacteristic.setEventHandler(BLEWritten, nsetCharacteristicWritten);
  csetCharacteristic.setEventHandler(BLEWritten, csetCharacteristicWritten);
  fixvelCharacteristic.setEventHandler(BLEWritten, fixvelCharacteristicWritten);
  // set an initial value for the characteristic
  nsetCharacteristic.setValue(String(n_set));
  ncntCharacteristic.setValue(String((float)count / 2));
  csetCharacteristic.setValue(String(c_set));
  ccntCharacteristic.setValue(String(it_c));
  fixvelCharacteristic.setValue(String(fixvel));
  // add the characteristic to the service
  ledService.addCharacteristic(nsetCharacteristic);
  ledService.addCharacteristic(ncntCharacteristic);
  ledService.addCharacteristic(csetCharacteristic);
  ledService.addCharacteristic(ccntCharacteristic);
  ledService.addCharacteristic(fixvelCharacteristic);
  ledService.addCharacteristic(regvelCharacteristic);

  // add service
  BLE.addService(ledService);
  BLE.advertise();
  cnt_lim = n_set;
  cnt_rot = &cnt_main_rot;
  /* Initialize PCNT event queue and PCNT functions */
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  pcnt_example_init(pcnt_unit, n_set);
  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
  // oled_print();
  // oled.update();
  fwdkar = &main_fwd_kar;
  revkar = &main_rev_kar;
  cut = &cut_main;
  vel = &regvel;
  tm = millis();
  xTaskCreatePinnedToCore(oled_update, "OLED UPDATE", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  if (is_intr)
  {
    is_intr = false;
    intr_src = io.interruptSource();
    // Serial.println(intr_src, BIN);
  }

  {
    // if (io.checkInterrupt(al_stop))
    //   Serial.println(io.digitalRead(al_stop) == HIGH ? "al_stop_on" : "al_stop_off");
    // if
    (deb_main_aut.update(intr_src));
    // Serial.println(deb_main_aut.read() ? "main_aut_on" : "main_aut_off");
    // if
    (deb_al_stop.update(intr_src));
    // Serial.println(deb_al_stop.read() ? "al_stop_on" : "al_stop_off");
    // if
    (deb_main_rot1.update(intr_src));
    // Serial.println(deb_main_rot1.read() ? "main_rot1_on" : "main_rot1_off");
    // if
    (deb_main_rot2.update(intr_src));
    // Serial.println(deb_main_rot2.read() ? "main_rot2_on" : "main_rot2_off");
    // if
    (deb_main_kar1.update(intr_src));
    // Serial.println(deb_main_kar1.read() ? "main_kar1_on" : "main_kar1_off");
    // if
    (deb_main_kar2.update(intr_src));
    // Serial.println(deb_main_kar2.read() ? "main_kar2_on" : "main_kar2_off");
    // if
    (deb_main_cut.update(intr_src));
    // Serial.println(deb_main_cut.read() ? "main_cut_on" : "main_cut_off");
    // if
    (deb_sens_kar1.update(intr_src));
    // Serial.println(deb_sens_kar1.read() ? "sens_kar1_on" : "sens_kar1_off");
    // if
    (deb_sens_kar2.update(intr_src));
    // Serial.println(deb_sens_kar2.read() ? "sens_kar2_on" : "sens_kar2_off");
    // if
    (deb_sens_cut.update(intr_src));
    // Serial.println(deb_sens_cut.read() ? "sens_cut_on" : "sens_cut_off");
    // if (deb_al_stop.fell())
    //   Serial.println("al_stop is fell");
    // if (deb_al_stop.rose())
    //   Serial.println("al_stop is rose");
    // if (deb_main_aut.fell())
    //   Serial.println("main_aut is fell");
    // if (deb_main_aut.rose())
    //   Serial.println("main_aut is rose");
    // if (deb_main_cut.fell())
    //   Serial.println("main_cut is fell");
    // if (deb_main_cut.rose())
    //   Serial.println("main_cut is rose");
    // if (deb_main_kar1.fell())
    //   Serial.println("main_kar1 is fell");
    // if (deb_main_kar1.rose())
    //   Serial.println("main_kar1 is rose");
    // if (deb_main_kar2.fell())
    //   Serial.println("main_kar2 is fell");
    // if (deb_main_kar2.rose())
    //   Serial.println("main_kar2 is rose");
    // if (deb_al_stop.fell()) Serial.println("al_stop is fell");
    // if (deb_al_stop.rose()) Serial.println("al_stop is rose");
    // if (deb_al_stop.fell()) Serial.println("al_stop is fell");
    // if (deb_al_stop.rose()) Serial.println("al_stop is rose");
    // if (deb_al_stop.fell()) Serial.println("al_stop is fell");
    // if (deb_al_stop.rose()) Serial.println("al_stop is rose");
    // if (deb_al_stop.fell()) Serial.println("al_stop is fell");
    // if (deb_al_stop.rose()) Serial.println("al_stop is rose");
    // if (deb_al_stop.fell()) Serial.println("al_stop is fell");
    // if (deb_al_stop.rose()) Serial.println("al_stop is rose");
    if (intr_src != 0)
      intr_src = 0;
  }
  res = xQueueReceive(pcnt_evt_queue, &evt, 0);
  if (res == pdTRUE)
  {
    pcnt_get_counter_value(pcnt_unit, &count);
    // if (evt.status & PCNT_EVT_ZERO)
    // {
    //   Serial.println("ZERO EVT");
    // }
    if (evt.status & PCNT_EVT_THRES_0)
    {
      // Serial.println("THRES0 EVT");
      // fix_vel = true;
      vel = &fixvel;
      setvel();
    }
    if (evt.status & PCNT_EVT_THRES_1)
    {
      // Serial.println("H_LIM EVT");
      *cnt_rot = false;
      // fix_vel = false;
      vel = &regvel;
      setvel();
    }
  }
  else
  {
    pcnt_get_counter_value(pcnt_unit, &count);
    // ESP_LOGI(TAG, "Current counter value :%d", count);
  }
  if (!deb_sens_kar1.read())
  {
    if (cnt_lim != n_set)
    {
      cnt_lim = n_set;
      changing_pcnt();
    }
  }
  if (!deb_sens_kar2.read())
  {
    if (cnt_lim != (n_set + 1))
    {
      cnt_lim = n_set + 1;
      changing_pcnt();
    }
  }
  if (deb_main_aut.read())
  {
    if (deb_main_aut.rose())
    {
      if (it_c == c_set)
        it_c = 0;
      cut_aut = false;
      fwdkar = &main_fwd_kar;
      revkar = &main_rev_kar;
      cut = &cut_main;
      // cnt_aut_rot = aut_fwd_kar = aut_rev_kar = cut_aut = false;
    }
    aut_mode_perm = deb_sens_kar1.read() != deb_sens_kar2.read();
    rot = !deb_main_rot1.read();
    // if (cnt_lim != n_set)
    // {
    //   cnt_lim = n_set;
    //   changing_pcnt();
    // }
    if (deb_main_rot2.fell())
    {
      if (count != 0)
        pcnt_counter_clear(pcnt_unit);
      cnt_main_rot = true;
    }
    if (cnt_rot != &cnt_main_rot)
      cnt_rot = &cnt_main_rot;
    main_fwd_kar = !deb_main_kar1.read() && deb_sens_kar1.read() && deb_main_kar2.read();
    main_rev_kar = !deb_main_kar2.read() && deb_sens_kar2.read() && deb_main_kar1.read();
    cut_main = (deb_main_cut.fell() || cut_main) && !deb_sens_cut.fell();
  }
  else if (aut_mode_perm)
  {
    if (deb_main_aut.fell())
    {
      // rot = cnt_main_rot = main_fwd_kar = main_rev_kar = cut_main = false;
      fwdkar = &aut_fwd_kar;
      revkar = &aut_rev_kar;
      cut = &cut_aut;
      pcnt_counter_clear(pcnt_unit);
      cnt_aut_rot = true;
    }
    cnt_aut_rot = (it_c < c_set) && ((deb_sens_kar1.fell() || deb_sens_kar2.fell()) || cnt_aut_rot);
    if (cnt_rot != &cnt_aut_rot)
      cnt_rot = &cnt_aut_rot;
    if (!deb_sens_cut.fell())
      cut_aut = /* ((!aut_fwd_kar && mcp.digitalRead(FWD_KAR)) || (!aut_rev_kar && mcp.digitalRead(REV_KAR)))  */
          (!cnt_aut_rot && fwd_rot_out) || cut_aut;
    else if (cut_aut)
    {
      cut_aut = false;
      it_c = it_c < c_set ? it_c + 1 : it_c;
      if (it_c < c_set)
      {
        pcnt_counter_clear(pcnt_unit);
        // cnt_aut_rot = true;
      }
    }
    aut_fwd_kar = (((!cut_aut && fwd_cut_out && !deb_sens_kar2.read()) || aut_fwd_kar) && deb_sens_kar1.read());
    aut_rev_kar = (((!cut_aut && fwd_cut_out && !deb_sens_kar1.read()) || aut_rev_kar) && deb_sens_kar2.read());
  }

  if (!deb_al_stop.read())
  {
    if (*cnt_rot)
      resume_cnt();
    else
      pause_cnt();
    fwd_rot_out = rot || *cnt_rot;
    fwd_kar_out = *fwdkar;
    rev_kar_out = *revkar;
    fwd_cut_out = *cut;
  }
  else
  {
    if (deb_al_stop.rose())
    {
      if (cnt_main_rot)
        cnt_main_rot = false;
      if (cut_main)
        cut_main = false;
    }
    pause_cnt();
    fwd_rot_out = false;
    fwd_kar_out = false;
    rev_kar_out = false;
    fwd_cut_out = false;
  }
  if (fwd_rot_out != tmp_fwd_rot_out)
  {
    tmp_fwd_rot_out = fwd_rot_out;
    mcp->digitalWrite(FWD_ROT, tmp_fwd_rot_out);
  }
  if (fwd_kar_out != tmp_fwd_kar_out)
  {
    tmp_fwd_kar_out = fwd_kar_out;
    mcp->digitalWrite(FWD_KAR, tmp_fwd_kar_out);
  }
  if (rev_kar_out != tmp_rev_kar_out)
  {
    tmp_rev_kar_out = rev_kar_out;
    mcp->digitalWrite(REV_KAR, tmp_rev_kar_out);
  }
  if (fwd_cut_out != tmp_fwd_cut_out)
  {
    tmp_fwd_cut_out = fwd_cut_out;
    mcp->digitalWrite(FWD_CUT, tmp_fwd_cut_out);
  }
  if ((millis() - tm) >= 300)
  {
    tm = millis();    
    // static int16_t lc_couunt = 0;
    // if (lc_couunt != count) {
    //   lc_couunt = count;
    //   ncntCharacteristic.setValue(String((float)count / 2));
    // }
    // static int16_t lc_it_c = 0;
    // if (lc_it_c != it_c) {
    //   lc_it_c = it_c;
    //   ccntCharacteristic.setValue(String(lc_it_c));
    // }
    regvel = map(analogRead(36), 0, 4095, 0, 100);
    setvel();
  }
}