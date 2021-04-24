#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <EasyNTPClient.h>
#include <RTClib.h>
RTC_DS3231 rtc;  
#include <FastLED.h>   //  3.2.6
#include <Wire.h>
/////// DHT 22 
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN D4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22     // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);
////BME280
#include <SparkFunBME280.h>
BME280 bme280;
/////////////////////SETTINGS//////////////////
const char* host = "esp8266-webupdate";
const char *ssid = "ssid";             // your network SSID (name)
const char *password = "password";         // your network password

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
EasyNTPClient ntpClient(udp, "pool.ntp.org", (3 * 60 * 60)); //заменить первую цифру на свой часовой пояс, 3 для МСК

/////////////////////Domoticz
#include <PubSubClient.h>
const char* mqtt_server = "";
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_topic = "domoticz/in";
WiFiClient espClient;
PubSubClient client(espClient);
const int idxdht = 29; /* must be the same as virual sensor idx in Domotocz */
const int idxbme = 30; /* must be the same as virual sensor idx in Domotocz */
char mqttbuffer_dht[60];
char mqttbuffer_bme[60];
float tempS,humS;                 // переменные dht22 для мониторинга
float tempH,humH,presH;           // переменные bme280 для мониторинга
int delay_monotoring = 60;        // как часто отправлять значения датчиков
unsigned long timing;             // переменная для мониторинга
static bool Monitoring = true;    // включаем мониторинг, иначе false

/////////////////////////
int UpdatePeriod = 12;             // период в часах, как часто обновлять время из интернета, лучше не пишите маленькие числа:)
int Period = UpdatePeriod * 3600;  //вычисление секунд
int brg = 5000;                    // как часто проверять изменение по датчику освещенности в мс, 10000 соответствуют 10сек
byte cnctd = 20;                   // кол-во попыток подключиться к роутеру, 2 запроса в сек
byte type_brg = 0;                 // выбрать тип датчика, 0 - аналог, 1 - цифровой
/////////////////////////////////////////////
#define LEDS_IN_SEGMENT 2     // задаём сколько у нас светодиодов в сегменте
#define DOTS_NUM 2            // задаём сколько у нас разделительных точек
#define NUM_LEDS (LEDS_IN_SEGMENT * 28 + DOTS_NUM) // вычисляем кол-во светодиодов
#define NUM_COLORS 16         // количество цветов
#define COLOR_CHANGE 3        // смена цвета ( 0 - никогда, 1 - раз в минуту, 2 - каждые десять минут, 3 - каждый час, 4 - каждые десять часов)
#define max_bright 255        // максимальная яркость (0 - 255)
#define min_bright 2        // минимальная яркость (0 - 255) 10
#define bright_constant 1023  // константа усиления от внешнего света (0 - 1023), чем МЕНЬШЕ константа, тем "резче" будет прибавляться яркость 1023
#define coef 0.4              // коэффициент фильтра (0.0 - 1.0), чем больше - тем медленнее меняется яркость 0.4
#define auto_bright 1         // автоматическая подстройка яркости от уровня внешнего освещения (1 - включить, 0 - выключить)
/////////////////////////////////////////////
#define COLOR_ORDER GRB  // тип ленты
#define LED_PIN 6  // пин дата от ленты
#define BRI_PIN A0 // PIN фоторезистора
/////////////////////////////////////////////
CRGB leds[NUM_LEDS];  // определение СД ленты
uint8_t  digits[] = { // определяем символы для отображения
  // код начинается с 0b0, далее идут 7 цифр, каждая цифра это номер фрагмента, 1 - включен, 0- отключен
  // далее указан получающийся символ и порядковый номер в массиве

  0b00111111,     // Символ 0          0
  0b00100001,     // Символ 1          1
  0b01110110,     // Символ 2          2
  0b01110011,     // Символ 3          3
  0b01101001,     // Символ 4          4
  0b01011011,     // Символ 5          5
  0b01011111,     // Символ 6          6
  0b00110001,     // Символ 7          7
  0b01111111,     // Символ 8          8
  0b01111011,     // Символ 9          9
  0b01111000,     // Символ * градус  10
  0b00011110,     // Символ C         11
  0b00000000,     // Без символа      12
  0b01000000,      // Символ -         13
  0b01111100,      // Символ P         14
  0b01001101,     // Символ h         15
};
/////////////////////////////////////////////
//bool Dot = true;    // переменная для точек
int last_digit = 0; // последний символ равен нулю
byte set_light;     // переменная для освещенности
byte brightness;    // переменная для освещенности
int new_bright, new_bright_f; // переменная для освещенности
unsigned long bright_timer, off_timer; // переменная для освещенности
float celsius,pressure,humidity; // переменные для библиотеки датчика bme280
byte prs = 0;                  // 0 - не показываем символ давления, 1 - показать
float celsiusS,humidityS;      // переменные для библиотеки датчика dht22

/////////////////////////////////////////////
//управление цветом
//int ledColor = 0x00FFFF;    // цвет в hex
//long ledColor = CRGB::Blue; // цвет в hex
CRGB ledColor = CRGB::Blue;   // цвет в hex
// массив цветов, для рандом при включенном режиме cylon(); ledColor =  ColorTable[random(16)];
CRGB ColorTable[NUM_COLORS] = { // Таблица цветов
  CRGB::Amethyst,
  CRGB::Aqua,
  CRGB::Blue,
  CRGB::Chartreuse,
  CRGB::DarkGreen, 
  CRGB::DarkMagenta,
  CRGB::DarkOrange, 
  CRGB::DeepPink,
  CRGB::Fuchsia,
  CRGB::Gold,
  CRGB::GreenYellow,
  CRGB::LightCoral,
  CRGB::Tomato,
  CRGB::Salmon,
  CRGB::Red,
  CRGB::Orchid
};


bool hasChange = false;
//uint8_t periodDisplay[] = {4, 2, 2, 2}; // массив вывода секунды
//uint8_t orderDisplay[] = {0, 1, 0, 2, 0, 3}; // порядка вывода
uint8_t periodDisplay[] = {50, 2, 2, 2, 2, 2}; // массив вывода  
uint8_t orderDisplay[] = {0, 1, 2, 3, 4, 5}; // порядка вывода
//uint8_t orderDisplay[] = {0}; // порядка вывода
uint8_t mode = 2;   //что выводим
/*
0 - часы, хотим выводить их 4 секунды
1 - температура дома, 2 сек
2 - температура улицы, 2 сек
3 - давление, 2 секунды
*/
bool showDot = false;

//////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  Serial.println();
  Serial.println("Booting Sketch...");
  LEDS.setBrightness(2);
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS); // подключение ленты
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
    wifi();
    syncTime();
    dht.begin();
    Serial.println(F("DHT22 start!")); 
    bme280.setI2CAddress(0x76); //Connect to a second sensor
    if (bme280.beginI2C() == false) Serial.println("Sensor B connect failed");
    // Setup MQTT
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

}
/////////////////////////////////////////////
void loop() {
  BrightnessCheck();
  if (millis() % 1000 < Period - 1)
  {
    body();
  }
  else {
    syncTime();
  }
    httpServer.handleClient();
    
    if (Monitoring) {
    if (millis() - timing > delay_monotoring * 1000) {
      timing = millis();
      send_to_domoticz();
      }
     }
  
}
/////////////////////////////////////////////////////////
void body() {
  static uint32_t ch_tmr = millis();

  if ((millis() - ch_tmr) < (periodDisplay[orderDisplay[mode]] * 1000))
  {
    switch (orderDisplay[mode]) {
      case 0:
        displayTime(timeToString());
        displayTime(timeToStringDots());
        break;
      case 1:
        displayTime(TemperToString());
        break;
      case 2:
        displayTime(HumToString());
        break;
      case 3:
        displayTime(PressToString());
        break;
      case 4:
        displayTime(Sensor2Temper());
        break;  
       case 5:
        displayTime(Sensor2Hum());
        break; 
    }
  }
  else
  {
    ch_tmr = millis();
    mode++;
    if (sizeof(orderDisplay) == mode) mode = 0;
  }
FastLED.show();

}
/////////////////////////////////////////////////////////
String TemperToString(){
  static uint32_t tmr1 = millis() - periodDisplay[1] * 1000;
  if (millis() - tmr1 < (periodDisplay[1] * 1100)) return "";
  tmr1 = millis();
  hasChange = true;
  Dots_off(); // выключаем точки
   
  bme280.beginI2C();
  tempH = (bme280.readTempC());
  int celsius = tempH;
  Serial.println ((String) celsius + " | " + tempH);
  Digit(digits[10], (NUM_LEDS - LEDS_IN_SEGMENT * 7)); // символ градуса
  int digit = abs (celsius % 10);
   Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM)); // если впереди ноль, то выключаем его
  digit = celsius / 10;
  if (digit == 0)Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // если впереди ноль, то выключаем его
  else
    Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // иначе показываем как есть
    Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 14)); // отключаем 1 сегмент
 }


String HumToString()
{
  static uint32_t tmr1 = millis() - periodDisplay[2] * 1000;
  if (millis() - tmr1 < (periodDisplay[2] * 1100)) return "";
  tmr1 = millis();
  hasChange = true;
  
  bme280.beginI2C();
  humH = (bme280.readFloatHumidity());
  
//  humH = (dht.readHumidity());
   Dots_off(); // выключаем точки
  int humidity = humH;
   Serial.println ((String) humidity + " | " + humH);
  Digit(digits[15], (NUM_LEDS - LEDS_IN_SEGMENT * 7)); // символ давления
    int digit = abs (humidity % 10);
  Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM)); // если впереди ноль, то выключаем его
  digit = humidity / 10;
  if (digit == 0)Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // если впереди ноль, то выключаем его
  else
    Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // иначе показываем как есть
    Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 14)); // отключаем 1 сегмент
}

String PressToString()
{
  static uint32_t tmr1 = millis() - periodDisplay[3] * 1000;
  if (millis() - tmr1 < (periodDisplay[3] * 1100)) return "";
  tmr1 = millis();

  hasChange = true;  
  bme280.beginI2C();
  presH = (bme280.readFloatPressure() * 0.0075);
  int pressure = presH;
  //  bmp280.getMeasurements(temperature, pressure, altitude);
  //  int davlenie = pressure * 0.75;

  //  presH = pressure * 0.75;

  Serial.println ((String) pressure + " | " + presH);

  Dots_off(); // выключаем точки

  int digit = pressure % 10;
  Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 7));

  digit = pressure % 100 / 10;
  Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 14));

  digit = pressure / 100;
  Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM));
  switch (prs) {
    case 1:
      Digit(digits[14], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // показываем символ P
      break;
    default:
      Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // отключаем первый сегмент
  }


}
/////////////////////////////////////////////////////////
String Sensor2Temper(){
  static uint32_t tmr1 = millis() - periodDisplay[1] * 1000;
  if (millis() - tmr1 < (periodDisplay[1] * 1100)) return "";
  tmr1 = millis();
  hasChange = true;
  Dots_off(); // выключаем точки
 tempS= (dht.readTemperature());
  int celsiusS = tempS;
  Serial.println ((String) celsiusS + " | " + tempS);
  Digit(digits[10], (NUM_LEDS - LEDS_IN_SEGMENT * 7)); // символ градуса
  int digit = abs (celsiusS % 10);
   Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM)); // если впереди ноль, то выключаем его
  digit = celsiusS / 10;
  if (digit == 0)Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // если впереди ноль, то выключаем его
  else
    Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // иначе показываем как есть
    Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 14)); // отключаем 1 сегмент
 }
String Sensor2Hum()
{
  static uint32_t tmr1 = millis() - periodDisplay[2] * 1000;
  if (millis() - tmr1 < (periodDisplay[2] * 1100)) return "";
  tmr1 = millis();
  hasChange = true;
  
 humS = (dht.readHumidity());
   Dots_off(); // выключаем точки
  int humidityS = humS;
   Serial.println ((String) humidityS + " | " + humS);
  Digit(digits[15], (NUM_LEDS - LEDS_IN_SEGMENT * 7)); // символ давления
    int digit = abs (humidityS % 10);
  Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM)); // если впереди ноль, то выключаем его
  digit = humidityS / 10;
  if (digit == 0)Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // если впереди ноль, то выключаем его
  else
    Digit(digits[digit], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM)); // иначе показываем как есть
    Digit(digits[12], (NUM_LEDS - LEDS_IN_SEGMENT * 14)); // отключаем 1 сегмент
}

String timeToString()
{
  static uint32_t tmr1 = millis();
  if (millis() - tmr1 < 500) return "";
  tmr1 = millis();

  char temp[10];
  uint32_t nt = millis() / 1000; //получили секунды
  static int8_t old_s = 255;
  int8_t s = nt % 60;
  if (s != old_s)
  {
    old_s = s;
    hasChange = true;
  }
  else return "";
  TimeToArray();
 /* int8_t m = nt / 60 % 60;
  int8_t h = nt / 3600 % 24;
  snprintf(temp, 10, "%02d:%02d:%02d", h, m, s);
  return String(temp);*/
}

/////////////////////////////////////////////
void TimeToArray() { // вывод времени на экран
  int Now = GetTime(); // получаем время
  boolean color_change_flag = false;
   
  for (int i = 1; i <= 4; i++) { // 4 сегмента
    int digit = Now % 10; // получаем последнюю цифру в времени
    int cursor = NUM_LEDS - i * LEDS_IN_SEGMENT * 7;
    if (i > 2) {  
      cursor -= DOTS_NUM;
    }
    if ( i == 4 & digit == 0)Digit(digits[12], cursor); // если впереди ноль, то выключаем его, например 01:23 будет как 1:23
    else
      Digit(digits[digit], cursor);                     // иначе показываем символ

    if ( i == COLOR_CHANGE) {                           // как часто менять цвет
      if (digit != last_digit) {
          color_change_flag = true;
          last_digit = digit;
         }
      }
    Now /= 10;
  };
if (color_change_flag) ledColor =  ColorTable[random(NUM_COLORS)];     // случайный цвет из таблицы)  
};


/////////////////////////////////////////////
int GetTime() {
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
 //  Serial.println((String)hour + ":" + minute + ":" + second);
 //delay(100);
  return (hour * 100 + minute);
};

/////////////////////////////////////
String timeToStringDots()
{
  static uint32_t tmr1 = millis();
  if (millis() - tmr1 < 500) return "";
  tmr1 = millis();

  char temp[10];
  uint32_t nt = millis() / 1000; //получили секунды
  static int8_t old_s = 255;
  int8_t s = nt % 60;
  showDot = !showDot;
  hasChange = true;

  int8_t m = nt / 60 % 60;
  int8_t h = nt / 3600 % 24;
  if (showDot)
   // snprintf(temp, 10, "%02d:%02d:%02d", h, m, s);\
   { // показ точек
    for (uint8_t i = 0; i < DOTS_NUM; i++) {
      leds[(LEDS_IN_SEGMENT * 14) + i] = ledColor;
    } 
  else
  //  snprintf(temp, 10, "%02d %02d %02d", h, m, s);
  Dots_off();
  return String(temp);
}
////////////////////////////////////////////

void Dots_off()  { // отключаем точки принудительно, где не нужны
  for (uint8_t i = 0; i < DOTS_NUM; i++) {
    leds[(LEDS_IN_SEGMENT * 14) + i] = 0x000000;
  }
}

////////////////////////////////////////////
void displayTime(String s)
{
  if (!hasChange) return;
  Serial.println(s);
  hasChange = false;
}

//////////////////////////////////////////////////////////
void wifi() {
 WiFi.mode(WIFI_STA);
 WiFi.begin(ssid, password);
  byte trys = 0;
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {
    trys++;
    //    delay(500);
    NoSignal();
    Serial.print(".");
    if (trys > cnctd)
    {
      Serial.println("Нет связи с роутером при старте!");
      return;
    }
  }
  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);
 }

/////////////////////////////////////////////
void syncTime() {
 // WiFi.begin(ssid, password);
  byte trys = 0;
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {
    trys++;
    //    delay(500);
    NoSignal();
    Serial.print(".");
    if (trys > cnctd)
    {
      Serial.println("Нет связи с роутером при старте!");
      return;
    }
  }
  long ntpTime = ntpClient.getUnixTime();
  if (ntpTime > 1609459200) {
    rtc.adjust(DateTime(ntpTime));
    Serial.println("");
    Serial.println("Время записано!");
  }
  else
  {
    Serial.println("");
    Serial.println("Отказ в записи! Время получено неправильное!");
//    GetTime();
  }
}

/////////////////////////////////////////////
void Digit (uint8_t digit, uint8_t cursor) { // функция отображения символов
  for (uint8_t mask = 0b01000000; mask > 0; mask = mask >> 1) {
    for (uint8_t i = 0; i < LEDS_IN_SEGMENT; i++) {
      leds[cursor] = (digit & mask) ? ledColor : CRGB (0, 0, 0);
      cursor ++;
    }
  }
}

/////////////////////////////////////////////
void NoSignal() { // анимация загрузки :)
  Digit(digits[13], (NUM_LEDS - LEDS_IN_SEGMENT * 28 - DOTS_NUM));
  FastLED.show(); delay(125);
  Digit(digits[13], (NUM_LEDS - LEDS_IN_SEGMENT * 21 - DOTS_NUM));
  FastLED.show(); delay(125);
  Digit(digits[13], (NUM_LEDS - LEDS_IN_SEGMENT * 14));
  FastLED.show(); delay(125);
  Digit(digits[13], (NUM_LEDS - LEDS_IN_SEGMENT * 7));
  FastLED.show(); delay(125);
  LEDS.clear();
}
/////////////////////////////////////////////
void BrightnessCheck() { // функция освещенности
  static uint32_t last_br = millis();
  if ((millis() - last_br) < brg) return;
  last_br = millis();
  if (auto_bright) {                         // если включена адаптивная яркость
    if (millis() - bright_timer > 100) {     // каждые 100 мс
      bright_timer = millis();               // сбросить таймер
      switch (type_brg) {
        case 1:
          new_bright = map(digitalRead(BRI_PIN), 0, bright_constant, min_bright, max_bright);   // считать показания с фоторезистора, перевести диапазон
          break;
        default:
          new_bright = map(analogRead(BRI_PIN), 0, bright_constant, min_bright, max_bright);   // считать показания с фоторезистора, перевести диапазон
      }
      //      Serial.println((String)"Освещенность: " + new_bright);
      new_bright = constrain(new_bright, min_bright, max_bright);
      new_bright_f = new_bright_f * coef + new_bright * (1 - coef);
      Serial.println((String)"Освещенность: " + new_bright);
      Serial.println((String)"Яркость: " + new_bright_f);
      LEDS.setBrightness(new_bright_f);      // установить новую яркость
    }
  }
};
void send_to_domoticz () {

  // Reconnect if needed
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

 sprintf(mqttbuffer_dht, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;%3.1f;0\" }", idxdht, tempS, humS);
  
  // send temperature and humidity to the MQTT topic
 client.publish(mqtt_topic, mqttbuffer_dht);
  // Debug message
 Serial.println(mqttbuffer_dht);
 // 84 TEMP_HUM_BARO 0.0;50;1;1010;1    svalue=TEMP;HUM;HUM_STATUS;BARO;BARO_FCST
  sprintf(mqttbuffer_bme, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;%3.1f;0;%3.0f;0\" }", idxbme, tempH, humH, presH);
    // send temperature and humidity to the MQTT topic
  client.publish(mqtt_topic, mqttbuffer_bme);
  // Debug message
  Serial.println(mqttbuffer_bme);
}


// Handle recieved MQTT message, just print it
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Reconnect to MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("NodeMCUClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
