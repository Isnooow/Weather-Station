  //Library Firebase
#include <WiFiManager.h> 
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

  #include <Firebase_ESP_Client.h>
  #include <addons/RTDBHelper.h>
  #include <addons/TokenHelper.h>

  #define DATABASE_URL "***************.asia-southeast1.firebasedatabase.app" //Link Firebase Console

  //Library Waktu
  #include <NTPClient.h>
  #include <WiFiUdp.h>
  const long utcOffsetInSeconds = 25200;
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "id.pool.ntp.org", utcOffsetInSeconds);  
  //Week Days
  String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  //Month names
  String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

  //Set Firebase Lib
  FirebaseData fbdo;  
  FirebaseAuth auth;
  FirebaseConfig config;
  unsigned long dataMillis = 0;
  WiFiManager wm;

  String localAddress = "Prototype";

  //Wind Direction
  #include <SoftwareSerial.h>
  SoftwareSerial dataserial(D4, D5); // D4,D5
  String data, arah_angin, s_angin;
  int a, b;

  //for dht11
  #include "DHT.h"
  #define DHTPIN D3
  #define DHTTYPE DHT11 
  DHT dht(DHTPIN, DHTTYPE);

  //for rainsensor
  #define rainAnalog A0
  long waktugerimis = 0;
  long int temp_Hujan = 0 ;

  //for bh1750
  #include <BH1750.h> 
  BH1750 lightMeter;

  //for bmp280
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_BMP280.h>
  #define BMP_SCK  (13)
  #define BMP_MISO (12)
  #define BMP_MOSI (11)
  #define BMP_CS   (10) 
  Adafruit_BMP280 bmp;

  //display
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 16, 2);

  //symbol derajat lcd
  byte Cok[8] =
    {
    0b01000,
    0b10100,
    0b01000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
    };

  ///Setup Sensor Tambahan.///
  //Rain Guage
  const int pin_interrupt = D7; // Gunakan pin D5 pada NodeMCU, GPIO14 pada ESP32
  long int jumlah_tip = 0; 
  long int temp_jumlah_tip = 0;
  float curah_hujan_hari_ini = 0.00;
  float curah_hujan = 0.00;
  float milimeter_per_tip = 0.70;
  String cuaca = "Berawan"; //kondisi awal cuaca
  volatile boolean flag = false;
  unsigned long int oldtime = 0;

  //Anemometer
  int GPIO_pulse = D6; // NodeMCU = D5
  volatile byte rpmcount = 0; // count signals
  volatile unsigned long last_micros = 0;
  unsigned long timeold = 0;
  unsigned long timemeasure = 25.00; // seconds
  float rpm = 0 ; float rps = 0;// frequencies
  float radius = 0.1; // meters - measure of the length of each anemometer wing
  float velocity_kmh = 0; // km/h
  float velocity_ms = 0; //m/s
  float omega = 0; // rad/s
  float calibration_value = 2.0;
  volatile boolean flag1 = false;
  volatile byte rpmcountnow = 0;



  //Variabel
  String suhu, temp, rain, lux, prss, hari, jam, hujan, rpm_data, ms_data, kmh_data;
  int counter = 0;
  // String currentDate;
  byte Hujan, Kelembaban, Suhu, Cahaya;
  unsigned sensor;
  float Pressure;
  unsigned long int WaktuPrint = 0;
  unsigned long oldtimewaktu;

  void ICACHE_RAM_ATTR hitung_curah_hujan()
  {
    flag = true;
  }

  void ICACHE_RAM_ATTR rpm_anemometer()
  {
    flag1 = true;
  }

void setup() {
  Serial.begin(9600);
  dataserial.begin(9600);
  timeClient.begin();
  Wire.begin();
  dht.begin();
  lightMeter.begin();
  lcd.begin();

  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                 Adafruit_BMP280::STANDBY_MS_500); 
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lcd.backlight();
  lcd.createChar(0,Cok);

  pinMode(rainAnalog,INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(pin_interrupt, INPUT);
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);

  // conect to wifi and firebase               
   bool res;

    res = wm.autoConnect("Mobcomm Auto WiFi","iotmobcomm23"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        ESP.restart();
    } 
    else { 
        Serial.println("connected...yeey :)");
    }


  Serial.println(); 
  lcd.setCursor(6,2);
  lcd.print("Berhasil");
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  config.database_url = DATABASE_URL;
  config.signer.test_mode = true;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  attachInterrupt(digitalPinToInterrupt(pin_interrupt), hitung_curah_hujan, FALLING);
  detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // Initialize the interrupt pin
}



void loop() {
    unsigned long int timeNow = micros();
    unsigned long int current_time = millis();
    timeClient.update();
    //Dht11
    Kelembaban = dht.readHumidity();
    Suhu = dht.readTemperature();

    //rainsensor  
    sensor = ( 100.097 - ( (analogRead(rainAnalog)/1023.00) * 100.00 ) );
    Hujan = sensor;

    //cahaya BH170
    Cahaya = lightMeter.readLightLevel();

    //BMP280
    Pressure = (bmp.readPressure());

    //Sensor Rain Gauge
    if (flag == true) // don't really need the == true but makes intent clear for new users
    {
    curah_hujan += milimeter_per_tip; // Akan bertambah nilainya saat tip penuh
    jumlah_tip++;
    delay(500);
    flag = false; // reset flag
    }
    curah_hujan_hari_ini = jumlah_tip * milimeter_per_tip;
    if (curah_hujan_hari_ini <= 0.00 && curah_hujan_hari_ini <= 0.50 && Hujan == 0)
    {
      cuaca = "Berawan           ";
    }
    if (Hujan > 0 )
    {
      cuaca ="Gerimis";
        if((Hujan == temp_Hujan) && (current_time - waktugerimis >= 20000) && (jumlah_tip == 0) )
        {
           Hujan = 0;
           waktugerimis = current_time;
        }
    }
    if (curah_hujan_hari_ini > 0.50 && curah_hujan_hari_ini <= 20.00)
    {
      cuaca = "Hujan Ringan      ";
    }
    if (curah_hujan_hari_ini > 20.00 && curah_hujan_hari_ini <= 50.00)
    {
      cuaca = "Hujan Sedang      ";
    }
    if (curah_hujan_hari_ini > 50.00 && curah_hujan_hari_ini <= 100.00)
    {
      cuaca = "Hujan Lebat       ";
    }
    if (curah_hujan_hari_ini > 100.00 && curah_hujan_hari_ini <= 150.00)
    {
      cuaca = "Hujan Sangat Lebat";
    }
    if (curah_hujan_hari_ini > 150.00)
    {
      cuaca = "Hujan ekstrem     ";
    }
    if ((jumlah_tip == temp_jumlah_tip) && (current_time - oldtime >= 30000)){
     jumlah_tip = 0;
     oldtime = current_time;
    }

    //Sensor Arah Angin
    if (dataserial.available()) // Jika ada data yang diterima dari sensor arah angin
    {
    data = dataserial.readString();
    a = data.indexOf("*");
    b = data.indexOf("#");
    s_angin = data.substring(a + 1, b);

    if (s_angin.equals("1")) {
      arah_angin = "utara     ";
    }
    if (s_angin.equals("2")) {
      arah_angin = "timur laut";
    }
    if (s_angin.equals("3")) {
      arah_angin = "timur     ";
    }
    if (s_angin.equals("4")) {
      arah_angin = "tenggara  ";
    }
    if (s_angin.equals("5")) {
      arah_angin = "selatan   ";
    }
    if (s_angin.equals("6")) {
      arah_angin = "barat daya";
    }
    if (s_angin.equals("7")) {
      arah_angin = "barat     ";
    }
    if (s_angin.equals("8")) {
      arah_angin = "barat laut";
    }
    //Serial.println(arah_angin);
  }

    //Sensor Anemometer
    if (flag1 == true) {
      if ((timeNow - last_micros) >= 5000) {
        rpmcount++;
        last_micros = timeNow;
      }
      flag1 = false;
    }

    if ((current_time - timeold) >= timemeasure * 100) {
      detachInterrupt(digitalPinToInterrupt(GPIO_pulse));
      rps = float(rpmcount) / float(timemeasure);
      rpm = 60 * rps;
      omega = 2 * PI * rps;
      velocity_ms = omega * radius * calibration_value;
      velocity_kmh = velocity_ms * 3.6;
      timeold = current_time;
      
      if ((rpmcountnow==rpmcount) && (current_time - oldtimewaktu >= 50) ){
        rpmcount=0;
        oldtimewaktu = current_time;
      }
      
      attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING);
    }
    rpmcountnow=rpmcount;


    //Waktu
    time_t epochTime = timeClient.getEpochTime();
    String waktu = timeClient.getFormattedTime();
    // Serial.print(waktu);
    String weekDay = weekDays[timeClient.getDay()];
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon+1;
    String currentMonthName = months[currentMonth-1];
    int currentYear = ptm->tm_year+1900;

    //currentDate = String(weekDay) + "," + String(monthDay) + "-" + String(currentMonthName) + "-" + String(currentYear) + " " + String(waktu);
    jam = String(waktu);
    hari = String(weekDay) + "," + String(monthDay) + "-" + String(currentMonthName) + "-" + String(currentYear);

    //Ubah Tipe Data
    ubah_tipedata();

    //Kirim Database
    if (current_time - dataMillis >= 3000)
    {
      Kirimdatafirebase();
      dataMillis = current_time;
    }

    //Print LCD
    print_lcd();

    //Print Serial 
    if (current_time - WaktuPrint >= 2000)
    {
      print_nilai();
      WaktuPrint = current_time;
    }

    temp_Hujan = Hujan;
    temp_jumlah_tip = jumlah_tip;

    // if(counter==7200){
    // Serial.println("Reset..");
    // ESP.restart();
    // }
    // counter++;
}

void ubah_tipedata(){

  temp = String(Kelembaban); 
  suhu = String(Suhu);
  lux = String(Cahaya);
  prss = String(Pressure);
  rain = String(Hujan);
  hujan = String(curah_hujan_hari_ini);
  rpm_data = String(rpm);
  ms_data = String(velocity_ms);
  kmh_data = String(velocity_kmh);
}

void print_nilai (){

  Serial.print("Suhu:");
  Serial.println(Suhu);
  Serial.print("Kelembaban:");
  Serial.println(Kelembaban);
  Serial.print("Raim sensor:");
  Serial.println(Hujan);
  Serial.print("Cahaya:");
  Serial.println(Cahaya);
  Serial.print("Tekanan:");
  Serial.println(Pressure);
  Serial.print("Rain Guage:");
  Serial.println(curah_hujan_hari_ini);
  Serial.print("Cuaca:");
  Serial.println(cuaca);
  Serial.print("Arah Angin:");
  Serial.println(arah_angin);
  Serial.print("rpm:");
  Serial.println(rpm);
  Serial.print("Kecepatan MS:");
  Serial.println(velocity_ms);
  Serial.print("Kecepatan KMH:");
  Serial.println(velocity_kmh);
  Serial.print("Tanggal:");
  Serial.println(hari);
  Serial.print("Waktu:");
  Serial.println(jam);
  Serial.println(rpmcountnow);
  Serial.println(rpmcount);  
  Serial.println(" ");
  Serial.println(" ");
}

void print_lcd(){

  lcd.setCursor(0,0);
  lcd.print("Hujan:");
  lcd.setCursor(6,0);
  lcd.print(Hujan);
  lcd.setCursor(0,1);
  lcd.print("Cahaya:");
  lcd.setCursor(7,1);
  lcd.print(Cahaya);
  lcd.setCursor(0,2);
  lcd.print("Tekanan:");
  lcd.setCursor(8,2);
  lcd.print(Pressure);
  lcd.setCursor(3,3);
  lcd.print("Mobcomm X MBC");
  delay(1000);
  lcd.clear();
 
  lcd.setCursor(0,0);
  lcd.print("Suhu:");
  lcd.setCursor(5,0);
  lcd.print(Suhu);
  lcd.setCursor(7,0);
  lcd.write(0);
  lcd.setCursor(0,1);
  lcd.print("Kelembaban:");
  lcd.setCursor(11,1);
  lcd.print(Kelembaban);
  lcd.setCursor(13,1);
  lcd.print("%");
  lcd.setCursor(0,2);
  lcd.print("Kecepatan Angin:");
  lcd.setCursor(16,2);
  lcd.print(kmh_data);
  lcd.setCursor(3,3);
  lcd.print("Mobcomm X MBC");
  delay(1000);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Arah Angin:");
  lcd.setCursor(11,0);
  lcd.print(arah_angin);
  lcd.setCursor(0,1);
  lcd.print("Cuaca:");
  lcd.setCursor(6,1);
  lcd.print(cuaca);
  lcd.setCursor(3,3);
  lcd.print("Mobcomm X MBC");
  delay(1000);
  lcd.clear();
}

void Kirimdatafirebase(){
  FirebaseJson json;

  json.add("Temperature", suhu);
  json.add("Humadity", temp);
  json.add("Tingkat Gerimis", rain);
  json.add("Tingkat Cahaya", lux);
  json.add("Tekanan Udara", prss);
  json.add("Tanggal", hari);
  json.add("Waktu", jam);
  json.add("Hujan",hujan);
  json.add("Cuaca",cuaca);
  json.add("Arah Angin",arah_angin);
  json.add("Kecepatan Angin rpm", rpm_data);
  json.add("Kecepatan Angin ms", ms_data);
  json.add("Kecepatan Angin kmh", kmh_data);

  if (Firebase.RTDB.pushJSON(&fbdo, "/Node1", &json)) {

    Serial.println(fbdo.dataPath());

    Serial.println(fbdo.pushName());

    Serial.println(fbdo.dataPath() + "/"+ fbdo.pushName());

  }else {
    Serial.println(fbdo.errorReason());
  }

}
