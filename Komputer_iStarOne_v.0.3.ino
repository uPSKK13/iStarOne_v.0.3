// użyj linear regression - ml
// problem funkcji trim w androidzie. Użycie ssid niezalecane - komplikacje lokalizacji andr. 8+
// milis do odczytu adc - stabilność



#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "PCF8574.h"
#include <BTS7960.h>
#include <Wire.h>
#include <Ticker.h>
#include <map>
#include <ESP_EEPROM.h>
#include <SimpleKalmanFilter.h>


const char* ssid = "iStarOne_v.03";
const char* password = "krzysio32";
IPAddress ip(192,168,0,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);


Ticker timer;


const uint8_t L_EN = 1;
const uint8_t R_EN = 2;
const uint8_t L_PWM = 3;
const uint8_t R_PWM = 4;

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
PCF8574 PCF_01(0x20);

WiFiUDP UDP;
unsigned int UDPPort = 3000;

struct EEPROMStruct {
  float   maxpressure;
  float   adc_calibration;
  int     wydatek_cieczy;
  float   rozstawa;
  float   speed_zawracania;
  int     impulsy;
  float   wydatek;
  float pressure_mix;
  float k_press1;
  float k_press2;
  float k_press3;
  float k_spd1;
  float k_spd2;
  float k_spd3;
  int m1,m2,m3,m4,m5,m6,m7,m8,m9,m11,m12,m13,m14,m15,m16,m17,m18,m19,m20;
} eeprom;


const int packetSize = 100;
byte packetBuffer[packetSize];

float k_pressure1=0.1,k_pressure2=0.1,k_pressure3=0.01;
float k_speed1=1,k_speed2=1,k_speed3=0.01;
boolean manual_up=0, manual_down=0;;
int czas=0, czas2=0;
float adc=0, pressure=0;
int value_up, value_down;
float speed_kmh=0;
boolean auto_manual=0, master=0, liczba_sekcji=1, helper=0, helper2=0;
String myData = "";
float cisnienie_reg=0, cisnienie_help;
std::map<float, int> m;
//std::map<float, int> //m;
SimpleKalmanFilter pressureKalmanFilter(k_pressure1, k_pressure2, k_pressure3);
SimpleKalmanFilter speedKalmanFilter(k_speed1, k_speed2, k_speed3);
//float estimated_altitude = pressureKalmanFilter.updateEstimate(a);

void setup() {
  Serial.begin(115200);
  yield();
  
  WiFi.softAPConfig(ip,gateway,subnet);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("WiFi created");  
  EEPROM.begin(sizeof(EEPROMStruct));
  EEPROM.get(0, eeprom);
  Serial.println(eeprom.maxpressure);
  Serial.println(eeprom.adc_calibration);
  PCF_01.begin();

  UDP.begin(UDPPort);

    timer.attach_ms(1,zegar);
    timer.attach_ms(50,readAnalog);

    map_maker();
    motorController.Enable();
    
}


void loop() {
handleUDPServer();
 if(auto_manual==1) { //AUTO
   int speedo;
   if(helper=1){
    helper=0;
    cisnienie_reg = pow(eeprom.wydatek_cieczy*eeprom.rozstawa*speed_kmh,2)*10;
    cisnienie_reg = cisnienie_reg/eeprom.wydatek;
    cisnienie_reg = round(cisnienie_reg*10)/10;
    speedo = m[abs(cisnienie_reg-pressure)]; ///TO NIE ZADZIAŁA, TRZEBA ODWRÓCIĆ WARTOŚCI
   }
   if(cisnienie_reg>pressure){
      motorController.TurnRight(speedo);
   } else if(cisnienie_reg<pressure) {
      motorController.TurnLeft(speedo);
   } else {
     motorController.Stop();
    }
   
  } else {  //MANUAL
    
 }
}

void motor_up() {
  analogWrite(4,HIGH);
  analogWrite(5,LOW);  
  analogWrite(6,m[cisnienie_reg]);  
}

void motor_down() {
  analogWrite(4,LOW);  
  analogWrite(5,HIGH);
  analogWrite(6,m[cisnienie_reg]);  
}

void motor_stop() {
  analogWrite(4,LOW);  
  analogWrite(5,LOW);  
  analogWrite(6,LOW);  
}

void handleUDPServer() {

int cb = UDP.parsePacket();
  if (cb) {
    IPAddress android_ip =  UDP.remoteIP();
    int android_port =  UDP.remotePort();
    UDP.read(packetBuffer, cb);
 
    for(int i = 0; i < cb; i++) {
      myData += (char)packetBuffer[i];
    }

    UDP.beginPacket(android_ip, android_port);
    String Packet = "@";
    Packet += speed_kmh;
    Packet += "#";
    Packet += pressure;
    Packet += "$";
    const char *PacketToSend = Packet.c_str();
    UDP.write(PacketToSend);
    UDP.endPacket();
    aktualizacja(); 
    //Serial.println(ESP.getFreeHeap(),DEC);   
  }

  
}

void aktualizacja() {
  if (myData!="0") {Serial.println(myData);}

                 if (myData.indexOf("data") != -1) { 
                  boolean s1 = convert(myData.substring(0, myData.indexOf("@")));
                  boolean s2 = convert(myData.substring(myData.indexOf("@")+1, myData.indexOf("#")));
                  boolean s3 = convert(myData.substring(myData.indexOf("#")+1, myData.indexOf("$")));
                  boolean s4 = convert(myData.substring(myData.indexOf("$")+1, myData.indexOf("%")));
                  boolean mixed_b = convert(myData.substring(myData.indexOf("%")+1, myData.indexOf("&")));
                  boolean led12 = convert(myData.substring(myData.indexOf("&")+1, myData.indexOf("{")));
                  auto_manual = convert(myData.substring(myData.indexOf("{")+1, myData.indexOf("}")));
                  manual_up = convert(myData.substring(myData.indexOf("}")+1, myData.indexOf("=")));  
                  manual_down = convert(myData.substring(myData.indexOf("=")+1, myData.indexOf("d")));  
                  
                  PCF_01.write(0,s1);     
                  PCF_01.write(2,s3);
                  if(liczba_sekcji) {
                    PCF_01.write(1,s2);
                   PCF_01.write(3,s4);
                  }
                  
                  PCF_01.write(4,led12);
                 

                  Serial.println((String)"s1:"+s1+"  s2:"+s2+"  s3:"+s3+"  s4:"+s4);
                  Serial.println((String)"up: " +manual_up + "  down: " + manual_down);
                  Serial.println(mixed_b);
                  Serial.println(led12);
                  Serial.println(auto_manual);
                }
                
                  else if (myData.indexOf("czujnik") != -1) { 

                       eeprom.maxpressure = (myData.substring(0, myData.indexOf("@"))).toFloat();   
                         
                       boolean kalibracja_pressure = convert(myData.substring(myData.indexOf("@")+1, myData.indexOf("c")));
                       if(kalibracja_pressure){
                         eeprom.adc_calibration = analogRead(A0);
                         Serial.println(eeprom.adc_calibration);
                         
                       }

                       EEPROM.put(0, eeprom);
                       EEPROM.commit();
 
                        Serial.println(eeprom.maxpressure);
                        Serial.println(kalibracja_pressure);
                }

                else if (myData.indexOf("program") != -1) {
                  
                       eeprom.rozstawa = (myData.substring(0, myData.indexOf("@"))).toFloat(); 
                       eeprom.wydatek_cieczy = (myData.substring(myData.indexOf("@")+1, myData.indexOf("#"))).toInt(); //USER VALUE

                       EEPROM.put(0, eeprom);
                       EEPROM.commit();
                      
                        Serial.println(eeprom.wydatek_cieczy); 
                        Serial.println(eeprom.rozstawa); 
         
                }

                else if (myData.indexOf("predkosc") != -1) {
                       eeprom.speed_zawracania = (myData.substring(0, myData.indexOf("@"))).toFloat();    
                       eeprom.impulsy = (myData.substring(myData.indexOf("@")+1, myData.indexOf("#"))).toInt();
                       if(eeprom.impulsy<=0){
                        eeprom.impulsy=250;
                       }
                       EEPROM.put(0, eeprom);
                       EEPROM.commit();
                       Serial.println(eeprom.speed_zawracania); 
                       Serial.println(eeprom.impulsy); 
                }

                else if (myData.indexOf("sekcje") != -1) { 

                       int liczba_sekcji = convert(myData.substring(myData.indexOf("$")+1, myData.indexOf("%")));
                       if(liczba_sekcji) {
                         float przeplyw_down_Float = (myData.substring(myData.indexOf("@")+1, myData.indexOf("#"))).toFloat();  
                         float przeplyw_up_Float = (myData.substring(myData.indexOf("#")+1, myData.indexOf("$"))).toFloat();  
                         int liczba_dysz1 = (myData.substring(myData.indexOf("&")+1, myData.indexOf("}"))).toInt();  
                         int liczba_dysz2 = (myData.substring(myData.indexOf("{")+1, myData.indexOf("s"))).toInt();  
                         liczba_dysz1 = check(liczba_dysz1);
                         liczba_dysz2 = check(liczba_dysz2);
                         eeprom.wydatek = pow(przeplyw_up_Float*liczba_dysz1+przeplyw_down_Float*liczba_dysz2,2)*360000; //SERWIS VALUE
                        Serial.println(przeplyw_down_Float); 
                        Serial.println(przeplyw_up_Float);
                       } else { 
                        float przeplyw_Float = (myData.substring(0, myData.indexOf("@"))).toFloat(); 
                        int liczba_dysz = (myData.substring(myData.indexOf("%")+1, myData.indexOf("&"))).toInt();
                        liczba_dysz = check(liczba_dysz);   
                        eeprom.wydatek= pow(przeplyw_Float*liczba_dysz,2)*360000;
                        Serial.println(przeplyw_Float); 
                       }
                       EEPROM.put(0, eeprom);
                       EEPROM.commit();
                       
                           
                } 
                else if (myData.indexOf("mieszanie") != -1) {
                  
                       eeprom.pressure_mix = (myData.substring(0, myData.indexOf("@"))).toFloat(); 
                       EEPROM.put(0, eeprom);
                       EEPROM.commit();
                          
                        Serial.println(eeprom.pressure_mix); 
                }     

                else if (myData.indexOf("advanced1") != -1) {
                  //(myData.substring(myData.indexOf("&")+1, myData.indexOf("}"))).toInt();  
                   m[0.1] = (myData.substring(0, myData.indexOf("a"))).toInt();
                   m[0.2] =(myData.substring(myData.indexOf("a")+1, myData.indexOf("b"))).toInt();
                   m[0.3] =(myData.substring(myData.indexOf("b")+1, myData.indexOf("c"))).toInt();
                   m[0.4] =(myData.substring(myData.indexOf("c")+1, myData.indexOf("d"))).toInt();
                   m[0.5] =(myData.substring(myData.indexOf("d")+1, myData.indexOf("e"))).toInt();
                   m[0.6] =(myData.substring(myData.indexOf("e")+1, myData.indexOf("f"))).toInt();
                   m[0.7] =(myData.substring(myData.indexOf("f")+1, myData.indexOf("g"))).toInt();
                   m[0.8] =(myData.substring(myData.indexOf("g")+1, myData.indexOf("h"))).toInt();
                   m[0.9] =(myData.substring(myData.indexOf("h")+1, myData.indexOf("i"))).toInt();
                   m[1.0] =(myData.substring(myData.indexOf("i")+1, myData.indexOf("j"))).toInt();
                   m[1.1] =(myData.substring(myData.indexOf("j")+1, myData.indexOf("k"))).toInt();
                   m[1.2] =(myData.substring(myData.indexOf("k")+1, myData.indexOf("l"))).toInt();
                   m[1.3] =(myData.substring(myData.indexOf("l")+1, myData.indexOf("m"))).toInt();
                   m[1.4] =(myData.substring(myData.indexOf("m")+1, myData.indexOf("n"))).toInt();
                   m[1.5] =(myData.substring(myData.indexOf("n")+1, myData.indexOf("o"))).toInt();
                   m[1.6] =(myData.substring(myData.indexOf("o")+1, myData.indexOf("p"))).toInt();
                   m[1.7] =(myData.substring(myData.indexOf("p")+1, myData.indexOf("r"))).toInt();
                   m[1.8] =(myData.substring(myData.indexOf("r")+1, myData.indexOf("s"))).toInt();
                   m[1.9] =(myData.substring(myData.indexOf("s")+1, myData.indexOf("t"))).toInt();
                   m[2.0] =(myData.substring(myData.indexOf("t")+1, myData.indexOf("advanced1"))).toInt();
                  
                  
                   Serial.println(m[0.1]);
                    Serial.println(m[0.2]);
                     Serial.println(m[0.3]);
                      Serial.println(m[0.4]);
                       Serial.println(m[0.5]);
                        Serial.println(m[0.6]);
                   
                      
                }         
                else if (myData.indexOf("advanced2") != -1) {
                  
                      k_pressure1 = (myData.substring(0, myData.indexOf("@"))).toFloat(); 
                      k_pressure2 = (myData.substring(myData.indexOf("@")+1, myData.indexOf("#"))).toFloat(); 
                      k_pressure3 = (myData.substring(myData.indexOf("#")+1, myData.indexOf("$"))).toFloat();  
                      k_speed1 = (myData.substring(myData.indexOf("$")+1, myData.indexOf("%"))).toFloat(); 
                      k_speed2 = (myData.substring(myData.indexOf("%")+1, myData.indexOf("&"))).toFloat(); 
                      k_speed3 = (myData.substring(myData.indexOf("&")+1, myData.indexOf("advanced2"))).toFloat(); 
                      kalman_update();
                      Serial.println(k_pressure1);
                      Serial.println(k_pressure2);
                      Serial.println(k_pressure3);
                      Serial.println(k_speed1);
                      Serial.println(k_speed2);
                      Serial.println(k_speed3);
                }     

                myData="";

                
}




boolean convert(String data) {
  if(data=="true") {
    return (true);
  } else {
    return (false);
  }
}

int check(int data) {
  if(data<=0) {
    return 7;
  } else{
    return data;
  }
}

void handleInterrupt() {
  czas2=czas;
  czas=0;
  if(czas2!=0) {
  speed_kmh = 360000/(czas2*eeprom.impulsy);
  if(speed_kmh<1||speed_kmh>70) {
    speed_kmh=0;
  }
  speed_kmh = speedKalmanFilter.updateEstimate(speed_kmh);
  helper=1;
  }

}

void zegar() {
  czas++;
  
}

void readAnalog() {
  adc = analogRead(A0);
  adc = adc-eeprom.adc_calibration;
  pressure =(adc*eeprom.maxpressure)/1023;
  pressure = round(pressure*10)/10;
  if(pressure<0) {
    pressure=0;
  }
  pressure = pressureKalmanFilter.updateEstimate(pressure);
  helper=1;
  
  //Serial.println((String)"up: "+value_up+"  down: " + value_down +"  bar: "+pressure+"  bar_reg: "+cisnienie_reg+"  bar_help:  "+cisnienie_help+"  Wydatek:  "+eeprom.wydatek);
}

void kalman_update() {
  SimpleKalmanFilter pressureKalmanFilter(k_pressure1, k_pressure2, k_pressure3);
  SimpleKalmanFilter speedKalmanFilter(k_speed1, k_speed2, k_speed3);
}

void map_maker() {

   m.insert(std::make_pair(0.0, 1023));
   m.insert(std::make_pair(0.1, 970));
   m.insert(std::make_pair(0.2, 920));
   m.insert(std::make_pair(0.3, 870));
   m.insert(std::make_pair(0.4, 820));
   m.insert(std::make_pair(0.5, 760));
   m.insert(std::make_pair(0.6, 710));
   m.insert(std::make_pair(0.7, 660));
   m.insert(std::make_pair(0.8, 610));
   m.insert(std::make_pair(0.9, 560));
   m.insert(std::make_pair(1.0, 510));
   m.insert(std::make_pair(1.1, 460));
   m.insert(std::make_pair(1.2, 410));
   m.insert(std::make_pair(1.3, 350));
   m.insert(std::make_pair(1.4, 300));
   m.insert(std::make_pair(1.5, 250));
   m.insert(std::make_pair(1.6, 200));
   m.insert(std::make_pair(1.7, 150));
   m.insert(std::make_pair(1.8, 100));
   m.insert(std::make_pair(1.9, 50));
   m.insert(std::make_pair(2.0, 0));

   
   //m.insert(std::make_pair(0.0, 0));
   //m.insert(std::make_pair(0.1, 50));
  //m.insert(std::make_pair(0.2, 100));
  // m.insert(std::make_pair(0.3, 150));
  // m.insert(std::make_pair(0.4, 200));
  // m.insert(std::make_pair(0.5, 250));
  // m.insert(std::make_pair(0.6, 300));
  // m.insert(std::make_pair(0.7, 350));
  // m.insert(std::make_pair(0.8, 410));
 //  m.insert(std::make_pair(0.9, 460));
  // m.insert(std::make_pair(1.0, 510));
  // m.insert(std::make_pair(1.1, 560));
  // m.insert(std::make_pair(1.2, 610));
  // m.insert(std::make_pair(1.3, 660));
  // m.insert(std::make_pair(1.4, 710));
  // m.insert(std::make_pair(1.5, 760));
  // m.insert(std::make_pair(1.6, 820));
  // m.insert(std::make_pair(1.7, 870));
  // m.insert(std::make_pair(1.8, 920));
  // m.insert(std::make_pair(1.9, 970));
  // m.insert(std::make_pair(2.0, 1023));

  
   
}
