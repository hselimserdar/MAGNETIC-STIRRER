#include "hourglass.cpp"
#include "stirring.cpp"
#include "heater.cpp"
#include "c64enh_font.h"
#include "Small4x6_font.h"
#include "ST7920_SPI.h"
#include "RTClib.h"
#include "max6675.h"
#include <analogWrite.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <PID_v1.h>

#define sensor_pin 23
#define IGBT_pin 3
#define PinDT 4
#define PinCLK 5
#define LCD_CS 6
#define PinSW 7
#define thermoDO 29
#define thermoCS 27
#define MOTOR_PIN 11
#define BUZZER_PIN 17
#define thermoCLK 25
#define HeaterRelay 37

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
ST7920_SPI lcd(LCD_CS);
RTC_DS1307 rtc;

double Set_RPM_D, Tachometer, Output;
double Set_Temp_D, TempPID, Output2;
double Kp = 0.04, Ki = 0.05, Kd = 0.05;
double Kp2 = 40, Ki2 = 50, Kd2 = 50;
const byte PulsesPerRevolution = 1;
const byte numReadings = 3;
const unsigned long ZeroTimeout = 700000;
volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
char buf[20];
int gun;
int ay;
int yil;
int saat;
int dakika;
int cur_gun;
int cur_ay;
int cur_yil;
int cur_saat;
int cur_dakika;
int cur_saniye;
int temp_eeprom;
int stir_eeprom;
int stir_dur_hour_eeprom;
int stir_dur_min_eeprom;
int stir_dur_state_eeprom;
int temp_dur_hour_eeprom;
int temp_dur_min_eeprom;
int temp_dur_state_eeprom;
int stir_durrem_hour;
int stir_durrem_min;
int stir_durrem_sec;
int temp_durrem_hour;
int temp_durrem_min;
int temp_durrem_sec;
int DelayofDebounce = 5;
int PreviousCLK;
int PreviousDATA;
int MAIN = 0; //0:MAIN MENU 1:SETTINGS 2:SETTINGS_TAB
int UNDERSETTINGS = 0;
int SETTINGS = 0; //0-6: SETTINGS 7:EXIT
int SELECTED = 0;
int Stir_ON = 0;
int Heat_ON = 0;
int BUZZER;
int ix = 50, iy = 45;
int w, h;
int HEATALARM, STIRALARM;
int Temp;
unsigned int Set_Temp;
unsigned int Set_Temp_G;
unsigned int Set_RPM;
unsigned int Set_RPM_G;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned int PulseCounter = 1;
unsigned int RPM_D;
unsigned long tempdurrem;
unsigned long stirdurrem;
unsigned long RotarySWTimer;
unsigned long TimeOfLastDebounce = 0;
unsigned long timestir;
unsigned long timeheat;
unsigned long arrowanimtime;
unsigned long rotarytimeout;
unsigned long stirelapsed;
unsigned long tempelapsed;
unsigned long FrequencyRaw;
unsigned long FrequencyReal;
unsigned long RPM;
unsigned long readings[numReadings];
unsigned long PeriodSum;
unsigned long readIndex;
unsigned long total;
unsigned long heater_temp_delay;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
unsigned long lastmicros;

PID myPID(&Tachometer, &Output, &Set_RPM_D, Kp, Ki, Kd, DIRECT);
PID myPID2(&TempPID, &Output2, &Set_Temp_D, Kp2, Ki2, Kd2, DIRECT);

void setup() {
  pinMode(PinSW, INPUT);
  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);
  pinMode (IGBT_pin, OUTPUT);
  pinMode (BUZZER_PIN, OUTPUT);
  pinMode (MOTOR_PIN, OUTPUT);
  pinMode (HeaterRelay, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING);
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  SPI.begin();
  lcd.init();
  lcd.cls();
  lcd.setFont(c64enh);
  snprintf(buf, 20, "LOADING");
  hourglass();
  rtc.begin();
  snprintf(buf, 20, "LOADING.");
  hourglass();
  DateTime now = rtc.now();
  cur_yil = now.year();
  cur_ay = now.month();
  cur_gun = now.day();
  cur_saat = now.hour();
  cur_dakika = now.minute();
  rtcloop();
  snprintf(buf, 20, "LOADING..");
  hourglass();
  BUZZER = EEPROM.read(0);
  temp_eeprom = EEPROM.read(1);
  stir_eeprom = EEPROM.read(2);
  stir_dur_hour_eeprom = EEPROM.read(3);
  stir_dur_min_eeprom = EEPROM.read(4);
  stir_dur_state_eeprom = EEPROM.read(5);
  temp_dur_hour_eeprom = EEPROM.read(6);
  temp_dur_min_eeprom = EEPROM.read(7);
  temp_dur_state_eeprom = EEPROM.read(8);
  Serial.begin(57600);
  snprintf(buf, 20, "LOADING...");
  hourglass();
  PreviousCLK = digitalRead(PinCLK);
  PreviousDATA = digitalRead(PinDT);
  lcd.cls();
}

void loop() {
  heatercontrol();
  TachometerCalc();
  DateTime now = rtc.now();
  cur_yil = now.year();
  cur_ay = now.month();
  cur_gun = now.day();
  cur_saat = now.hour();
  cur_dakika = now.minute();
  cur_saniye = now.second();
  lcd.cls();
  rotaryencoder();
  heat_animation();
  stir_animation();
  settings();
  alarm();
  Date();
  Time_L();
  heatingduration();
  stirduration();
  heatingtemp();
  stirringrpm();
  eeprom();
  durrem();
  timer();
  settingscheck();
  lcd.display(0);
  arrowanimtime = millis() / 500;
}

void timer(void) {
  if (Heat_ON == 1) {
    Set_Temp_D = Set_Temp;
    Set_Temp_G = Set_Temp;
  }
  else {
    Set_Temp_D = 0;
    Set_Temp_G = 0;
  }
  if (Stir_ON == 1) {
    Set_RPM_D = Set_RPM;
    Set_RPM_G = Set_RPM;
  }
  else {
    Set_RPM_D = 0;
    Set_RPM_G = 0;
  }
}

void durrem(void) {
  if (temp_dur_state_eeprom == 1) {
    tempelapsed = ((millis() - tempdurrem) / 1000);
    temp_durrem_sec = (((temp_dur_hour_eeprom * 3600) + (temp_dur_min_eeprom * 60)) - tempelapsed) % 60;
    temp_durrem_min = ((((temp_dur_hour_eeprom * 3600) + (temp_dur_min_eeprom * 60)) - tempelapsed) / 60) % 60;
    temp_durrem_hour = (((temp_dur_hour_eeprom * 3600) + (temp_dur_min_eeprom * 60)) - tempelapsed) / 3600;
    if ((Heat_ON == 1) && (tempelapsed >= ((temp_dur_hour_eeprom * 3600) + (temp_dur_min_eeprom * 60)))) {
      Heat_ON = 0;
      temp_durrem_hour = 0;
      temp_durrem_min = 0;
      temp_durrem_sec = 0;
      if (BUZZER == 1) {
        HEATALARM = 1;
      }
    }
    if (Heat_ON == 0) {
      temp_durrem_hour = 0;
      temp_durrem_min = 0;
      temp_durrem_sec = 0;
    }
  }
  if (stir_dur_state_eeprom == 1) {
    stirelapsed = ((millis() - stirdurrem) / 1000);
    stir_durrem_sec = (((stir_dur_hour_eeprom * 3600) + (stir_dur_min_eeprom * 60)) - stirelapsed) % 60;
    stir_durrem_min = ((((stir_dur_hour_eeprom * 3600) + (stir_dur_min_eeprom * 60)) - stirelapsed) / 60) % 60;
    stir_durrem_hour = (((stir_dur_hour_eeprom * 3600) + (stir_dur_min_eeprom * 60)) - stirelapsed) / 3600;
    if ((Stir_ON == 1) && (stirelapsed >= ((stir_dur_hour_eeprom * 3600) + (stir_dur_min_eeprom * 60)))) {
      Stir_ON = 0;
      stir_durrem_hour = 0;
      stir_durrem_min = 0;
      stir_durrem_sec = 0;
      if (BUZZER == 1) {
        STIRALARM = 1;
      }
    }
    if (Stir_ON == 0) {
      stir_durrem_hour = 0;
      stir_durrem_min = 0;
      stir_durrem_sec = 0;
    }
  }
  if ((STIRALARM == 0) && (HEATALARM == 0)) {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void RotarySW(void) {
  if ((STIRALARM == 1) || (HEATALARM == 1)) {
    STIRALARM = 0;
    HEATALARM = 0;
    RotarySWTimer = millis();
  }
  if (millis() - RotarySWTimer > 200) {
    if (SELECTED == 1) {
      SELECTED = 0;
      RotarySWTimer = millis();
    }
  }
  if (millis() - RotarySWTimer > 200) {
    if (MAIN == 2) {
      if (SELECTED == 0) {
        if (SETTINGS == 6) {
          if (UNDERSETTINGS == 0) {
            if (BUZZER == 0) {
              BUZZER = 1;
              EEPROM.write(0, 1);
            }
            else {
              BUZZER = 0;
              EEPROM.write(0, 0);
            }
          }
          if (UNDERSETTINGS == 1) {
            MAIN = 1;
          }
        }
        if (SETTINGS == 5) {
          if (UNDERSETTINGS < 3) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 3) {
            MAIN = 1;
            UNDERSETTINGS = 0;
            rtc.adjust(DateTime(yil, ay, gun, cur_saat, cur_dakika, cur_saniye));
            rtcloop();
          }
          else if (UNDERSETTINGS == 4) {
            MAIN = 1;
            UNDERSETTINGS = 0;
            rtcloop();
          }
        }
        if (SETTINGS == 4) {
          if (UNDERSETTINGS < 2) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 2) {
            MAIN = 1;
            UNDERSETTINGS = 0;
            rtc.adjust(DateTime(cur_yil, cur_ay, cur_gun, saat, dakika, 0));
            rtcloop();
          }
          else if (UNDERSETTINGS == 3) {
            MAIN = 1;
            UNDERSETTINGS = 0;
            rtcloop();
          }
        }
        if (SETTINGS == 3) {
          if (UNDERSETTINGS == 0) {
            if (temp_dur_state_eeprom == 0) {
              temp_dur_state_eeprom = 1;
              EEPROM.write(8, temp_dur_state_eeprom);
            }
            else {
              temp_dur_state_eeprom = 0;
              EEPROM.write(8, temp_dur_state_eeprom);
            }
          }
          else if ((UNDERSETTINGS == 1) || (UNDERSETTINGS == 2)) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 3) {
            MAIN = 1;
          }
        }
        if (SETTINGS == 2) {
          if (UNDERSETTINGS == 0) {
            if (Heat_ON == 0) {
              tempdurrem = millis();
              Heat_ON = 1;
            }
            else {
              Heat_ON = 0;
            }
          }
          else if (UNDERSETTINGS == 1) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 2) {
            MAIN = 1;
          }
        }
        if (SETTINGS == 1) {
          if (UNDERSETTINGS == 0) {
            if (stir_dur_state_eeprom == 0) {
              stir_dur_state_eeprom = 1;
              EEPROM.write(5, stir_dur_state_eeprom);
            }
            else {
              stir_dur_state_eeprom = 0;
              EEPROM.write(5, stir_dur_state_eeprom);
            }
          }
          else if ((UNDERSETTINGS == 1) || (UNDERSETTINGS == 2)) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 3) {
            MAIN = 1;
          }
        }
        if (SETTINGS == 0) {
          if (UNDERSETTINGS == 0) {
            if (Stir_ON == 0) {
              stirdurrem = millis();
              Stir_ON = 1;
            }
            else {
              Stir_ON = 0;
            }
          }
          else if (UNDERSETTINGS == 1) {
            SELECTED = 1;
          }
          else if (UNDERSETTINGS == 2) {
            MAIN = 1;
          }
        }
      }
    }
    else if (MAIN == 1) {
      if (SETTINGS < 7) {
        MAIN = 2;
        UNDERSETTINGS = 0;
        if ((SETTINGS == 5) || (SETTINGS == 4)) {
          rtcloop();
        }
      }
      else if (SETTINGS == 7) {
        MAIN = 0;
      }
    }
    else if (MAIN == 0) {
      MAIN = 1;
      SETTINGS = 0;
    }
  }
  RotarySWTimer = millis();
}

void RotaryCW(void) {
  if (MAIN == 1) {
    SETTINGS ++;
    if (SETTINGS == 8) {
      SETTINGS = 0;
    }
  }
  if ((MAIN == 2) && (SELECTED == 0)) {
    UNDERSETTINGS ++;
    if ((SETTINGS == 6) && (UNDERSETTINGS > 1)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 5) && (UNDERSETTINGS > 4)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 4) && (UNDERSETTINGS > 3)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 3) && (UNDERSETTINGS > 3)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 2) && (UNDERSETTINGS > 2)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 1) && (UNDERSETTINGS > 3)) {
      UNDERSETTINGS = 0;
    }
    if ((SETTINGS == 0) && (UNDERSETTINGS > 2)) {
      UNDERSETTINGS = 0;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 5 && SELECTED == 1)) {
    if (UNDERSETTINGS == 0) {
      yil = yil + 1;
    }
    else if (UNDERSETTINGS == 1) {
      ay = ay + 1;
    }
    else if (UNDERSETTINGS == 2) {
      gun = gun + 1;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 4 && SELECTED == 1)) {
    if (UNDERSETTINGS == 0) {
      saat = saat + 1;
    }
    else if (UNDERSETTINGS == 1) {
      dakika = dakika + 1;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 3 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      temp_dur_hour_eeprom = temp_dur_hour_eeprom + 1;
      EEPROM.write(6, temp_dur_hour_eeprom);
    }
    else if (UNDERSETTINGS == 2) {
      temp_dur_min_eeprom = temp_dur_min_eeprom + 1;
      EEPROM.write(7, temp_dur_min_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 2 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      temp_eeprom = temp_eeprom + 1;
      EEPROM.write(1, temp_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 1 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      stir_dur_hour_eeprom = stir_dur_hour_eeprom + 1;
      EEPROM.write(3, stir_dur_hour_eeprom);
    }
    else if (UNDERSETTINGS == 2) {
      stir_dur_min_eeprom = stir_dur_min_eeprom + 1;
      EEPROM.write(4, stir_dur_min_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 0 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      stir_eeprom = stir_eeprom + 1;
      EEPROM.write(2, stir_eeprom);
    }
  }
}

void RotaryCCW(void) {
  if (MAIN == 1) {
    SETTINGS --;
    if (SETTINGS < 0) {
      SETTINGS = 7;
    }
  }
  if ((MAIN == 2) && (SELECTED == 0)) {
    UNDERSETTINGS --;
    if ((SETTINGS == 6) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 1;
    }
    if ((SETTINGS == 5) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 4;
    }
    if ((SETTINGS == 4) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 3;
    }
    if ((SETTINGS == 3) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 3;
    }
    if ((SETTINGS == 2) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 2;
    }
    if ((SETTINGS == 1) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 3;
    }
    if ((SETTINGS == 0) && (UNDERSETTINGS < 0)) {
      UNDERSETTINGS = 2;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 5 && SELECTED == 1)) {
    if (UNDERSETTINGS == 0) {
      yil = yil - 1;
    }
    else if (UNDERSETTINGS == 1) {
      ay = ay - 1;
    }
    else if (UNDERSETTINGS == 2) {
      gun = gun - 1;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 4 && SELECTED == 1)) {
    if (UNDERSETTINGS == 0) {
      saat = saat - 1;
    }
    else if (UNDERSETTINGS == 1) {
      dakika = dakika - 1;
    }
  }
  if ((MAIN == 2) && (SETTINGS == 3 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      temp_dur_hour_eeprom = temp_dur_hour_eeprom - 1;
      EEPROM.write(6, temp_dur_hour_eeprom);
    }
    else if (UNDERSETTINGS == 2) {
      temp_dur_min_eeprom = temp_dur_min_eeprom - 1;
      EEPROM.write(7, temp_dur_min_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 2 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      temp_eeprom = temp_eeprom - 1;
      EEPROM.write(1, temp_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 1 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      stir_dur_hour_eeprom = stir_dur_hour_eeprom - 1;
      EEPROM.write(3, stir_dur_hour_eeprom);
    }
    else if (UNDERSETTINGS == 2) {
      stir_dur_min_eeprom = stir_dur_min_eeprom - 1;
      EEPROM.write(4, stir_dur_min_eeprom);
    }
  }
  if ((MAIN == 2) && (SETTINGS == 0 && SELECTED == 1)) {
    if (UNDERSETTINGS == 1) {
      stir_eeprom = stir_eeprom - 1;
      EEPROM.write(2, stir_eeprom);
    }
  }
}

void checkrotary(void) {
  if (millis() - rotarytimeout > 50) {
    if ((PreviousCLK == 0) && (PreviousDATA == 1)) {
      if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
        RotaryCW();
        rotarytimeout = millis();
      }
      if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
        RotaryCCW();
        rotarytimeout = millis();
      }
    }

    if ((PreviousCLK == 1) && (PreviousDATA == 0)) {
      if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
        RotaryCW();
        rotarytimeout = millis();
      }
      if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
        RotaryCCW();
        rotarytimeout = millis();
      }
    }

    if ((PreviousCLK == 1) && (PreviousDATA == 1)) {
      if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
        RotaryCW();
        rotarytimeout = millis();
      }
      if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
        RotaryCCW();
        rotarytimeout = millis();
      }
    }

    if ((PreviousCLK == 0) && (PreviousDATA == 0)) {
      if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
        RotaryCW();
        rotarytimeout = millis();
      }
      if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
        RotaryCCW();
        rotarytimeout = millis();
      }
    }
  }
}

void heatercontrol(void) {
  if (millis() - heater_temp_delay > 1000) {
    Temp = thermocouple.readCelsius();
    heater_temp_delay = millis();
    TempPID = Temp;
  }
  if (Heat_ON == 1) {
    myPID2.Compute();
    analogWrite(IGBT_pin, Output2);
    digitalWrite(HeaterRelay, LOW);
  }
  else {
    analogWrite(IGBT_pin, 0);
    digitalWrite(HeaterRelay, HIGH);
  }
}

void TachometerCalc(void) {
  LastTimeCycleMeasure = LastTimeWeMeasured;
  CurrentMicros = micros();

  if (CurrentMicros < LastTimeCycleMeasure) {
    LastTimeCycleMeasure = CurrentMicros;
  }

  FrequencyRaw = 10000000000 / PeriodAverage;

  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra) {
    FrequencyRaw = 0;
    ZeroDebouncingExtra = 2000;
  }
  else {
    ZeroDebouncingExtra = 0;
  }

  FrequencyReal = FrequencyRaw / 10000;
  RPM = FrequencyRaw / PulsesPerRevolution * 60;
  RPM = RPM / 10000;
  total = total - readings[readIndex];
  readings[readIndex] = RPM;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }

  Tachometer = total / numReadings;

  Serial.print("Period: ");
  Serial.print(PeriodBetweenPulses);
  Serial.print("\tReadings: ");
  Serial.print(AmountOfReadings);
  Serial.print("\tFrequency: ");
  Serial.print(FrequencyReal);
  Serial.print("\tRPM: ");
  Serial.print(RPM);
  Serial.print("\tTachometer: ");
  Serial.println(Tachometer);

  if (Stir_ON == 1) {
    myPID.Compute();
    analogWrite(MOTOR_PIN, Output);
  }
  else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void Pulse_Event() {
  PeriodBetweenPulses = micros() - LastTimeWeMeasured;
  LastTimeWeMeasured = micros();

  if (PulseCounter >= AmountOfReadings) {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;
    PeriodSum = PeriodBetweenPulses;

    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
    AmountOfReadings = RemapedAmountOfReadings;
  }
  else {
    PulseCounter++;
    PeriodSum = PeriodSum + PeriodBetweenPulses;
  }
}

void rotaryencoder(void) {
  if ((millis() - TimeOfLastDebounce) > DelayofDebounce) {
    checkrotary();
    PreviousCLK = digitalRead(PinCLK);
    PreviousDATA = digitalRead(PinDT);
    TimeOfLastDebounce = millis();
  }

  if (digitalRead(PinSW) == LOW) {
    RotarySW();
  }
}

void eeprom(void) {
  Set_RPM = (300 + ((stir_eeprom - 1) * 50));
  Set_Temp = (50 + ((temp_eeprom - 1) * 10));
}

void settingscheck(void) {
  if (saat > 23) {
    saat = 0;
  }
  if (saat < 0) {
    saat = 23;
  }
  if (dakika > 59) {
    dakika = 0;
  }
  if (dakika < 0) {
    dakika = 59;
  }
  if (yil > 2030) {
    yil = 2020;
  }
  if (yil < 2010) {
    yil = 2020;
  }
  else if (yil < 2020) {
    yil = 2030;
  }
  if (ay > 12) {
    ay = 1;
  }
  if (ay < 1) {
    ay = 12;
  }
  if (gun > 31) {
    gun = 1;
  }
  if (gun < 1) {
    gun = 31;
  }
  if (temp_dur_min_eeprom > 59) {
    temp_dur_min_eeprom = 0;
    EEPROM.write(7, temp_dur_min_eeprom);
  }
  if (temp_dur_min_eeprom < 0) {
    temp_dur_min_eeprom = 59;
    EEPROM.write(7, temp_dur_min_eeprom);
  }
  if (stir_dur_min_eeprom > 59) {
    stir_dur_min_eeprom = 0;
    EEPROM.write(4, stir_dur_min_eeprom);
  }
  if (stir_dur_min_eeprom < 0) {
    stir_dur_min_eeprom = 59;
    EEPROM.write(4, stir_dur_min_eeprom);
  }
  if (temp_dur_hour_eeprom > 48) {
    temp_dur_hour_eeprom = 0;
    EEPROM.write(6, temp_dur_hour_eeprom);
  }
  if (temp_dur_hour_eeprom < 0) {
    temp_dur_hour_eeprom = 48;
    EEPROM.write(6, temp_dur_hour_eeprom);
  }
  if (stir_dur_hour_eeprom > 48) {
    stir_dur_hour_eeprom = 0;
    EEPROM.write(3, stir_dur_hour_eeprom);
  }
  if (stir_dur_hour_eeprom < 0) {
    stir_dur_hour_eeprom = 48;
    EEPROM.write(3, stir_dur_hour_eeprom);
  }
  if (BUZZER > 1) {
    BUZZER = 0;
    EEPROM.write(0, BUZZER);
  }
  if (temp_dur_state_eeprom > 1) {
    temp_dur_state_eeprom = 0;
    EEPROM.write(8, temp_dur_state_eeprom);
  }
  if (stir_dur_state_eeprom > 1) {
    stir_dur_state_eeprom = 0;
    EEPROM.write(5, stir_dur_state_eeprom);
  }
  if (temp_eeprom < 1) {
    temp_eeprom = 36;
    EEPROM.write(1, temp_eeprom);
  }
  if (temp_eeprom > 36) {
    temp_eeprom = 1;
    EEPROM.write(1, temp_eeprom);
  }
  if (stir_eeprom < 1) {
    stir_eeprom = 35;
    EEPROM.write(2, stir_eeprom);
  }
  if (stir_eeprom > 35) {
    stir_eeprom = 1;
    EEPROM.write(2, stir_eeprom);
  }
}

void stirringrpm(void) {
  if ((MAIN == 2) && (SETTINGS == 0)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "STIRRING RPM");
    lcd.drawLine(0, 7, 127, 7, 1);
    if (Stir_ON == 0) {
      lcd.printStr(5, 9, "TURN ON");
      lcd.printStr(70, 9, "CURRENT: OFF");
    }
    if (Stir_ON == 1) {
      lcd.printStr(5, 9, "TURN OFF");
      lcd.printStr(70, 9, "CURRENT: ON");
    }
    snprintf(buf, 20, "RPM: %02d", Set_RPM);
    lcd.printStr(70, 16, buf);
    lcd.printStr(5, 16, "SET RPM");
    lcd.printStr(5, 23, "EXIT");
    if (UNDERSETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      w = 0;
      h = 23;
      menu_arrow();
    }
  }
}

void heatingtemp(void) {
  if ((MAIN == 2) && (SETTINGS == 2)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "HEATING TEMP");
    lcd.drawLine(0, 7, 127, 7, 1);
    if (Heat_ON == 0) {
      lcd.printStr(5, 9, "TURN ON");
      lcd.printStr(70, 9, "CURRENT: OFF");
    }
    if (Heat_ON == 1) {
      lcd.printStr(5, 9, "TURN OFF");
      lcd.printStr(70, 9, "CURRENT: ON");
    }
    snprintf(buf, 20, "TEMP: %02d", Set_Temp);
    lcd.printStr(70, 16, buf);
    lcd.printStr(5, 16, "SET TEMP");
    lcd.printStr(5, 23, "EXIT");
    if (UNDERSETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      w = 0;
      h = 23;
      menu_arrow();
    }
  }
}

void stirduration(void) {
  if ((MAIN == 2) && (SETTINGS == 1)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "STIRRING DURATION");
    lcd.drawLine(0, 7, 127, 7, 1);
    if (stir_dur_state_eeprom == 0) {
      lcd.printStr(5, 9, "TURN ON");
      lcd.printStr(70, 9, "CURRENT: OFF");
    }
    if (stir_dur_state_eeprom == 1) {
      lcd.printStr(5, 9, "TURN OFF");
      lcd.printStr(70, 9, "CURRENT: ON");
    }
    snprintf(buf, 20, "%02dHr %02dMin", stir_dur_hour_eeprom, stir_dur_min_eeprom);
    lcd.printStr(70, 16, buf);
    lcd.printStr(5, 16, "SET HOUR");
    lcd.printStr(5, 23, "SET MINUTE");
    lcd.printStr(5, 30, "EXIT");
    if (UNDERSETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      if (SELECTED == 0) {
        w = 0;
        h = 23;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 23;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 3) {
      w = 0;
      h = 30;
      menu_arrow();
    }
  }
}

void heatingduration(void) {
  if ((MAIN == 2) && (SETTINGS == 3)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "HEATING DURATION");
    lcd.drawLine(0, 7, 127, 7, 1);
    if (temp_dur_state_eeprom == 0) {
      lcd.printStr(5, 9, "TURN ON");
      lcd.printStr(70, 9, "CURRENT: OFF");
    }
    if (temp_dur_state_eeprom == 1) {
      lcd.printStr(5, 9, "TURN OFF");
      lcd.printStr(70, 9, "CURRENT: ON");
    }
    snprintf(buf, 20, "%02dHr %02dMin", temp_dur_hour_eeprom, temp_dur_min_eeprom);
    lcd.printStr(70, 16, buf);
    lcd.printStr(5, 16, "SET HOUR");
    lcd.printStr(5, 23, "SET MINUTE");
    lcd.printStr(5, 30, "EXIT");
    if (UNDERSETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      if (SELECTED == 0) {
        w = 0;
        h = 23;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 23;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 3) {
      w = 0;
      h = 30;
      menu_arrow();
    }
  }
}

void Time_L(void) {
  if ((MAIN == 2) && (SETTINGS == 4)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "TIME");
    lcd.drawLine(0, 7, 127, 7, 1);
    lcd.printStr(5, 9, "SET HOUR");
    snprintf(buf, 20, "%02d:%02d", saat, dakika);
    lcd.printStr(80, 9, buf);
    lcd.printStr(5, 16, "SET MINUTE");
    lcd.printStr(5, 23, "SAVE AND EXIT");
    lcd.printStr(5, 30, "EXIT");
    if (UNDERSETTINGS == 0) {
      if (SELECTED == 0) {
        w = 0;
        h = 9;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 9;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      w = 0;
      h = 23;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 3) {
      w = 0;
      h = 30;
      menu_arrow();
    }
  }
}

void Date(void) {
  if ((MAIN == 2) && (SETTINGS == 5)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "DATE");
    lcd.drawLine(0, 7, 127, 7, 1);
    lcd.printStr(5, 9, "SET YEAR");
    snprintf(buf, 20, "%02d.%02d.%02d", gun, ay, yil);
    lcd.printStr(70, 9, buf);
    lcd.printStr(5, 16, "SET MONTH");
    lcd.printStr(5, 23, "SET DAY");
    lcd.printStr(5, 30, "SAVE AND EXIT");
    lcd.printStr(5, 37, "EXIT");
    if (UNDERSETTINGS == 0) {
      if (SELECTED == 0) {
        w = 0;
        h = 9;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 9;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 1) {
      if (SELECTED == 0) {
        w = 0;
        h = 16;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 16;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 2) {
      if (SELECTED == 0) {
        w = 0;
        h = 23;
        menu_arrow();
      }
      else if ((SELECTED == 1) && (arrowanimtime % 2 == 0)) {
        w = 0;
        h = 23;
        menu_arrow();
      }
    }
    else if (UNDERSETTINGS == 3) {
      w = 0;
      h = 30;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 4) {
      w = 0;
      h = 37;
      menu_arrow();
    }
  }
}

void alarm(void) {
  if ((MAIN == 2) && (SETTINGS == 6)) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "ALARM");
    lcd.drawLine(0, 7, 127, 7, 1);
    if (BUZZER == 0) {
      lcd.printStr(5, 9, "TURN ON");
      lcd.printStr(70, 9, "CURRENT: OFF");
    }
    if (BUZZER == 1) {
      lcd.printStr(5, 9, "TURN OFF");
      lcd.printStr(70, 9, "CURRENT: ON");
    }
    lcd.printStr(5, 16, "EXIT");
    if (UNDERSETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (UNDERSETTINGS == 1) {
      w = 0;
      h = 16;
      menu_arrow();
    }
  }
}

void settings(void) {
  if (MAIN == 1) {
    lcd.setFont(Small4x6PL);
    lcd.printStr(0, 1, "SETTINGS MENU");
    lcd.drawLine(0, 7, 127, 7, 1);
    lcd.printStr(5, 9, "STIRRING RPM");
    lcd.printStr(5, 16, "STIRRING DURATION");
    lcd.printStr(5, 23, "HEATING TEMP");
    lcd.printStr(5, 30, "HEATING DURATION");
    lcd.printStr(5, 37, "TIME");
    lcd.printStr(5, 44, "DATE");
    lcd.printStr(5, 51, "ALARM");
    lcd.printStr(5, 58, "MAIN MENU");
    if (SETTINGS == 0) {
      w = 0;
      h = 9;
      menu_arrow();
    }
    else if (SETTINGS == 1) {
      w = 0;
      h = 16;
      menu_arrow();
    }
    else if (SETTINGS == 2) {
      w = 0;
      h = 23;
      menu_arrow();
    }
    else if (SETTINGS == 3) {
      w = 0;
      h = 30;
      menu_arrow();
    }
    else if (SETTINGS == 4) {
      w = 0;
      h = 37;
      menu_arrow();
    }
    else if (SETTINGS == 5) {
      w = 0;
      h = 44;
      menu_arrow();
    }
    else if (SETTINGS == 6) {
      w = 0;
      h = 51;
      menu_arrow();
    }
    else if (SETTINGS == 7) {
      w = 0;
      h = 58;
      menu_arrow();
    }
  }
}

void rtcloop(void) {
  yil = cur_yil;
  ay = cur_ay;
  gun = cur_gun;
  saat = cur_saat;
  dakika = cur_dakika;
}

void menu_arrow(void) {
  lcd.drawPixel(w + 1, h, 1);
  lcd.drawPixel(w + 2, h + 1, 1);
  lcd.drawPixel(w + 3, h + 2, 1);
  lcd.drawPixel(w + 2, h + 3, 1);
  lcd.drawPixel(w + 1, h + 4, 1);
}

void heat_animation(void) {
  timeheat = millis() / 200;
  if (MAIN == 0) {
    if (Heat_ON == 1) {
      if (timeheat % 4 == 0) {
        lcd.drawBitmap(heat1, 0, 30);
      }
      else if (timeheat % 4 == 1) {
        lcd.drawBitmap(heat2, 0, 30);
      }
      else if (timeheat % 4 == 2) {
        lcd.drawBitmap(heat3, 0, 30);
      }
      else {
        lcd.drawBitmap(heat4, 0, 30);
      }
    }
    else {
      lcd.drawBitmap(heat1, 0, 30);
    }
    lcd.drawLine(4, 54, 20, 54, 1);
    lcd.setFont(c64enh);
    if (Temp >  0) {
      snprintf(buf, 20, "%02d 'C", Temp);
      lcd.printStr(28, 42, buf);
    }
    else {
      lcd.printStr(28, 42, "0 'C");
    }
    if (temp_dur_state_eeprom == 1) {
      lcd.setFont(Small4x6PL);
      snprintf(buf, 50, "SET:%02d", Set_Temp_G);
      lcd.printStr(90, 34, buf);
      lcd.setFont(c64enh);
      if (HEATALARM == 0) {
        if (temp_durrem_hour != 0) {
          snprintf(buf, 20, "%02d:%02d", temp_durrem_hour, temp_durrem_min);
        }
        else {
          snprintf(buf, 20, "%02d:%02d", temp_durrem_min, temp_durrem_sec);
        }
        lcd.printStr(90, 42, buf);
      }
    }
    if ((HEATALARM == 1)) {
      if ((arrowanimtime % 2) == 0) {
        if (temp_durrem_hour != 0) {
          snprintf(buf, 20, "%02d:%02d", temp_durrem_hour, temp_durrem_min);
        }
        else {
          snprintf(buf, 20, "%02d:%02d", temp_durrem_min, temp_durrem_sec);
        }
        lcd.printStr(90, 42, buf);
        digitalWrite(BUZZER_PIN, HIGH);
      }
      else {
        digitalWrite(BUZZER_PIN, LOW);
      }
    }
    if (temp_dur_state_eeprom == 0) {
      lcd.setFont(Small4x6PL);
      snprintf(buf, 50, "SET:%02d", Set_Temp_G);
      lcd.printStr(90, 42, buf);
    }
  }
}

void stir_animation(void) {
  timestir = millis() / 100;
  if (MAIN == 0) {
    if (Stir_ON == 1) {
      if (timestir % 4 == 0) {
        lcd.drawBitmap(stir1, 0, 0);
      }
      else if (timestir % 4 == 1) {
        lcd.drawBitmap(stir2, 0, 0);
      }
      else if (timestir % 4 == 2) {
        lcd.drawBitmap(stir3, 0, 0);
      }
      else {
        lcd.drawBitmap(stir4, 0, 0);
      }
    }
    else {
      lcd.drawBitmap(stir1, 0, 0);
    }
    lcd.drawLine(8, 24, 16, 24, 1);
    lcd.setFont(c64enh);
    if (Tachometer > 0) {
      snprintf(buf, 50, "%02d RPM", RPM);
      lcd.printStr(28, 9, buf);
    }
    else {
      lcd.printStr(28, 9, "0 RPM");
    }
    if (stir_dur_state_eeprom == 1) {
      lcd.setFont(Small4x6PL);
      snprintf(buf, 50, "SET:%02d", Set_RPM_G);
      lcd.printStr(90, 1, buf);
      lcd.setFont(c64enh);
      if (STIRALARM == 0) {
        if (stir_durrem_hour != 0) {
          snprintf(buf, 20, "%02d:%02d", stir_durrem_hour, stir_durrem_min);
        }
        else {
          snprintf(buf, 20, "%02d:%02d", stir_durrem_min, stir_durrem_sec);
        }
        lcd.printStr(90, 9, buf);
      }
    }
    if (STIRALARM == 1) {
      if ((arrowanimtime % 2) == 0) {
        if (stir_durrem_hour != 0) {
          snprintf(buf, 20, "%02d:%02d", stir_durrem_hour, stir_durrem_min);
        }
        else {
          snprintf(buf, 20, "%02d:%02d", stir_durrem_min, stir_durrem_sec);
        }
        lcd.printStr(90, 9, buf);
        digitalWrite(BUZZER_PIN, HIGH);
      }
      else {
        digitalWrite(BUZZER_PIN, LOW);
      }
    }
    if (stir_dur_state_eeprom == 0) {
      lcd.setFont(Small4x6PL);
      snprintf(buf, 50, "SET:%02d", Set_RPM_G);
      lcd.printStr(90, 10, buf);
    }
  }
}

void hourglass(void) {
  lcd.drawBitmap(hourglass1, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass2, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass3, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass4, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass5, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass6, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass7, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass8, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass9, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass10, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass11, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass12, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass13, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass14, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass15, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass16, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass17, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass18, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass19, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass20, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass21, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass22, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass23, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass24, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass25, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass26, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass27, 0, 0);
  lcd.printStr(ix, iy, buf);
  lcd.display(0);
  delay(35);
  lcd.cls();
  lcd.drawBitmap(hourglass28, 0, 0);
  lcd.display(0);
  delay(35);
}
