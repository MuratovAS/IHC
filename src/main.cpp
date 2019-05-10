#include <Arduino.h>
#include <U8g2lib.h>
#include "max6675.h"
#include <PID_my.h>
#include "GyverEncoder.h"
#include <EEPROM.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

typedef struct ProfileStruct {
    int temper_1;
    int temper_2;
    int temper_3;
    int temper_4;
    unsigned int timer_1; //в секундах
    unsigned int timer_2;
    unsigned int timer_3;
    unsigned int timer_4;
} ProfileS;

struct EEpromStruct {
    unsigned int Pulse;
    unsigned int P;
    double I;
    unsigned int D;
    double thermocorrection;
    byte T_Ambient;
    byte ErrorRate;
    byte Mode;// 3-manual 2,1,0-prof
    int T_manual;
    unsigned int Time_entry_manual;
    unsigned int Time_hold_manual;
    ProfileS TProfile[3];
};

/////////////////////////////////////////////////////////////////////////////////display
//128x64
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);

/////////////////////////////////////////////////////////////////////////////////MAX6675
const byte T_CS = 10;   // CS - chip selection
const byte T_DO = 12;   // SO - data
const byte T_CLK = 13;  // SCK - clock
MAX6675 temperature_bottom(T_CLK, T_CS, T_DO);

///////////////////////////////////////////////////////////////////////////////// i/o
const byte Pin_HOT = 9;       //relay

const byte Pin_ENC_CLK = 3;   //left
const byte Pin_ENC_DT = 4;    //right    
const byte Pin_ENC_SW = 5;    //but
const bool ENC_TYPE = 1;      //0 one-step, 1 two-step

Encoder enc1(Pin_ENC_CLK, Pin_ENC_DT, Pin_ENC_SW,ENC_TYPE);

/////////////////////////////////////////////////////////////////////////////////SYS
struct EEpromStruct EEprom; //data storage structure

double InputBottom, OutBottom;
unsigned long windowONTime;

unsigned long Time;//current time
unsigned long TimeCOM;//for timing COM
unsigned long TimeMAX;//for timing MAX
unsigned long TimePID;//for timing PID
unsigned long TimeSSD;//for timing display
unsigned long TimeProfile;//for timing Profile

byte ErrorRate_buf = 0;
byte ErrorRate_count = 0; //counting iteration check

bool on_off = false;
byte ProfilStatus = 0; //profile stage
unsigned long TimeProfileStart = 0;//launch time

double T_Bottom; //temperature from the sensor
double T_Set; //specified temperature

PID BottomPID(&InputBottom, &OutBottom, &T_Set, 3, 5, 1, DIRECT);

/**
 * @brief data reading function
 * 
 */
void getEEPROM ()
{
  if (EEPROM.read(0) != 110) 
  {
    EEprom.Mode = 0;  // 3-manual 2,1,0-profile
    
    EEprom.T_manual = 225;
    EEprom.T_Ambient = 25;
    EEprom.Time_entry_manual = 60;
    EEprom.Time_hold_manual = 20; //0 - indefinitely
    
    EEprom.thermocorrection = 0;
    EEprom.ErrorRate = 80; 
    EEprom.P = 50;
    EEprom.I = 0.1;
    EEprom.D = 20;
    EEprom.Pulse = 500;

    /*//first start profiles
    for(byte i = 0; i < 3; i++)
    {
      EEprom.TProfile[0].temper_1 = 145;
      EEprom.TProfile[0].temper_2 = 200;
      EEprom.TProfile[0].temper_3 = 250;
      EEprom.TProfile[0].temper_4 = 100;
      EEprom.TProfile[0].timer_1 = 120;
      EEprom.TProfile[0].timer_2 = 90;
      EEprom.TProfile[0].timer_3 = 60;
      EEprom.TProfile[0].timer_4 = 60;
    }*/

    EEPROM.put(1, EEprom);
    EEPROM.update(0, 110);  //noted data availability
  }
  EEPROM.get(1, EEprom);
  T_Set = EEprom.T_Ambient;
}

/**
 * @brief data recording function
 * 
 */
void saveEEPROM () 
{
  EEPROM.put(1, EEprom);
}

/**
 * @brief the function starts the heating process
 * 
 * @param MODE - heating mode: 3-manual 2,1,0-profile
 */
void RunHot(byte MODE)
{
  //
  u8g2.firstPage();
  do {
    u8g2.setFontMode(1);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setDrawColor(1);
    u8g2.drawStr(48, 35, "RUN");
  } while ( u8g2.nextPage() );
  Serial.print("RUN");
  Serial.print("\n");
  delay(1000);

  EEprom.Mode = MODE;

  OutBottom = 0;
  ErrorRate_count = 0;
  ErrorRate_buf = 0;
  ProfilStatus = 0;
  TimeProfileStart = millis();

  BottomPID.SetTunings(EEprom.P,EEprom.I,EEprom.D);
  BottomPID.SetMode(AUTOMATIC);
  //windowONTime = millis();
  
  on_off = true;
}

/**
 * @brief the function forcibly stops the heating process
 * 
 */
void StopHot()
{
  u8g2.firstPage();
  do {
    u8g2.setFontMode(1);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setDrawColor(1);
    u8g2.drawStr(48, 35, "STOP");
  } while ( u8g2.nextPage() );
  Serial.print("STOP");
  Serial.print("\n");

  digitalWrite(Pin_HOT, 0);
  delay(1000);

  OutBottom = 0;
  ErrorRate_count = 0;
  ErrorRate_buf = 0;
  ProfilStatus = 0;
  TimeProfileStart = 0;
  T_Set = EEprom.T_Ambient;

  BottomPID.SetMode(MANUAL);
  
  on_off = false;
}

void menu1()
{
  byte menu_pos = 0;
  bool menu_edit = true;
  void* structure_field[8] = { &EEprom.TProfile[EEprom.Mode].temper_1, 
                                        &EEprom.TProfile[EEprom.Mode].temper_2, 
                                        &EEprom.TProfile[EEprom.Mode].temper_3,
                                        &EEprom.TProfile[EEprom.Mode].temper_4, 
                                        &EEprom.TProfile[EEprom.Mode].timer_1,
                                        &EEprom.TProfile[EEprom.Mode].timer_2, 
                                        &EEprom.TProfile[EEprom.Mode].timer_3,
                                        &EEprom.TProfile[EEprom.Mode].timer_4};
  TimeSSD = millis();

  while(true)
  {
    enc1.tick();
    Time = millis();

    if (enc1.isHolded())
    {
      saveEEPROM();
      return;
    }

    if (enc1.isPress())
      menu_edit = !menu_edit;

    if (EEprom.Mode < 3)//profile
    {
      if (enc1.isTurn()) 
      {
        if (menu_edit == false)
        {
          if (enc1.isRight()) 
            menu_pos < 7 ? menu_pos++: menu_pos = 7;
          if (enc1.isLeft())
            menu_pos > 0 ? menu_pos--: menu_pos = 0;
        }
        else
        {
          if (enc1.isRight())
          {
            if (menu_pos < 4)
              *((int*)structure_field[menu_pos]) < 400 ? *((int*)structure_field[menu_pos]) += 1: *((int*)structure_field[menu_pos]) = 400; 
            else
              *((unsigned int*)structure_field[menu_pos]) < 999 ? *((unsigned int*)structure_field[menu_pos]) += 1: *((unsigned int*)structure_field[menu_pos]) = 999;
          }

          if (enc1.isLeft())
          {
            if (menu_pos < 4)
              *((int*)structure_field[menu_pos]) > EEprom.T_Ambient ? *((int*)structure_field[menu_pos]) -= 1: *((int*)structure_field[menu_pos]) = EEprom.T_Ambient; 
            else
              *((unsigned int*)structure_field[menu_pos]) > 5 ? *((unsigned int*)structure_field[menu_pos]) -= 1: *((unsigned int*)structure_field[menu_pos]) = 5;
          }

          if (enc1.isFastR())
          {
            if (menu_pos < 4)
              *((int*)structure_field[menu_pos]) < 400-3 ? *((int*)structure_field[menu_pos]) += 3: *((int*)structure_field[menu_pos]) = 400; 
            else
              *((unsigned int*)structure_field[menu_pos]) < 999-3 ? *((unsigned int*)structure_field[menu_pos]) += 3: *((unsigned int*)structure_field[menu_pos]) = 999;
          }

          if (enc1.isFastL())
          {
            if (menu_pos < 4)
              *((int*)structure_field[menu_pos]) > EEprom.T_Ambient+3 ? *((int*)structure_field[menu_pos]) -= 3: *((int*)structure_field[menu_pos]) = EEprom.T_Ambient; 
            else
              *((unsigned int*)structure_field[menu_pos]) > 5+3 ? *((unsigned int*)structure_field[menu_pos]) -= 3: *((unsigned int*)structure_field[menu_pos]) = 5;
          } 
        }
      }
    }
    else//manual
    {
       if (enc1.isTurn()) 
      {
        if (menu_edit == false)
        {
          if (enc1.isRight()) 
            menu_pos < 2 ? menu_pos++: menu_pos = 2;
          if (enc1.isLeft())
            menu_pos > 0 ? menu_pos--: menu_pos = 0;
        }
        else
        {
          void* structure_field[3] = {&EEprom.T_manual, &EEprom.Time_entry_manual, &EEprom.Time_hold_manual};
          
          if (enc1.isRight())
          {
            if (menu_pos == 0)
              *((int*)structure_field[menu_pos]) < 400 ? *((int*)structure_field[menu_pos]) += 1: *((int*)structure_field[menu_pos]) = 400; 
            else
              *((unsigned int*)structure_field[menu_pos]) < 999 ? *((unsigned int*)structure_field[menu_pos]) += 1: *((unsigned int*)structure_field[menu_pos]) = 999;
          }

          if (enc1.isLeft())
          {
            if (menu_pos == 0)
              *((int*)structure_field[menu_pos]) > EEprom.T_Ambient ? *((int*)structure_field[menu_pos]) -= 1: *((int*)structure_field[menu_pos]) = EEprom.T_Ambient; 
            else
              if (menu_pos == 1)
                *((unsigned int*)structure_field[menu_pos]) > 5 ? *((unsigned int*)structure_field[menu_pos]) -= 1: *((unsigned int*)structure_field[menu_pos]) = 5;
              else
                *((unsigned int*)structure_field[menu_pos]) > 0 ? *((unsigned int*)structure_field[menu_pos]) -= 1: *((unsigned int*)structure_field[menu_pos]) = 0;
          }

          if (enc1.isFastR())
          {
            if (menu_pos == 0)
              *((int*)structure_field[menu_pos]) < 400-3 ? *((int*)structure_field[menu_pos]) += 3: *((int*)structure_field[menu_pos]) = 400; 
            else
              *((unsigned int*)structure_field[menu_pos]) < 999-3 ? *((unsigned int*)structure_field[menu_pos]) += 3: *((unsigned int*)structure_field[menu_pos]) = 999;
          }

          if (enc1.isFastL())
          {
            if (menu_pos == 0)
              *((int*)structure_field[menu_pos]) > EEprom.T_Ambient+3 ? *((int*)structure_field[menu_pos]) -= 3: *((int*)structure_field[menu_pos]) = EEprom.T_Ambient; 
            else
              if (menu_pos == 1)
                *((unsigned int*)structure_field[menu_pos]) > 5+3 ? *((unsigned int*)structure_field[menu_pos]) -= 3: *((unsigned int*)structure_field[menu_pos]) = 5;
              else
                *((unsigned int*)structure_field[menu_pos]) > 0+3 ? *((unsigned int*)structure_field[menu_pos]) -= 3: *((unsigned int*)structure_field[menu_pos]) = 0;
         }
        }
      }
    }
    
    if(Time > TimeSSD + 500)
    {
      if (EEprom.Mode < 3) 
      {
        String str;
        char tmpMode[3] = {};
        char tmpNum[8][4] = {};
        
        //data preparation, conversion to Str
        str = "M" + String(EEprom.Mode+1);
        str.toCharArray(tmpMode,3);
        
        for(byte i = 0; i < 8; i++)
        {
          if(i < 4)
            str = String(*((int*)structure_field[i])) + "C";
          else
            str = String(*((unsigned int*)structure_field[i])) + "s";

          str.toCharArray(tmpNum[i],4);
        }

        //output
        u8g2.firstPage();
        do {
          u8g2.setFontMode(1);
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.setDrawColor(1);
          u8g2.drawStr(2, 10, "Configuration");
          u8g2.drawStr(100, 10, tmpMode);
          
          u8g2.drawStr(4, 25, "T1");
          u8g2.drawStr(4, 37, "T2");
          u8g2.drawStr(4, 49, "T3");
          u8g2.drawStr(4, 61, "T4");

          
          u8g2.drawStr(66, 25, "t1");
          u8g2.drawStr(66, 37, "t2");
          u8g2.drawStr(66, 49, "t3");
          u8g2.drawStr(66, 61, "t4");

          for(byte i = 0; i < 8; i++)
          {
            if(i<4)
            {
              u8g2.drawStr(4+25, 25 + 12*i, tmpNum[i]);
              //delay(10);
            }
            else
            {
              u8g2.drawStr(66+25, 25 + 12*(i-4), tmpNum[i]);
              //delay(10);
            }
            
          }
          u8g2.setDrawColor(2); 
          u8g2.drawBox((menu_pos<4 ? 2 : 64) + (menu_edit == true ? 25:0), 16 + (menu_pos<4 ? menu_pos: menu_pos-4)*12 , 18+ (menu_edit == true ? 15:0), 11);
        } while(u8g2.nextPage());
      }
      else
      {
        char tmpNum[3][5] = {};
        String str;
       
        //data preparation, conversion to Str
        str = String(round(EEprom.T_manual)) + "C";
        str.toCharArray(tmpNum[0],5);
        str = String(EEprom.Time_entry_manual) + "s";
        str.toCharArray(tmpNum[1],5);
        str = EEprom.Time_hold_manual != 0 ? String(EEprom.Time_hold_manual) + "s": "++";
        str.toCharArray(tmpNum[2],5);

        //output
        u8g2.firstPage();
        do {
          u8g2.setFontMode(1);
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.setDrawColor(1);
          u8g2.drawStr(2, 10, "Configuration");
          u8g2.drawStr(100, 10, "MAN");
          
          u8g2.drawStr(4, 25, "T:");
          u8g2.drawStr(4, 37, "time entry:");
          u8g2.drawStr(4, 49, "time hold:");

          for(byte i = 0; i < 3; i++)
            u8g2.drawStr(4+80, 25 + 12*i, tmpNum[i]); 
          
          u8g2.setDrawColor(2); 
          u8g2.drawBox(2 +  (menu_edit == true ? 80:0), 16 + menu_pos*12 , 70 + (menu_edit == true ? -37:0), 11);

        } while(u8g2.nextPage()); 
      }
      TimeSSD = millis();
    }
  }
}

void menu2()
{
  byte menu_pos = 0;
  bool menu_edit = true;
  void* structure_field[7] = {&EEprom.Pulse, 
                              &EEprom.P, 
                              &EEprom.D,
                              &EEprom.I, 
                              &EEprom.thermocorrection,
                              &EEprom.T_Ambient, 
                              &EEprom.ErrorRate};

  
  TimeSSD = millis();
  while(true)
  {
    Time = millis();
    enc1.tick();

    if (enc1.isHolded())
    {
      saveEEPROM();
      return;
    }

    if (enc1.isPress())
      menu_edit = !menu_edit;

    if (enc1.isTurn()) 
    {
      if (menu_edit == false)
      {
        if (enc1.isRight()) 
          menu_pos < 6 ? menu_pos++: menu_pos = 6;
        if (enc1.isLeft())
          menu_pos > 0 ? menu_pos--: menu_pos = 0;
      }
      else
      {
        if (enc1.isRight())
        {
          if (menu_pos >= 0 && menu_pos < 3)
            *((unsigned int*)structure_field[menu_pos]) < 999 ? *((unsigned int*)structure_field[menu_pos]) += 1: *((unsigned int*)structure_field[menu_pos]) = 999;
          else
            if (menu_pos >= 3 && menu_pos < 5)
              *((double*)structure_field[menu_pos]) < 10 ? *((double*)structure_field[menu_pos]) += 0.02: *((double*)structure_field[menu_pos]) = 10;
            else
              *((byte*)structure_field[menu_pos]) < 90 ? *((byte*)structure_field[menu_pos]) += 1: *((byte*)structure_field[menu_pos]) = 90;  
        }

        if (enc1.isLeft())
        {
          if (menu_pos >= 0 && menu_pos < 3)
            *((unsigned int*)structure_field[menu_pos]) > 0 ? *((unsigned int*)structure_field[menu_pos]) -= 1: *((unsigned int*)structure_field[menu_pos]) = 0;
          else
            if (menu_pos >= 3 && menu_pos < 5)
              *((double*)structure_field[menu_pos]) > -10 ? *((double*)structure_field[menu_pos]) -= 0.02: *((double*)structure_field[menu_pos]) = -10;
            else
              *((byte*)structure_field[menu_pos]) > 1 ? *((byte*)structure_field[menu_pos]) -= 1: *((byte*)structure_field[menu_pos]) = 1;
        }

        if (enc1.isFastR())
        {
          if (menu_pos >= 0 && menu_pos < 3)
            *((unsigned int*)structure_field[menu_pos]) < 999-3 ? *((unsigned int*)structure_field[menu_pos]) += 3: *((unsigned int*)structure_field[menu_pos]) = 999;
          else
            if (menu_pos >= 3 && menu_pos < 5)
              *((double*)structure_field[menu_pos]) < 10-0.05 ? *((double*)structure_field[menu_pos]) += 0.05: *((double*)structure_field[menu_pos]) = 10;
            else
              *((byte*)structure_field[menu_pos]) < 90-3 ? *((byte*)structure_field[menu_pos]) += 3: *((byte*)structure_field[menu_pos]) = 90;
        }

        if (enc1.isFastL())
        {
          if (menu_pos >= 0 && menu_pos < 3)
            *((unsigned int*)structure_field[menu_pos]) > 0+3 ? *((unsigned int*)structure_field[menu_pos]) -= 3: *((unsigned int*)structure_field[menu_pos]) = 0;
          else
            if (menu_pos >= 3 && menu_pos < 5)
              *((double*)structure_field[menu_pos]) > -10+3 ? *((double*)structure_field[menu_pos]) -= 0.05: *((double*)structure_field[menu_pos]) = -10;
            else
              *((byte*)structure_field[menu_pos]) > 1+3 ? *((byte*)structure_field[menu_pos]) -= 3: *((byte*)structure_field[menu_pos]) = 1;
        } 
      }
    }
    
    if(Time > TimeSSD + 500)
    {
      String str;
      char tmpNum[7][6] = {};

      //data preparation, conversion to Str
      for(byte i = 0; i < 7; i++)
      {
        switch (i) 
        {
          case 0:
            str = String(*((unsigned int*)structure_field[i])) + "ms";
            break;
          case 1:
            str = String(*((unsigned int*)structure_field[i]));
            break;
          case 2:
            str = String(*((unsigned int*)structure_field[i]));
            break;
          case 3:
            str = String(*((double*)structure_field[i]));
            break;
          case 4:
            str = String(*((double*)structure_field[i])) + "C";
            break;
          case 5:
            str = String(*((byte*)structure_field[i])) + "C";
            break;
          case 6:
            str = String(*((byte*)structure_field[i])) + "%";
            break;
        }
        str.toCharArray(tmpNum[i],6);
      }

      //output
      u8g2.firstPage();
      do {
        u8g2.setFontMode(1);
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);
        u8g2.drawStr(2, 10, "Setting");
          
        u8g2.drawStr(4, 25, "Pu");
        u8g2.drawStr(4, 37, "P");
        u8g2.drawStr(4, 49, "D");
        u8g2.drawStr(4, 61, "I");

        
        u8g2.drawStr(66, 25, "co");
        u8g2.drawStr(66, 37, "am");
        u8g2.drawStr(66, 49, "er");

        for(byte i = 0; i < 7; i++)
          if(i<4)
            u8g2.drawStr(4+25, 25 + 12*i, tmpNum[i]);
          else
            u8g2.drawStr(66+25, 25 + 12*(i-4), tmpNum[i]);
        
        u8g2.setDrawColor(2); 
        u8g2.drawBox((menu_pos<4 ? 2 : 64) + (menu_edit == true ? 25:0), 16 + (menu_pos<4 ? menu_pos: menu_pos-4)*12 , 18+ (menu_edit == true ? 15:0), 11);
      } while(u8g2.nextPage());
      
      TimeSSD = millis();
    }
  }
}

void setup() 
{
  //EEPROM.update(0, 0); // overwriting default values
  
  getEEPROM();

  Serial.begin(9600);
  u8g2.begin();
  
  BottomPID.SetOutputLimits(0, EEprom.Pulse); //regulation limit
  BottomPID.SetMode(MANUAL); //PID to manual (stop)

  Time = millis();
  windowONTime = Time;
  TimePID  = Time;
  TimeCOM = Time;
  TimeMAX = Time;
  TimeProfile = Time;

  pinMode(Pin_HOT, OUTPUT);
  digitalWrite(Pin_HOT, 0);

  pinMode(Pin_ENC_DT, INPUT);          
  digitalWrite(Pin_ENC_DT, HIGH);//20k vcc
  
  pinMode(Pin_ENC_CLK, INPUT);          
  digitalWrite(Pin_ENC_CLK, HIGH);//20k vcc
  
  pinMode(Pin_ENC_SW, INPUT);          
  digitalWrite(Pin_ENC_SW, HIGH);//20k vcc
}

void loop() 
{
  Time = millis();
  unsigned int Prof_Time_sec = (Time - TimeProfileStart)/1000;

  //Profil
  if (on_off == true && Time > TimeProfile + 1000) 
  {
    if(EEprom.Mode < 3)
    {
      if(Prof_Time_sec <= EEprom.TProfile[EEprom.Mode].timer_1)
      {
        if(ProfilStatus < 1)
        ProfilStatus = 1;
        T_Set = (double(EEprom.TProfile[EEprom.Mode].temper_1 - EEprom.T_Ambient)/(EEprom.TProfile[EEprom.Mode].timer_1))*Prof_Time_sec + EEprom.T_Ambient;
      }
      else
        if(Prof_Time_sec <= EEprom.TProfile[EEprom.Mode].timer_1 + EEprom.TProfile[EEprom.Mode].timer_2)
          {
            if(ProfilStatus < 2)
              ProfilStatus = 2;
            T_Set = (double(EEprom.TProfile[EEprom.Mode].temper_2 - EEprom.TProfile[EEprom.Mode].temper_1)/(EEprom.TProfile[EEprom.Mode].timer_2))*(Prof_Time_sec - EEprom.TProfile[EEprom.Mode].timer_1) + EEprom.TProfile[EEprom.Mode].temper_1;
          }
        else
          if(Prof_Time_sec <= EEprom.TProfile[EEprom.Mode].timer_1 + EEprom.TProfile[EEprom.Mode].timer_2 + EEprom.TProfile[EEprom.Mode].timer_3/2)
            {
              if(ProfilStatus < 3)
              ProfilStatus = 3;
              T_Set = (double(EEprom.TProfile[EEprom.Mode].temper_3 - EEprom.TProfile[EEprom.Mode].temper_2)/(EEprom.TProfile[EEprom.Mode].timer_3/2))*(Prof_Time_sec - EEprom.TProfile[EEprom.Mode].timer_1 - EEprom.TProfile[EEprom.Mode].timer_2) + EEprom.TProfile[EEprom.Mode].temper_2;
            }
          else
              if(Prof_Time_sec <= EEprom.TProfile[EEprom.Mode].timer_1 + EEprom.TProfile[EEprom.Mode].timer_2 + EEprom.TProfile[EEprom.Mode].timer_3)
                {
                  if(ProfilStatus < 4)
                    ProfilStatus = 4;
                  T_Set = (double(EEprom.TProfile[EEprom.Mode].temper_2 - EEprom.TProfile[EEprom.Mode].temper_3)/(EEprom.TProfile[EEprom.Mode].timer_3/2))*(Prof_Time_sec - EEprom.TProfile[EEprom.Mode].timer_1 - EEprom.TProfile[EEprom.Mode].timer_2- EEprom.TProfile[EEprom.Mode].timer_3/2) + EEprom.TProfile[EEprom.Mode].temper_3;
                }
              else
                if(Prof_Time_sec <= EEprom.TProfile[EEprom.Mode].timer_1 + EEprom.TProfile[EEprom.Mode].timer_2 + EEprom.TProfile[EEprom.Mode].timer_3 + EEprom.TProfile[EEprom.Mode].timer_4)
                  {
                    if(ProfilStatus < 5)
                      ProfilStatus = 5;
                    T_Set = (double(EEprom.TProfile[EEprom.Mode].temper_4 - EEprom.TProfile[EEprom.Mode].temper_2)/(EEprom.TProfile[EEprom.Mode].timer_4))*(Prof_Time_sec - EEprom.TProfile[EEprom.Mode].timer_1 - EEprom.TProfile[EEprom.Mode].timer_2 - EEprom.TProfile[EEprom.Mode].timer_3) + EEprom.TProfile[EEprom.Mode].temper_2;
                  }
                else
                  StopHot();
    }
    else
    {
      if(Prof_Time_sec <= EEprom.Time_entry_manual)
      {
        T_Set = (double(EEprom.T_manual - EEprom.T_Ambient)/EEprom.Time_entry_manual)*Prof_Time_sec + EEprom.T_Ambient;
      }
      else
        if(EEprom.Time_hold_manual != 0 && Prof_Time_sec > EEprom.Time_entry_manual + EEprom.Time_hold_manual)
          StopHot();
        else
          T_Set = EEprom.T_manual;
    }

    TimeProfile = millis();
  }
  
  //MAX
  if (Time > TimeMAX + 500) 
  {
    T_Bottom = temperature_bottom.readCelsius() + EEprom.thermocorrection;

    //thermocouple test
    if(on_off == true)
    {
      if(ErrorRate_count <= 5)
      {
        ErrorRate_buf = (ErrorRate_buf +(T_Bottom/T_Set)*100)/2;
        ErrorRate_count++;
      }
      else 
      {
        if(100 - ErrorRate_buf > EEprom.ErrorRate)
        {
          String tmp = "";
          tmp += String(100 - ErrorRate_buf);
          tmp += "\% > ";
          tmp += String(EEprom.ErrorRate);
          tmp += "\%";
          char charVar[11];
          tmp.toCharArray(charVar, 11);
          
          StopHot();

          u8g2.firstPage();
          do {
            u8g2.setFontMode(1);
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setDrawColor(1);
            u8g2.drawStr(40, 10, "ERROR!!!");
            u8g2.drawStr(19, 25, "NO Thermocouple");
            u8g2.drawStr(37, 40, charVar);
            u8g2.drawStr(52, 55, " OK ");
            u8g2.setDrawColor(2); 
            u8g2.drawBox(52, 47, 24, 10);
          } while(u8g2.nextPage() );
          Serial.print("ERROR ");
          Serial.print(tmp);
          Serial.print("\n");

          while(true)
          {
            enc1.tick();
            delay(200);
            if (enc1.isHold())
              break;
          }
        }
        ErrorRate_count = 0;
        ErrorRate_buf = 0;
      }
    }

    TimeMAX = millis();
  }

  //PID
  if(on_off == true)
  {
    if (Time > TimePID + 200) 
    {
      InputBottom = T_Bottom;
      BottomPID.Compute();
      TimePID = millis();
    }

    unsigned long Time_now = millis();
    if (Time_now - windowONTime > EEprom.Pulse)
      windowONTime += EEprom.Pulse;

    if (OutBottom > (Time_now - windowONTime))
      digitalWrite(Pin_HOT, 1);
    else
      digitalWrite(Pin_HOT, 0);
  }

  //UART
  if (on_off == true && Time > TimeCOM + 1000) 
  {
    Serial.print("Mode: ");
    Serial.print(EEprom.Mode);
    Serial.print("\n");

    if(EEprom.Mode < 3)
    {
      Serial.print("ProfilStatus: ");
      Serial.print(ProfilStatus);
      Serial.print("\n");
    }
    else
    {
      Serial.print("Temperature MAX: ");
      Serial.print(EEprom.T_manual);
      Serial.print("\n");
    }
    
    Serial.print("Temperature: ");
    Serial.print(T_Bottom);
    Serial.print("\n");
    Serial.print("Temperature Set:  ");
    Serial.print(T_Set);
    Serial.print("\n");
    Serial.print("Pulse Out:  ");
    Serial.print(OutBottom);
    Serial.print("\n");
    Serial.print("Time:  ");
    Serial.print(Prof_Time_sec);
    Serial.print("\n");
    Serial.print("\n");

    TimeCOM = millis();
  }

  //display
  if (Time > TimeSSD + (on_off == true ? 500 : 100)) 
  {
    //preliminary calculation of the scale of the schedule
    const byte x0 =44;
    const byte y0 =52;
    const byte x1 =120;
    const byte y1 =2;
    double scaleX = 0;
    double scaleY = 0;
    void* structure_field[8] = {&EEprom.TProfile[EEprom.Mode].temper_1, 
                                &EEprom.TProfile[EEprom.Mode].temper_2, 
                                &EEprom.TProfile[EEprom.Mode].temper_3,
                                &EEprom.TProfile[EEprom.Mode].temper_4, 
                                &EEprom.TProfile[EEprom.Mode].timer_1,
                                &EEprom.TProfile[EEprom.Mode].timer_2, 
                                &EEprom.TProfile[EEprom.Mode].timer_3,
                                &EEprom.TProfile[EEprom.Mode].timer_4};

    if(EEprom.Mode <3)
    {
      for (byte i = 4; i < 8; i++)
        scaleX += *((unsigned int*)structure_field[i]);
      scaleX = scaleX/(x1-x0);
      
      for (byte i = 0; i < 4; i++)
        if(scaleY < *((int*)structure_field[i]))
          scaleY = *((int*)structure_field[i]);
      scaleY = scaleY/(y0-y1);
    }

    const void* SSD_field[5] = {&ProfilStatus, 
                                &T_Set, 
                                &T_Bottom,
                                &EEprom.T_manual, 
                                &Prof_Time_sec};

    //data preparation, conversion to Str
    String str;
    char tmpSSD[5][5] = {};
    for(byte i = 0; i < 5; i++)
    {
      if(i == 0)
        str = String(*((byte*)SSD_field[i]));
      else
        if(i == 4)
          str = String(*((unsigned int*)SSD_field[i])) + "s";
        else
          if(i == 3)
            str = String(round(*((int*)SSD_field[i]))) + "C";
          else
            str = String(round(*((double*)SSD_field[i]))) + "C";
          
      str.toCharArray(tmpSSD[i],5);
    }

    //output
    u8g2.firstPage();
    do {
      u8g2.setFontMode(1);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setDrawColor(1);
      u8g2.drawStr(2, 62, " M1 ");
      u8g2.drawStr(25+2, 62, " M2 ");
      u8g2.drawStr(50+2, 62, " M3 ");
      u8g2.drawStr(73, 62, " MAN ");

      
      u8g2.drawStr(2, 10, "T:");
      u8g2.drawStr(13+2, 10,  tmpSSD[2]);
      
      if(EEprom.Mode == 3)
      {
        u8g2.drawStr(2, 20, "S:");
        u8g2.drawStr(13+2, 20,  tmpSSD[1]);

        u8g2.drawStr(2, 30, "M:");
        u8g2.drawStr(13+2, 30,  tmpSSD[3]);

      }
      else
      {
        u8g2.drawStr(2, 20, "S:");
        u8g2.drawStr(13+2, 20,  tmpSSD[1]);
        u8g2.drawStr(2, 30, "P:");
        u8g2.drawStr(13+2, 30,  tmpSSD[0]);
      }

      u8g2.setDrawColor(2); 
      u8g2.drawBox(2+(EEprom.Mode*25), 54, 22, 11);
      

      if(on_off == true)
      {
        u8g2.drawStr(2, 40, "t:");
        u8g2.drawStr(13+2, 40,  tmpSSD[4]);

        u8g2.setFont(u8g2_font_6x10_tf);//u8g2_font_unifont_t_symbols
        u8g2.drawUTF8(110, 62, "ON");//"☕"
      }

      //plotting
      if(EEprom.Mode < 3)
      {
        u8g2.drawLine(x0, y0, x0, y1);
        u8g2.drawLine(x0, y0, x1, y0);

        unsigned int tmpGraph = *((unsigned int*)structure_field[4]);
        u8g2.drawLine(x0, 
                      y0 - round(EEprom.T_Ambient/scaleY), 
                      x0 + round(tmpGraph/scaleX), 
                      y0 - round(*((int*)structure_field[0])/scaleY));
        
      
        u8g2.drawLine(x0 + round(*((unsigned int*)structure_field[4])/scaleX), 
                      y0 - round(*((int*)structure_field[0])/scaleY), 
                      x0 + round((tmpGraph + *((unsigned int*)structure_field[5]))/scaleX), 
                      y0 - round(*((int*)structure_field[1])/scaleY));

        tmpGraph += *((unsigned int*)structure_field[5]);
        u8g2.drawLine(x0 + round(tmpGraph/scaleX), 
                      y0 - round(*((int*)structure_field[1])/scaleY), 
                      x0 + round((tmpGraph+*((unsigned int*)structure_field[6])/2)/scaleX), 
                      y0 - round(*((int*)structure_field[2])/scaleY));
        
        u8g2.drawLine(x0 + round((tmpGraph+*((unsigned int*)structure_field[6])/2)/scaleX), 
                      y0 - round(*((int*)structure_field[2])/scaleY), 
                      x0 + round((tmpGraph+*((unsigned int*)structure_field[6]))/scaleX), 
                      y0 - round(*((int*)structure_field[1])/scaleY));
        
        
        tmpGraph += *((unsigned int*)structure_field[6]);
        u8g2.drawLine(x0 + round(tmpGraph/scaleX), 
                      y0 - round(*((int*)structure_field[1])/scaleY), 
                      x0 + round((tmpGraph+*((unsigned int*)structure_field[7]))/scaleX), 
                      y0 - round(*((int*)structure_field[3])/scaleY));

        if(on_off == true)
          u8g2.drawLine(x0 + round(*((unsigned int*)SSD_field[4])/scaleX), 
                        y0, 
                        x0 + round(*((unsigned int*)SSD_field[4])/scaleX), 
                        y1);
      }
    }while(u8g2.nextPage());

    TimeSSD = millis();
  }

  //encoder
  enc1.tick();
  if (enc1.isTurn()) 
  {
    if(on_off == false)
    {
      if (enc1.isRightH())
        menu2();

      if (enc1.isLeftH())
        menu1();

      if (enc1.isRight()) 
        EEprom.Mode >= 3 ? EEprom.Mode = 3: EEprom.Mode++; 
      else
        if (enc1.isLeft()) 
          EEprom.Mode <= 0 ? EEprom.Mode = 0: EEprom.Mode--; 
    }
    else
    {
      if(EEprom.Mode == 3)
      {
        if (enc1.isRight()) 
          EEprom.T_manual < 350 ? EEprom.T_manual++: EEprom.T_manual = 350; 
        else
          if (enc1.isLeft()) 
            EEprom.T_manual > EEprom.T_Ambient ? EEprom.T_manual--: EEprom.T_manual = EEprom.T_Ambient;

        if (enc1.isFastR()) 
          EEprom.T_manual < 347 ? EEprom.T_manual +=3: EEprom.T_manual = 350; 
        else
          if (enc1.isFastL()) 
            EEprom.T_manual > EEprom.T_Ambient+3 ? EEprom.T_manual -=3: EEprom.T_manual = EEprom.T_Ambient;
      }
    }
  }

  if (enc1.isHolded()) 
  {
    if(on_off == true)
      StopHot();
    else
    {
      saveEEPROM();
      RunHot(EEprom.Mode);
    }
  }
}