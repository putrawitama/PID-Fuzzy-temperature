#include <OneWire.h>

#include <DallasTemperature.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// sensor diletakkan di pin 2
#define ONE_WIRE_BUS 2
 
// setup sensor
OneWire oneWire(ONE_WIRE_BUS);
 
// berikan nama variabel,masukkan ke pustaka Dallas
DallasTemperature sensorSuhu(&oneWire);

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

int PWM_pin = 3;
float set_temperature = 69;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
int reading = 0;
float temperature_read = 0.0;
float PID_error = 0;
float d_PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
float last_set_temperature = 0;
float error[3];
float d_error[3];
float rule[9];

// int kp = 90;   int ki = 30;   int kd = 80;
int kp = random(50, 100);   int ki = random(0, 50);   int kd = random(40, 90);
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;


void setup()
{
  Serial.begin(9600);
  Time = millis();
  // initialize the LCD
  lcd.begin();
  lcd.backlight();
  //lcd.begin();

  // Turn on the blacklight and print a message.
  //lcd.backlight();
  lcd.print("Hello, world!");
}

void loop()
{
  temperature_read = ambilSuhu();
  PID_error = set_temperature - temperature_read;

  if(PID_error >= previous_error) {
    fuzzify();
    ruleEva();
    defuzzyfy();
  }

  PID_p = 0.01*kp * PID_error;
  PID_i = 0.01*PID_i + (ki * PID_error);

  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;


  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }

  //grafik
  Serial.println(temperature_read);
//  Serial.print(" ");
//  Serial.println(PID_error);

  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
  d_PID_error = abs(previous_error - PID_error);
  last_kp = kp;
  last_ki = ki;
  last_kd = kd;


  delay(250);

  lcd.setCursor(0,0);
  lcd.print("P:");
  lcd.setCursor(2,0);
  lcd.print(kp,1);

  lcd.setCursor(4,0);
  lcd.print(" ");
  
  lcd.setCursor(5,0);
  lcd.print("I:");
  lcd.setCursor(7,0);
  lcd.print(ki,1);

  lcd.setCursor(9,0);
  lcd.print(" ");

  lcd.setCursor(10,0);
  lcd.print("D:");
  lcd.setCursor(12,0);
  lcd.print(kd,1);

  
  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.setCursor(2,1);
  lcd.print(set_temperature,1);
  lcd.setCursor(9,1);
  lcd.print("R:");
  lcd.setCursor(11,1);
  lcd.print(temperature_read,1);                
  // Do nothing here...
}

float ambilSuhu()
{
   sensorSuhu.requestTemperatures();
   float suhu = sensorSuhu.getTempCByIndex(0);
   return suhu;   
}

void fuzzify(){
  // untuk PID_error Rendah
  if (PID_error <= 25)
  { error [0] = 1;}
  else if (PID_error > 25 && PID_error <= 50)
  {  error [0] = (50 - PID_error)/(50 - 25); }
  else
  { error [0] = 0;}
 
  // untuk PID_error Sedang
  if (PID_error <= 25)
  { error [1] = 0;}
  else if (PID_error > 25 && PID_error <= 50)
  { error [1] = (PID_error-25)/(50-25);}
  else if (PID_error > 50 && PID_error <= 75)
  { error [1] = (75-PID_error)/(75 - 50);}
  else
 { error [1] = 0;}
 
  // untuk PID_error Tinggi
  if (PID_error <= 50)
  { error [2] = 0;}
  else if (PID_error > 50 && PID_error <= 75)
  { error [2] = (PID_error-50)/(75-50);}
  else
  { error [2] = 1;}



  // untuk delta_error Rendah
  if (d_PID_error <= 0.25)
  { d_error [0] = 1;}
  else if (d_PID_error > 0.25 && d_PID_error <= 0.50)
  {  d_error [0] = (0.50 - d_PID_error)/(0.50 - 0.25); }
  else
  { d_error [0] = 0;}
 
  // untuk delta_error Sedang
  if (d_PID_error <= 0.25)
  { d_error [1] = 0;}
  else if (d_PID_error > 0.25 && d_PID_error <= 0.50)
  { d_error [1] = (d_PID_error-0.25)/(0.50-0.25);}
  else if (d_PID_error > 0.50 && d_PID_error <= 0.75)
  { d_error [1] = (0.75-d_PID_error)/(0.75 - 0.50);}
  else
 { d_error [1] = 0;}
 
  // untuk delta_error Tinggi
  if (d_PID_error <= 0.50)
  { d_error [2] = 0;}
  else if (d_PID_error > 0.50 && d_PID_error <= 0.75)
  { d_error [2] = (d_PID_error-0.50)/(0.75-0.50);}
  else
  { d_error [2] = 1;}
}

void ruleEva() {
  int k = 0;
  for (int i=0; i<=2; i=i+1) {
    for (int j=0; j<=2; j=j+1) {
      rule[k] = min(error[i], d_error[j]);
      k++;
    }
  }
}

void defuzzyfy() {
  float kp_[] = {62.5, 75, 87.5};
  float ki_[] = {12.5, 25, 37.5};
  float kd_[] = {52.5, 65, 77.5};

    float pemb = 0, pemy = 0;
    pemb = (rule[0]*kp_[0])+(rule[1]*kp_[0])+(rule[2]*kp_[0])+(rule[3]*kp_[1])+(rule[4]*kp_[1])+(rule[5]*kp_[1])+(rule[6]*kp_[2])+(rule[7]*kp_[2])+(rule[8]*kp_[2]);
    pemy = rule[0]+rule[1]+rule[2]+rule[3]+rule[4]+rule[5]+rule[6]+rule[7]+rule[8];

    kp = pemb/pemy;

    pemb = 0;
    pemy = 0;
    pemb = (rule[0]*ki_[0])+(rule[1]*ki_[0])+(rule[2]*ki_[0])+(rule[3]*ki_[1])+(rule[4]*ki_[1])+(rule[5]*ki_[1])+(rule[6]*ki_[2])+(rule[7]*ki_[2])+(rule[8]*ki_[2]);
    pemy = rule[0]+rule[1]+rule[2]+rule[3]+rule[4]+rule[5]+rule[6]+rule[7]+rule[8];

    ki = pemb/pemy;

    pemb = 0;
    pemy = 0;
    pemb = (rule[0]*kd_[0])+(rule[1]*kd_[0])+(rule[2]*kd_[0])+(rule[3]*kd_[1])+(rule[4]*kd_[1])+(rule[5]*kd_[1])+(rule[6]*kd_[2])+(rule[7]*kd_[2])+(rule[8]*kd_[2]);
    pemy = rule[0]+rule[1]+rule[2]+rule[3]+rule[4]+rule[5]+rule[6]+rule[7]+rule[8];

    kd = pemb/pemy; 

}
