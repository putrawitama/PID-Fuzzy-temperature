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
float set_temperature = 70;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
int reading = 0;
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
float last_set_temperature = 0;


int kp = 90;   int ki = 30;   int kd = 80;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

int sensorPin = A5;

float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;


void setup()
{

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
  PID_error = set_temperature - temperature_read + 3;

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

  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.

  delay(250);

  lcd.setCursor(0,0);
  lcd.print("PID TEMP control");
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
