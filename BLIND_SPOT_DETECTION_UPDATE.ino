#include <LiquidCrystal_I2C.h>
#include <NewPing.h>

//assign pin to ultrasonics
#define TRIGGER_PIN_RF  2         //Front right 
#define ECHO_PIN_RF     3         //Front right
#define TRIGGER_PIN_LF  4         //Front left
#define ECHO_PIN_LF     5         //Front left
#define TRIGGER_PIN_RR  6         //Rear right
#define ECHO_PIN_RR     7         //Rear right
#define TRIGGER_PIN_LR  8         //Rear left
#define ECHO_PIN_LR     9         //Rear left

//assign other components' pin
#define RED_LED         10
#define GREEN_LED       11
#define BUZZER          12

#define MAX_DISTANCE    200       //Maximum distance for ultrasonic to detect
#define BLIND_DIST      5         //distance for car enter blind spot in cm

LiquidCrystal_I2C lcd(0x27,16,2); //setting up LCD

//setting up ultrasonics
NewPing sonarRF(TRIGGER_PIN_RF, ECHO_PIN_RF, MAX_DISTANCE);
NewPing sonarLF(TRIGGER_PIN_LF, ECHO_PIN_LF, MAX_DISTANCE);
NewPing sonarRR(TRIGGER_PIN_RR, ECHO_PIN_RR, MAX_DISTANCE);
NewPing sonarLR(TRIGGER_PIN_LR, ECHO_PIN_LR, MAX_DISTANCE);

void setup()
{
  //initialize LCD
  lcd.begin();
  lcd.clear();         
  lcd.backlight(); 
  //initialize Nano's Pin 
  pinMode(RED_LED,OUTPUT); 
  pinMode(GREEN_LED,OUTPUT); 
  pinMode(BUZZER, OUTPUT);
  Serial.begin(9600); 
}

void loop()
{
  int distRF = sonarRF.ping_cm();
  int distLF = sonarLF.ping_cm();
  int distRR = sonarRR.ping_cm();
  int distLR = sonarLR.ping_cm();

  if(distRF<=BLIND_DIST)
  {
    if(distRF<=BLIND_DIST && distRR<=BLIND_DIST) //condition for both front and rear right sensor detected
    {
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("RIGHT SIDE");
      Serial.println("Right Side");
      delay(100);
    }
    else                                        //condtion if only front right sensor detected
    { 
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("RIGHT FRONT");
      Serial.println("Right Front");
      delay(100);
    }
  }

  if(distRR<=BLIND_DIST)
  {
    if(distRF<=BLIND_DIST && distRR<=BLIND_DIST) //condition for both front and rear right sensor detected
    {
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("RIGHT SIDE");
      Serial.println("Right Side");
      delay(100);
    }
    else                                          //condtion if only rear right sensor detected
    {
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("RIGHT REAR");
      Serial.println("Right rear");
      delay(100);
    }
  }
  
  if(distLF<=BLIND_DIST)
  {
    if(distLF<=BLIND_DIST && distLR<=BLIND_DIST) //condition for both front and rear left sensor detected
    {
     digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("LEFT SIDE");
      Serial.println("Left Side");
      delay(100); 
    }
    else                                          //condtion if only front left sensor detected
    {
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("LEFT FRONT");
      Serial.println("Left Front");
      delay(100);
    }
  }
  
  if(distLR<=BLIND_DIST)
  {
    //
    if(distLF<=BLIND_DIST && distLR<=BLIND_DIST) //condition for both front and rear left sensor detected
    {
     digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("LEFT SIDE");
      Serial.println("Left Side");
      delay(100); 
    }
    else                                        //condtion if only rear left sensor detected
    {
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      digitalWrite(GREEN_LED,LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CAR PRESENT:");
      lcd.setCursor(0,1);
      lcd.print("LEFT REAR");
      Serial.println("Left Rear");
      delay(100);
    }
  }
    
  //condition if none of the sensor detected
  if(distLR>BLIND_DIST && distRR>BLIND_DIST && distLF>BLIND_DIST && distRF>BLIND_DIST)
  {
    digitalWrite(BUZZER,LOW);
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,HIGH);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("CLEAR, NO CAR");
    delay(100);
  }
}
