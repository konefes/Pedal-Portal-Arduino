#include <avr/sleep.h>
#include <avr/interrupt.h>

// Digital pin connected to PIR sensor (input)
#define pirPin 3
//Digital pin connected to PIs
#define pi1 8
#define pi2 9
#define pi3 10
// Digital pin connected to ethernet switch (output)
#define ethPin 11
// Digital pin connected to LED
#define led 13
// Pin value ( 0 or 1)
int  flag = 0;
// To record time input pin (3) transitioned from low to high
long int transTime = 0;
//To record time input pin (3) transitioned from high to low
long int lowIn;
// The time we give the sensor to calibrate
int calibrationTime = 13;
// To make sure we wait for a transition to HIGH before any further output is made
boolean lockHigh = true;
// To mkae sure we wait for a transition to LOW before any further output is made
boolean lockLow = true;
// To make sure we take the high time, time pin is high, only when no motion is detected
boolean takeHighTime;
// To make sure we take the low time only when no motion is detected
boolean takeLowTime;

//WakeUp when pirPin is high
void wakeUp(void)
{
  //Go inside loop
}

// Setup process
void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);      // set pin 3 as input
  pinMode(ethPin, OUTPUT);     // set pin 13 as output
  pinMode(pi1, OUTPUT);
  pinMode(pi2, OUTPUT);
  pinMode(pi3, OUTPUT);
  pinMode(led, OUTPUT);
  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  attachInterrupt(1, wakeUp, HIGH); // use interrupt 1 (pin 3) and run function
  // wakeUpNow when pin 2 gets LOW
  sleepNow();
  delay(50);
}

//Loop
void loop()
{
  flag = digitalRead(pirPin); // assign pin_3 value to flag
  Serial.print("flag:");
  Serial.println(flag);

  // If pin_3 is low, record time it went low and compare to curent time
  // If the difference is more than 2 min then set all pin low and sleep
  if (flag == LOW) {
    if (takeLowTime) {
      lowIn = millis();          //save the time of the transition from high to LOW
      takeLowTime = false;       //make sure this is only done at the start of a LOW phase
    }
    Serial.println("Pin is low");

    //makes sure we wait for a transition to High before any further output is made:
    if (lockHigh) {
      lockHigh = false;
      Serial.println("---");
      Serial.println("Waiting for transistion to high");
      delay(50);
    }
    //if the sensor is low for more than 2min=120000ms,
    //we assume that no more motion is going to happen
    if (!lockLow && millis() - lowIn > 120000) {
      //makes sure this block of code is only executed again after
      //a new motion sequence has been detected
      lockLow = true;
      Serial.print("No motion for 6s ");      //output
      digitalWrite(ethPin, LOW);   //the led visualizes the sensors output pin state
      digitalWrite(pi1, LOW);
      digitalWrite(pi2, LOW);
      digitalWrite(pi3, LOW);
      digitalWrite(led, LOW);
      sleepNow();
      delay(50);
    }
    takeHighTime = true;
  }

  // If pin_3 is high, wake up and record time it went high and compare to curent time
  // If the difference is more than 15 sec then set all pin high
  if (flag == HIGH) {
    if (takeHighTime) {
      transTime = millis();          //save the time of the transition from LOW to HIGH
      takeHighTime = false;       //make sure this is only done at the start of a LOW phase
    }
    Serial.print("Transit to high time: ");
    Serial.println(transTime);

    //makes sure we wait for a transition to High before any further output is made:
    if (lockLow) {
      lockLow = false;
      Serial.println("---");
      Serial.println("Waiting for transistion to Low");
      delay(50);
    }
    // if the sensor is high for 15 sec, turn ethernet hub on
    if (!lockHigh && (millis() - transTime) > 15000 )
    {
      //makes sure this block of code is only executed again after
      //a new motion sequence has been detected
      lockHigh = true;
      Serial.println("10 sec of motion done");
      digitalWrite(ethPin, HIGH);
      digitalWrite(pi1, HIGH);
      digitalWrite(pi2, HIGH);
      digitalWrite(pi3, HIGH);
      digitalWrite(led, HIGH);
      delay(50);
    }
    takeLowTime = true;
  }

}

// Put the arduino into power down sleep mode and wake up when 
// interupt 1, pin 3, is high
void sleepNow()
{
  Serial.println("Entering Sleep");
  // Set pin 3 as interrupt and attach handler:
  delay(100);
  // Choose our preferred sleep mode:
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Set sleep enable (SE) bit:
  sleep_enable();

  attachInterrupt(1, wakeUp, HIGH);

  // Put the device to sleep:
  sleep_mode();

  // Upon waking up, sketch continues from this point.
  sleep_disable();

  detachInterrupt(1);// disables interrupt 1 on pin 3 so the
  // wakeUp code will not be executed
  // during normal running time.
  wakeUp();
}

