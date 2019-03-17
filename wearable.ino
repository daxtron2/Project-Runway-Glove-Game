//TJ Wolschon
//Wearables Project
//PINS:
//      - D2 = PALM WIRE
//      - D11 = BUZZER
//      - A5 = SCL
//      - A4 = SDL
//      - GND = BUZZER, D2 RESISTOR, FLORA GND
//      - 3.3v = FLORA 3.3v, FINGER WIRE

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "pitches.h"
#include <math.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

int targetX = 0;
int targetY = 0;
int targetZ = 0;
bool isShooting = false; 
int points = 0;

const int SENSITIVITY = 10; // higher == easier
const int POINTS_TO_WIN = 5;
const int POINTING_NOTE = NOTE_C4;
const int buzzerPin = 11;


void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float DistanceToTarget(float playerHeading)
{
  float delta = fabs(playerHeading - targetY);
  float modDelta = fmod(delta, 360.0);
  float distance = modDelta > 180.0 ? 360.0 - modDelta : modDelta;
  return distance;
}

//from the example sketch toneMelody
void PlayJingle()
{
    // notes in the melody:
    int melody[] = {
        NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
        4, 8, 8, 4, 4, 4, 4, 4};

    for (int thisNote = 0; thisNote < 8; thisNote++)
    {
        // to calculate the note duration, take one second divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(buzzerPin, melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        // stop the tone playing:
        noTone(buzzerPin);
    }
}

float GetHeading()
{
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.z, event.magnetic.y)*180)/PI;
  if(heading < 0)
  {
    heading += 360;
  }
  return heading;
}

void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  randomSeed(analogRead(0));
}

void loop(void)
{
  
  isShooting = digitalRead(2);

  float heading = GetHeading();
  float distance = DistanceToTarget(heading);

  int note = -distance + POINTING_NOTE;
  
  Serial.print("Points: "); Serial.print(points);                      Serial.print("  ");
  Serial.print("Shooting: "); Serial.print(isShooting);                Serial.print("  ");
  Serial.print("Distance: "); Serial.print(distance); Serial.print("  ");
  Serial.print("Target Bearing: "); Serial.print(targetY);             Serial.print("  ");
  Serial.print("Player Bearing: "); Serial.println(heading);

  if(distance > SENSITIVITY){
    tone(buzzerPin, note, 20);  
  }
  else{
    tone(buzzerPin, POINTING_NOTE + 100, 50);
  }
  
    
  if(CollisionCheck(distance))
  {
    points++;
  }


  if( points >= POINTS_TO_WIN)
  {
    Serial.println("YOU WON!!!");
    PlayJingle();
    while(1);
  }
}

void SetNewTarget()
{
  int oldTargetY = targetY;
  targetY = random(0, 360);
  while(abs(oldTargetY - targetY) < 20){
      targetY = random(0, 360);
  }
}

bool CollisionCheck(float distance)
{
  if(!isShooting){
    return false;
  }
  else if( distance < SENSITIVITY )
  {
    tone(buzzerPin, NOTE_A1, 1000);
    delay(1000);
    Serial.println("HITTING TARGET");
    SetNewTarget();
    return true;
  }
  else
  {
    return false;
  }
}
