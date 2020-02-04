#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Servo esc;
void setup() {
pinMode( 7, INPUT); //top limit switch
pinMode( 6, INPUT); // bottom limit switch
esc.attach(8); //servo pin
Serial.begin(74880);
Serial.println("Air brake test code"); Serial.println("");
if(!bno.begin())
{ Serial.print("No detection"); while(1); }
delay(100);
bno.setExtCrystalUse(true);
}
void loop(void) { 
  if (! baro.begin()) 
     { Serial.println("Couldnt find sensor"); return; }
int TargetHeight=1200; //meters
float vel;
float alt;
float targetvel;
int launch=1; //launch trigger
int flaps=1; // flap trigger
int z; // orientation value
int threshold=50; // lowest velocity airbrakes will work
delay(100);
//Launch Detection Loop//
while(launch == 1) {
    if( digitalRead(7) == HIGH ) {
    esc.write(180); delay(5);} // if motor is open
    if( digitalRead(7) == LOW ) {
    esc.write(90); delay(5);} // if motor is closed
    //gathering z acc. data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
           Serial.print("Waiting for launch ");
           Serial.print("\tAcceleration z: ");
           Serial.print(abs(euler.z()), 4);
           Serial.print("\n");
    z=abs(euler.z());
    delay(50);
//acceleration needed to detect launch m/s^2
if ( z > 4 ) { 
          Serial.print("\nLaunch detected");
          Serial.print("\tmotor burning for 1.8 sec");
          delay(2100);
          launch=0;
}
}
//Main Airbrake programProgram//    
while(1) { 
vel=velocity();
sensors_event_t event; 
bno.getEvent(&event);
alt=baro.getAltitude();
targetvel=targetvelocity(alt);
if( vel > threshold && vel > targetvel && digitalRead(6) == (HIGH)) {
  //open flaps
  esc.write(0);
  delay(10);
  esc.write(92);
  Serial.print("Openflaps");
}
if((vel < threshold && digitalRead(7) == (HIGH)) || (vel < targetvel && digitalRead(7) == (HIGH)))  {
  //close flaps
  esc.write(180);
  delay(10);
  esc.write(92);
  Serial.print("Close Flaps");
}
}
}
float velocity() {
  sensors_event_t event; 
  bno.getEvent(&event);
  float a1 = baro.getAltitude(); //altitude check 1
  delay(100);
  float a2 = baro.getAltitude(); //altitude check 2
  float vel = abs((a2-a1)/(.1)); //delta altitude over time = vertical vel
  Serial.print("\tVelocity:");
  Serial.print(vel);
return(vel);
}
float targetvelocity(float a) {
   float targetvel = (-0.000000000004*(a*a*a*a*a*a)) + (0.000000006*(a*a*a*a*a)) - (0.000004*(a*a*a*a)) + (0.0012*(a*a*a)) - (0.2193*(a*a)) + 20.271*a - 667.28;
   Serial.print("\tTarget velocity:");
   Serial.print(targetvel);
   return(targetvel);  
}
