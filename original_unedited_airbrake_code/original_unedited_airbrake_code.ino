#include <Servo.h>
#include <Wire.h>
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
Serial.begin(9600);
Serial.println("Air brake test code"); Serial.println("");
if(!bno.begin())
{ Serial.print("No detection"); while(1); }
delay(1000);
bno.setExtCrystalUse(true);
}
void loop(void) {
  if (! baro.begin()) 
     { Serial.println("Couldnt find sensor"); return; }
int TargetHeight=300; //meters
int Cd = -1000 ;//drag factor we calculate using a function of velocity
int launch=1; //launch trigger
int flaps=1; // flap trigger
int z; // orientation value
int threshold=0; // lowest velocity airbrakes will work
sensors_event_t event; //grabbing sensor data
bno.getEvent(&event);
delay(100);
//Launch Detection Loop//
while(launch == 1) {
    while( digitalRead(7) == HIGH ) {
    esc.write(180); delay(5);} // motor set to hold position
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
          delay(1800);
//Main Airbrake programProgram//    
while(1) { 
  sensors_event_t event; 
  bno.getEvent(&event);
float a1 = baro.getAltitude(); //altitude check 1
delay(100);
float a2 = baro.getAltitude(); //altitude check 2
float vel = ((abs(a2)-abs(a1))/(.1)); //delta altitude over time = vertical vel
float Hp = ((vel*vel)*(sin(event.orientation.x)*(3.1415/180)))/(2*9.81) + a2 - Cd;
Serial.print("\ta1="); // debugging tool
Serial.print(a1, 4);
Serial.print("meters");
Serial.print("\ta2=");
Serial.print(a2, 4);
Serial.print("meters");
Serial.print("\tvel=");
Serial.print(vel, 4);
Serial.print("m/s");
Serial.print("\tthreshold= ");
Serial.print(threshold);
Serial.print("m/s\n");
//When Rocket Is Near Apogee//
if( vel <= threshold ) {
      Serial.print("\nVelocity Under Threshold closing flaps");
         //Closing Flaps
         if( digitalRead(7) == HIGH ) {
         esc.write(180);
         delay(1);
         if( digitalRead(7) == LOW ) {
         esc.write(90);
                                     }
                                      }
//Get Sensor Data//
  sensors_event_t event; 
  bno.getEvent(&event);
a1 = baro.getAltitude(); //altitude check 1
delay(100);
a2 = baro.getAltitude(); //altitude check 2
vel = ((abs(a2)-abs(a1))/(.1)); //delta altitude over time = vertical vel
Hp = ((vel*vel)*(sin(event.orientation.x)*(3.1415/180)))/(2*9.81) + a2 - Cd;
Serial.print("\ta1="); // debugging tool
Serial.print(a1, 4);
Serial.print("meters");
Serial.print("\ta2=");
Serial.print(a2, 4);
Serial.print("meters");
Serial.print("\tvel=");
Serial.print(vel, 4);
Serial.print("m/s");
Serial.print("\tthreshold= ");
Serial.print(threshold);
Serial.print("m/s\n");
}
//Rocket Speed Above Limit//
if (vel >= threshold) {
  sensors_event_t event; 
  bno.getEvent(&event);
a1 = baro.getAltitude(); //altitude check 1
delay(100);
a2 = baro.getAltitude(); //altitude check 2
vel = ((abs(a2)-abs(a1))/(.1)); //delta altitude over time = vertical vel
Hp = ((vel*vel)*(sin(event.orientation.x)*(3.1415/180)))/(2*9.81) + a2 - Cd;
Serial.print("\ta1="); // debugging tool
Serial.print(a1, 4);
Serial.print("meters");
Serial.print("\ta2=");
Serial.print(a2, 4);
Serial.print("meters");
Serial.print("\tvel=");
Serial.print(vel, 4);
Serial.print("m/s");
Serial.print("\tthreshold= ");
Serial.print(threshold);
Serial.print("m/s\n");
//Overshooting
if (Hp >= TargetHeight) { 
   Serial.print("\nOvershooting Target Height, opening flaps");
      if(( digitalRead(6) == HIGH ) && ( Hp >= TargetHeight )){
         Serial.print("\nOvershooting Target Height, opening flaps");
         esc.write(0);
         delay(1);
//Get Sensor Data//
  sensors_event_t event; 
  bno.getEvent(&event);
a1 = baro.getAltitude(); //altitude check 1
delay(100);
a2 = baro.getAltitude(); //altitude check 2
vel = ((abs(a2)-abs(a1))/(.1)); //delta altitude over time = vertical vel
Hp = ((vel*vel)*(sin(event.orientation.x)*(3.1415/180)))/(2*9.81) + a2 - Cd;
Serial.print("\ta1="); // debugging tool
Serial.print(a1, 4);
Serial.print("meters");
Serial.print("\ta2=");
Serial.print(a2, 4);
Serial.print("meters");
Serial.print("\tvel=");
Serial.print(vel, 4);
Serial.print("m/s");
Serial.print("\tthreshold= ");
Serial.print(threshold);
Serial.print("m/s\n");
}        
         //stop opening when they are open  
         else if(digitalRead(6) == (LOW)) {
         esc.write(90);
         Serial.print("\nFlaps Opened");
                        }
         }
//UnderShooting// 
else if(Hp <= TargetHeight) {
      Serial.print("\nUndershooting, Closing Flaps");
      if((digitalRead(7) == HIGH) && (Hp <= TargetHeight)) {
      esc.write(180);
      delay(1);
//Get Sensor Data//
 sensors_event_t event; 
 bno.getEvent(&event);
a1 = baro.getAltitude(); //altitude check 1
delay(100);
a2 = baro.getAltitude(); //altitude check 2
vel = ((abs(a2)-abs(a1))/(.1)); //delta altitude over time = vertical vel
Hp = ((vel*vel)*(sin(event.orientation.x)*(3.1415/180)))/(2*9.81) + a2 - Cd;
Serial.print("\ta1="); // debugging tool
Serial.print(a1, 4);
Serial.print("meters");
Serial.print("\ta2=");
Serial.print(a2, 4);
Serial.print("meters");
Serial.print("\tvel=");
Serial.print(vel, 4);
Serial.print("m/s");
Serial.print("\tthreshold= ");
Serial.print(threshold);
Serial.print("m/s\n");
      }
     else if(digitalRead(7) == (LOW)) {
     esc.write(90); delay(1); Serial.print("\nFlaps closed");
                                      }
}}}}}}       
