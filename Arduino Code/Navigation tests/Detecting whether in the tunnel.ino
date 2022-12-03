
#include <VitconSAA1064T.h>
#include <VitconGP2Y0A21YK.h>

using namespace vitcon;

SAA1064T fnd;
GP2Y0A21YK sensor(A0);

void setup()
{
  SAA1064T::Init();
  Serial.begin(9600);
}

void loop()
{
  float distance = sensor.GetDistance();
  Serial.println(distance);
  delay(500);
  if distance < 50 {
    //CARRY OUT THE CASE OF BEING IN THE TUNNEL
  }
}

