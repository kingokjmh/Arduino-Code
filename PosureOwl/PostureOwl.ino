/*Posture Owl - A werable training lower body sitting posutre device.
  Consist of IR distance sensor, vibrator, digital vibration sensor & three-axis accelerometer
  Author: Minghui Ju
  Date: 12-09-2020
*/


#include <Wire.h>
byte Version[3];
int8_t x_data;
int8_t y_data;
int8_t z_data;
byte range = 0x00;
float divi = 16;
float x, y, z;

int LED = 12;
const int vibrationPin = 5;
const int sensorPin = A0;
const int sensorVib = 3;

unsigned char state = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  pinMode(sensorVib, INPUT);
  pinMode(vibrationPin, OUTPUT);
  pinMode(LED, OUTPUT);
  attachInterrupt(1, blink, FALLING);

  Wire.begin();
  Wire.beginTransmission(0x0A); // address of the accelerometer
  // range settings
  Wire.write(0x22); //register address
  Wire.write(range); //can be set at"0x00""0x01""0x02""0x03", refer to Datashhet on wiki
  // low pass filter
  Wire.write(0x20); //register address
  Wire.write(0x05); //can be set at"0x05""0x04"......"0x01""0x00", refer to Datashhet on wiki
  Wire.endTransmission();
}

void AccelerometerInit()
{
  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x04); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A, 1);   // request 6 bytes from slave device #2
  while (Wire.available())   // slave may send less than requested
  {
    Version[0] = Wire.read(); // receive a byte as characte
  }
  x_data = (int8_t)Version[0] >> 2;

  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x06); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A, 1);   // request 6 bytes from slave device #2
  while (Wire.available())   // slave may send less than requested
  {
    Version[1] = Wire.read(); // receive a byte as characte
  }
  y_data = (int8_t)Version[1] >> 2;

  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x08); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A, 1);   // request 6 bytes from slave device #2
  while (Wire.available())   // slave may send less than requested
  {
    Version[2] = Wire.read(); // receive a byte as characte
  }
  z_data = (int8_t)Version[2] >> 2;

  x = (float)x_data / divi;
  y = (float)y_data / divi;
  z = (float)z_data / divi;
  Serial.print("X=");
  Serial.print(x);         // print the character
  Serial.print("  ");
  Serial.print("Y=");
  Serial.print(y);         // print the character
  Serial.print("  ");
  Serial.print("Z=");           // print the character
  Serial.println(z);
}

void loop()
{
  //switch(range)  //change the data dealing method based on the range u've set
  // {
  //  case 0x00:divi=16;  break;
  // case 0x01:divi=8;  break;
  // case 0x02:divi=4;  break;
  // case 0x03:divi=2;  break;
  // default: Serial.println("range setting is Wrong,range:from 0to 3.Please check!");while(1);
  //  }

  AccelerometerInit();
  if (z > 0.2 | z < -0.2) {
    analogWrite(vibrationPin, 180); digitalWrite(LED, HIGH); //PWM
    delay(200);
    analogWrite(vibrationPin, 0);   //PWM
    delay(200);
    analogWrite(vibrationPin, 180);   //PWM
    delay(100);
    analogWrite(vibrationPin, 0);   //PWM
    delay(200);
    analogWrite(vibrationPin, 180);   //PWM
    delay(100);
    analogWrite(vibrationPin, 0);   //PWM
    delay(200);


    Serial.println("Tilt detected");
    delay(1000);
  } else {
    analogWrite(vibrationPin, 0);
    digitalWrite(LED, LOW);
  }

  uint16_t value = analogRead (sensorPin);
  uint16_t range = get_gp2d12 (value);
  //  Serial.println(value);
  Serial.print("Distance:");
  Serial.print(range);
  Serial.println("mm");
  Serial.println();
  delay(500);
  if (range > 280) {
    Serial.println("Too wide +++++");
    analogWrite(vibrationPin, 130); digitalWrite(LED, HIGH);
    delay(500);
    analogWrite(vibrationPin, 0);
    delay(300);
    analogWrite(vibrationPin, 130);
    delay(500);
    digitalWrite(LED, HIGH);
    delay(1300);
  } else {
    analogWrite(vibrationPin, 0);
    digitalWrite(LED, LOW);

  }

  if (state != 0)
  {
    state = 0;
    analogWrite(vibrationPin, 250); digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, HIGH);
    delay(1000);
    Serial.println("Jiggling detected ++++++");
    //delay(500);
  }
  else
    analogWrite(vibrationPin, 0);
  digitalWrite(LED, LOW);

  //delay(500);

}
void blink() {
  state++;
}


uint16_t get_gp2d12 (uint16_t value) {
  if (value < 30)
    value = 30;
  return ((67870.0 / (value - 3.0)) - 40.0);
}
