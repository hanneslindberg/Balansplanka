
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <Servo.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double setpoint = 0; // Önskad lutning (grader)
double Kp = 10;      // Proportionell konstant
double Ki = 0.1;     // Integral konstant
double Kd = 1;       // Derivativ konstant

double error = 0;
double last_error = 0;
double integral = 0;
double angle = 0;

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your connections.");
    while (1);
  }

  servoX.attach(3);
  servoY.attach(9);
}

void loop() {
  updateAngle();
}

void updateAngle() {
  sensors_event_t event;
  accel.getEvent(&event);

  int angle_y = atan2(event.acceleration.x, event.acceleration.z) * (180 / M_PI);
  int angle_x = atan2(event.acceleration.y, event.acceleration.z) * (180 / M_PI);

  Serial.print(String(map(angle_x, -90, 90, 0, 180)) + "      ");
  Serial.println(map(angle_y, -90, 90, 0, 180));

  servoX.write(map(angle_x, -90, 90, 0, 180));
  servoY.write(map(angle_y, -90, 90, 0, 180));
  delay(10);
}
