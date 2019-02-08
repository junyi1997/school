#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;
int ax, ay, az;
int gx, gy, gz;

void setup() {
    Wire.begin();
    Serial.begin(38400);
    accelgyro.initialize();   
}
void loop() {
    digitalWrite(2,LOW); 
    digitalWrite(3,HIGH);   
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        delay(500);
        Serial.print("a/g:");Serial.print("\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
}
