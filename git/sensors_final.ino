#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PULSE_PIN 34 // ADC pin for KG011 Pulse Monitor

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("Error initializing MLX90614");
    while (1);
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Error initializing MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Setup Pulse Monitor
  pinMode(PULSE_PIN, INPUT);
}

void loop() {
  // Pulse Monitor Reading
  int pulseValue = analogRead(PULSE_PIN);

  // MLX90614 Readings
  float ambientTemp = mlx.readAmbientTempC();
  float objectTemp = mlx.readObjectTempC();

  // MPU6050 Readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Output for Serial Plotter (separated by tabs or commas)
  Serial.print(pulseValue);                // Pulse value
  Serial.print("\t");                       // Tab separator
  Serial.print(ambientTemp);               // Ambient temperature
  Serial.print("\t");
  Serial.print(objectTemp);                // Object temperature
  Serial.print("\t");
  Serial.print(accel.acceleration.x);      // Accelerometer X
  Serial.print("\t");
  Serial.print(accel.acceleration.y);      // Accelerometer Y
  Serial.print("\t");
  Serial.print(accel.acceleration.z);      // Accelerometer Z
  Serial.print("\t");
  Serial.print(gyro.gyro.x);               // Gyroscope X
  Serial.print("\t");
  Serial.print(gyro.gyro.y);               // Gyroscope Y
  Serial.print("\t");
  Serial.println(gyro.gyro.z);             // Gyroscope Z (newline for next frame)

  delay(50); // Reduce delay for smoother plotting
}
