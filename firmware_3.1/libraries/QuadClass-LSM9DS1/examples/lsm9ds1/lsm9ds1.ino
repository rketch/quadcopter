#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


void setupSensor()
{
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_952 | G_BW_G_10 );  //952hz ODR + 63Hz cuttof

  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_9);
  
  // enable mag continuous (Section 8.7)
  lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000); // continuous mode

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
  lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_245DPS);
}


void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
}

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
  delay(200);
}
