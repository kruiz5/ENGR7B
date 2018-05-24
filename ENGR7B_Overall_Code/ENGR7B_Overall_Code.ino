#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>  //get library for IMU
#include "Ultrasonic.h"  //get library for Ultransoic sensor

//create imu object and connect sensors
LSM9DS1 imu; 
Ultrasonic X_sensor (12,13); //Trig,Echo
Ultrasonic Y_sensor (8,9);
Ultrasonic Z_sensor (2,3);

#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.56 // Declination (degrees) in Irvine, CA.

boolean straight; //boolean to determine if straight enough to read sensors
float roll, pitch; //where values of roll and pitch will be stores
float x, y, z; //where sensor readings will be stored
int quadrant, color; //where values of quadrant and color will be stored

void setup() 
{
 Serial.begin(9600);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop() 
{
  updateIMU();  //updates roll and pitch readings
  checkIfStraight();  //check if roll and pitch are within 5 degrees of tilt
  if (straight == true)  //do the following if quadcopter is not tilting
  {
    read_XY_sensors();  //get readings from X and Y sensors
    returnQuadrant();   //determine quadrant based on X and Y sensor readings
    read_Z_sensor();   // get reading for Z sensor
    returnColor();    //determine color value based on Z sensor reading
    char a = 'E'; 
    Serial.write(a);  //transmitting new signal to XBEE
    //send qudcopter number as two bytes
    Serial.write(quadrant/256); 
    Serial.write(quadrant%256);
    //send color number as two bytes
    Serial.write(color/256);
    Serial.write(color%256);
    delay(1000);
  }
}

void updateIMU()
{
   // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(imu.ax, imu.ay, imu.az, 
                 -imu.my, -imu.mx, imu.mz);
    Serial.println();
    
    lastPrint = millis(); // Update lastPrint time
  } 

}
void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  pitch = atan2(ay, az);
  roll = atan2(-ax, sqrt(ay * ay + az * az));
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); 
  Serial.println(heading, 2);

  if (roll > 8) //see if tilting right
  {
    Serial.println("tilting right");
  }

  if (roll < -2) //see if tilting left
  {
    Serial.println("tilting left");
  }

  if (pitch > 0) //see if tilting forward
  {
    Serial.println("tilting forward");
  }
  if (pitch < -10) //see if tilting backwards
  {
    Serial.println("tilting backward");
  }
}

void checkIfStraight() 
{
    //checks if tilting too much
    if (-2 < roll && roll < 8 && -10 < pitch && pitch < 0)  
    {
      straight = true;
      Serial.println("straight");
    }
    else 
      straight = false;
}

void read_XY_sensors()
{
  x = X_sensor.Ranging(INC)/12.0; //convert X reading to ft
  Serial.print("X = ");
  Serial.print(x);
  Serial.println(" ft");
  delay(100);
  y = Y_sensor.Ranging(INC)/12.0;  //convert Y reading to ft
  Serial.print("Y = ");
  Serial.print(y);
  Serial.println(" ft");
  delay(100);
}

void read_Z_sensor()
{
  z = Z_sensor.Ranging(INC)/12.0;  //convert Z reading to ft
  Serial.print("Z = ");
  Serial.print(z);
  Serial.println(" ft");
  delay(100);
}

void returnQuadrant()
{
  //if 5<x<10 and 0<y<5,in Q1
  if (5 < x && x < 10 && 0 < y && y < 5) 
  {
    Serial.println("Q1");
    quadrant = 1;
  }
   //if 0<x<5 and 0<y<5,in Q2
  else if (0 < x && x < 5 && 0 < y && y < 5)
  {
    Serial.println("Q2");
   quadrant = 2;
    
  }
  //if 0<x<5 and 5<y<10,in Q3
  else if (0 < x && x < 5 && 5 < y && y < 10)  
  {
    Serial.println("Q3");
    quadrant = 3;
    
  }
   //if 5<x<10 and 5<y<10,in Q4
  else if (5 < x && x < 10 && 5 < y && y < 10) 
  {
    Serial.println("Q4");
    quadrant = 4;
    
  }
  //if none of the above, no quadrant
  else
  {
    Serial.println("no Quadrant");
    return 0;
  }
}

void returnColor()
{
  if (6.5 < z && z < 8) //if distance between "object" and quadcopter is 6.5 to 8ft
  {
    Serial.println("byte, height color: ");
    Serial.println("1,white");
    color = 1; //no object
  }
  if (5 < z && z < 6.5) //if distance between "object" and quadcopter is 5 to 6.5ft
  {
    Serial.print("byte, height color: ");
    Serial.println("2,red");
    color = 2;  //largest object
  }
  if (3.5 < z && z < 5) //if distance between "object" and quadcopter is 3.5 to 5ft
  {
    Serial.print("byte, height color: ");
    Serial.println("3,green");
    color = 3; //middle object
  }
  if (2 < z && z < 3.5) //if distance between "object" and quadcopter is 2 to 3.5ft
  {
    Serial.print("byte, height color: ");
    Serial.println("4,blue");
    color = 4; //smallest object
  }
}
