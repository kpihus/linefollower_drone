#include <Wire.h>

#define GYRO 0x68
#define ACCEL 0x40


#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z

int g_offx = 120;
int g_offy = 20;
int g_offz = 93;
int hx, hy, hz, turetemp;



void initGyro()
{
  /*****************************************
   * ITG 3200
   * power management set to:
   * clock select = internal oscillator
   * no reset, no sleep mode
   * no standby mode
   * sample rate to = 125Hz
   * parameter to +/- 2000 degrees/sec
   * low pass filter = 5Hz
   * no interrupt
   ******************************************/
  writeTo(GYRO, G_PWR_MGM, 0x00);
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(GYRO, G_INT_CFG, 0x00);
}

void AccelerometerInit()
{
  Wire.beginTransmission(ACCEL); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x10);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10); 

  Wire.beginTransmission(ACCEL); // address of the accelerometer
  // low pass filter, range settings
  Wire.write(0x0D);
  Wire.write(0x10);
  Wire.endTransmission(); 

  Wire.beginTransmission(ACCEL); // address of the accelerometer
  Wire.write(0x20); // read from here
  Wire.endTransmission();
  Wire.requestFrom(0x40, 1);
  byte data = Wire.read();
  Wire.beginTransmission(ACCEL); // address of the accelerometer
  Wire.write(0x20);
  Wire.write(data & 0x0F); // low pass filter to 10 Hz
  Wire.endTransmission(); 

  Wire.beginTransmission(ACCEL); // address of the accelerometer
  Wire.write(0x35); // read from here
  Wire.endTransmission();
  Wire.requestFrom(0x40, 1);
  data = Wire.read();
  Wire.beginTransmission(ACCEL); // address of the accelerometer
  Wire.write(0x35);
  Wire.write((data & 0xF1) | 0x04); // range +/- 2g
  Wire.endTransmission();
} 

void getGyroscopeData(int * result)
{
  /**************************************
   * Gyro ITG-3200 I2C
   * registers:
   * temp MSB = 1B, temp LSB = 1C
   * x axis MSB = 1D, x axis LSB = 1E
   * y axis MSB = 1F, y axis LSB = 20
   * z axis MSB = 21, z axis LSB = 22
   *************************************/
  int regAddress = 0x1B;
  int temp, x, y, z;
  byte buff[G_TO_READ];
  readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
  result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
  result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  result[3] = (buff[0] << 8) | buff[1]; // temperature
}

void AccelerometerRead()
{
  Wire.beginTransmission(0x40); // address of the accelerometer
  Wire.write(0x02); // set read pointer to data
  Wire.endTransmission();
  Wire.requestFrom(0x40, 6); 

  // read in the 3 axis data, each one is 16 bits
  // print the data to terminal
  Serial.print("Accelerometer: X = ");
  short data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.print(" , Y = ");
  data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.print(" , Z = ");
  data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.println();
} 



void setup(){
  Wire.begin();
  initGyro();
  AccelerometerInit();
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor

}

void loop(){
  byte addr;
  int gyro[4];
  getGyroscopeData(gyro);
  hx = gyro[0] / 14.375;
  hy = gyro[1] / 14.375;
  hz = gyro[2] / 14.375;
  turetemp = 35+ ((double) (gyro[3] + 13200)) / 280; // temperature

  AccelerometerRead();




  delay(50);           // wait 5 seconds for next scan

}

//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}
//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC

  int i = 0;
  while(Wire.available())    //ACC may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}


