#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_MCP23X17.h>
#include <MS5611.h>
#include <Adafruit_MPL3115A2.h>
#include "ICM42688.h"
#include <SparkFun_KX13X.h>
#include <Math.h>

//The following states are possible:
// 0000 0000 (0) [0x00]: Pad Idle
// 0000 0001 (1) [0x01]: Main deploy setpoint set
// 0000 0010 (2) [0x02]: Continuity positive
// 0000 0011 (3) [0x03]: Armed
// 0000 0100 (4) [0x04]: Liftoff detected
// 0000 0101 (5) [0x05]: Booster Burnout/Coasting
// 0000 0110 (6) [0x06]: Apogee detected
// 0000 0111 (7) [0x07]: Pyro 1 signal sent (drogue)
// 0000 1000 (8) [0x08]: Pyro2 signal sent (drogue)
// 0000 1001 (9) [0x09]: Drogue deplyoment detected
// 0000 1010 (10) [0x0A]: Pyro3 signal sent (main
// 0000 1011 (11) [0x0B]: Pryo4 signal sent (main)
// 0000 1100 (12) [0x0C]: Main deployment detected
// 0000 1101 (13) [0x0D]: Landed
// 0000 1110 (14) [0x0E]: ABORT - Failed to initialize
// 0000 1111 (15) [0x0F]: ABORT - No continuity

//The following commands are possible:
// a: 50 m
// b: 100 m
// c: 150 m
// d: 200 m
// f: arming
// g: arming zurücknehmen 
// q: drogue ejection 
// r: main ejection
// called in ReceiveFunc() function

//Code includes test setup and flight code by using following defines to enable/disable code: "TEST" "FLIGHT" 
#define FLIGHT
#define TIMERSETPOINT 0 //to be added with timer set point from simulations (in ms)

TwoWire MASTER = TwoWire(0);
TwoWire SLAVE = TwoWire(1);
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(0x10);
Adafruit_MCP23X17 mcp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
MS5611 MS(0x77);
Adafruit_MPL3115A2 baro;
ICM42688 IMU(Wire, 0x68);
SparkFun_KX134 kxAccel; //Declares type of KX13x Accelerometer used
HardwareSerial Serial1(1); // Für die Ausgabe am Serial Monitor
HardwareSerial Serial2(2); // 1 = USB Serial, wir können also 2 oder 3 wählen

outputData kxData; //Struct for accelerometer's data

char arr_tel[2]; //global status array
char status = 0; //global status byte
char cmd = 0; //global command byte
float mainSetPoint = 0; 
float p0 = 0; //in Pa
float alt = 0;
const int contPIN = 1;
unsigned long start = 0;


int SensorInit() {
  #ifdef FLIGHT

    int initSensors = 0;

    if(mcp.begin_I2C())
    initSensors++;
    delay(100);

    if(accel.begin())
    initSensors++;
    delay(100);

    if(baro.begin());
    initSensors++;
    delay(100);

    if(MS.begin())
    initSensors++;
    delay(100);

    if(IMU.begin())
    initSensors++;
    delay(100);

    if(kxAccel.begin())
    initSensors++;
    delay(100);

    if(initSensors == 6)
      return 1;
    else
      return 0;
  #endif

  #ifdef TEST

    //Initialize MCP (GPIO extender)
    if (!mcp.begin()) 
      Serial.println("MCP23017 init failed!");
    else
      Serial.println("MCP23017 initialized.");
    delay(100);

    //initalize ADXL375 (Accelerometer)
    if(!accel.begin())
      Serial.println("AXL375 init failed!");
    else
      Serial.println("ADXL375 initialized.");
    delay(100);

    //Initialize MPL
    if(!baro.begin()) 
      Serial.println("MPL init failed!");
    else
     Serial.println("MPL initialized.");
    delay(100);

    //Initialize MS5611
    if(!MS.begin())
      Serial.println("MS5611 init failed!");
    else
      Serial.println("MS4511 initialized.");
    delay(100);

    //Initialize ICM42688
    if(!IMU.begin())
      Serial.println("ICMinit failed!");
    else
      Serial.println("ICM42688 initizialized");
    delay(100);

    //Initialize KX134
    if (!kxAccel.begin())
      Serial.println("KX134 init failed!");
    else
      Serial.println("KX134 initializied");
    delay(100);

    return 1;

  #endif
}

void RequestFunc() {
  arr_tel[0] = 0; //to be used for LED

  arr_tel[1] = status;
  for(int i = 0; i < 2; i++)
    SLAVE.write(arr_tel[i]);
}

void cmdFunc() {
  for(int i = 0; i>1; i++)
    char cmd = SLAVE.read();
}

void setup() {
  bool sensorInit = 0; //boolean to show if all sensors have been initialized
  unsigned long timerSetPoint = TIMERSETPOINT; //in ms 
  int contThresh = 2480; //equivalent to approx. 2V -> ideally it should be 3V3

  MASTER.setPins(9, 8);
  MASTER.begin();

  SLAVE.setPins(10, 16);
  SLAVE.begin(0x40);
  SLAVE.onRequest(RequestFunc);

  Serial1.begin(9600, SERIAL_8N1, 44, 43);
  Serial2.begin(9600, SERIAL_8N1, 4, 5);

  sensorInit = SensorInit();

  

  pinMode(contPIN, INPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);

  float press[20] = {0};

  for(int i = 0; i<10; i++) { //read pressure sensors 10 times
    MS.read();

    press[i] = MS.getPressurePascal();

    press[i+10] = baro.getPressure()/100; //convert from hPa to Pa

    int p0 = (press[i] + press[i+10])/10; //get avg of first 10 measurement for ground level pressure
  }



  #ifdef TEST

    for(int i = 0; i>10; i++) {
      status = 0x0E;
  
      sensorInit = SensorInit(); //try to reinitialize sensors -> might want to be deleted because if there is issues we should investigate

      //Run I2C scanner
      byte error, address;
      int nDevices;

      Serial1.println("Scanning...");

      nDevices = 0;
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        MASTER.beginTransmission(address);
        error = MASTER.endTransmission();

        if (error == 0)
        {
        Serial1.print("I2C device found at address 0x");
          if (address<16)
            Serial1.print("0");
          Serial1.print(address,HEX);
          Serial1.println("  !");

          nDevices++;
        }
        else if (error==4)
        {
          Serial1.print("Unknown error at address 0x");
          if (address<16)
            Serial1.print("0");
          Serial1.println(address,HEX);
        }
      }
      if (nDevices == 0)
        Serial1.println("No I2C devices found\n");
      else
        Serial1.println("done\n");

      delay(5000);           // wait 5 seconds for next scan
    }

  #endif

  #ifdef FLIGHT

    while(sensorInit == 0) {  //stay in loop until every sensor is initialized
      status = 0x0E;
  
      sensorInit = SensorInit(); //try to reinitialize sensors -> might want to be deleted because if there is issues we should investigate

    //Run I2C scanner
      byte error, address;
      int nDevices;

      Serial0.println("Scanning...");

      nDevices = 0;
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        MASTER.beginTransmission(address);
        error = MASTER.endTransmission();

        if (error == 0)
        {
        Serial1.print("I2C device found at address 0x");
          if (address<16)
            Serial1.print("0");
          Serial1.print(address,HEX);
          Serial1.println("  !");

          nDevices++;
        }
        else if (error==4)
        {
          Serial1.print("Unknown error at address 0x");
          if (address<16)
            Serial1.print("0");
          Serial1.println(address,HEX);
        }
      }
      if (nDevices == 0)
        Serial1.println("No I2C devices found\n");
      else
        Serial1.println("done\n");

      delay(5000);           // wait 5 seconds for next scan
    }

   while(status < 0x01) { //should only trigger if main set point has not already been set
      SLAVE.onReceive(cmdFunc);
      switch(cmd) {
        case 'a': mainSetPoint = 50; break; 
        case 'b': mainSetPoint = 100; break; 
        case 'c': mainSetPoint = 150; break; //not necessary but fuck it, redundant code
        case 'd': mainSetPoint = 200; break;
        default: mainSetPoint == 150; break;
      }
      if(mainSetPoint == 50 || mainSetPoint == 100 || mainSetPoint == 150 || mainSetPoint == 200) {
        status = 0x01;
      }
    }
  #endif
}

void Pre_Flight_Loop () {
  while(1) {
    SLAVE.onRequest(RequestFunc);

    float alt2 = 0; //moving avg variable
    float contThresh = 2480; //should be approx. 2V on 3V3 system
    float velX = 0;
    float velY = 0;
    float velZ = 0;
    float velZ_abs = 0;
    float vel = 0;
    float mpl_alt = 0;
    float ms_alt = 0;
    float sens_arr[3] = {0}; //Data packet received from sensorics -> format: arr[0] = GNSS altitude; arr[1] = GNSS vel; arr[2] = RotZ
    float raw_data[36] = {0}; //Data full of raw data
    float data[20] = {0}; //Data to be sent to sensorics for logging
    float conv[10] = {0}; //To copy into for processing of data
    float alt1[20] = {0};
    
  

    float contData = analogRead(contPIN);
    while(contData < contThresh) {  //stays in loop until continuity is detected
      status = 0x0F; 
      contData = analogRead(contPIN);
    }
    
    status = 0x02;

    SLAVE.onReceive(cmdFunc);

    if(cmd = 'f' && status == 0x02)
      status = 0x03; //confirms "armed" status

    for(int k=0; k<10; k++) { //iterate over 10 altitude points
      

    //Gather data
      unsigned long t2 = millis(); //get time before each sampling


      while(Serial2.available()) { //read three floats at a time
        for(int i=0; i<2; i++) {
          sens_arr[i] = Serial2.read();
          raw_data[i+12]; //append received data to raw data array
        }
      }

      sensors_event_t event; //Read ADXL375
      accel.getEvent(&event);
      raw_data[0] = event.acceleration.x;
      raw_data[1] = event.acceleration.y;
      raw_data[2] = event.acceleration.z;

      kxAccel.getAccelData(&kxData); //Read KX134
      raw_data[3] = kxData.xData;
      raw_data[4] = kxData.yData;
      raw_data[5] = kxData.zData;
      
      IMU.getAGT(); //Read ICM
      raw_data[6] = IMU.accX();
      raw_data[7] = IMU.accY();
      raw_data[8] = IMU.accZ();
      raw_data[9] = IMU.gyrZ(); //z-Axis should be parallel to rocket vel. vector

      raw_data[10] = baro.getPressure()/100;

      MS.read(); 
      raw_data[11] = MS.getPressurePascal();


    //Gather data again for integration (very ugly will be changed in the future)
      unsigned long t1 = millis(); //get time before each sampling

      accel.getEvent(&event);
      raw_data[0+15] = event.acceleration.x;
      raw_data[1+15] = event.acceleration.y;
      raw_data[2+15] = event.acceleration.z;

      kxAccel.getAccelData(&kxData); //Read KX134
      raw_data[3+15] = kxData.xData;
      raw_data[4+15] = kxData.yData;
      raw_data[5+15] = kxData.zData;
      
      IMU.getAGT(); //Read ICM
      raw_data[6+15] = IMU.accX();
      raw_data[7+15] = IMU.accY();
      raw_data[8+15] = IMU.accZ();
      raw_data[9+15] = IMU.gyrZ(); //z-Axis should be parallel to rocket vel. vector


      unsigned long deltaT = t2 - t1;

      for(int i=0; i<10; i++) {  //integrate using trapezoidal rule which states: Integral += (sample1 + sample2) * 0.5 * deltaT
        conv[i] += (raw_data[i] + raw_data[i+15])* 0.5 * deltaT; 
        data[17] = conv[9];
      } 
      velX = (conv[0] + conv[3] + conv[6])/3;
      velY = (conv[1] + conv[4] + conv[7])/3;
      velZ = (conv[2] + conv[5] + conv[8])/3;

      data[16] = velX;
      data[17] = velY;
      data[18] = velZ;

      vel = sqrt(velX*velX + velY*velY + velZ*velZ);

      velZ_abs = vel * sin(conv[9]); //Is angle in degrees or radiants? -> CHECK!!!!!!!


      alt1[k] += velZ_abs * deltaT; //integrates velocity to altitude

      mpl_alt = 288.15/0.0065 * (1 - pow(raw_data[10]/p0, 1/5.255)); //altitude according to barometric height (using standard atmosphere temps -> maybe should be changed)
      ms_alt =  288.15/0.0065 * (1 - pow(raw_data[11]/p0, 1/5.255));
      
      alt1[k+10] = (mpl_alt * ms_alt)/2;

      alt += alt1[k] + alt1[k+10]; //variable to have sum of all altitudes

      alt2 = alt/20; //average of last ten iterations of height (20 altitude datapoints -> 10 from accel, 10 from pressure)

      data[21] = (raw_data[10] + raw_data[11])/2;
      data[22] = alt2;
    }

    for(int i=0; i<12; i++) {
      data[i] = raw_data[i]; //copy raw data to send array
    }


    data[13] = (raw_data[0] + raw_data[3] + raw_data[6])/3; //avg of AccelX
    data[14] = (raw_data[1] + raw_data[4] + raw_data[7])/3; //avg of AccelY
    data[15] = (raw_data[2] + raw_data[5] + raw_data[8])/3; //avg of AccelZ

    data[12] = status;


    for(int i=0; i<23; i++) 
      Serial2.write(data[i]);


    if(data[13] > 5) {//5 referring to 5 m/s^2
      status = 0x04;
      start = millis();
      break;
    }
  }
}

void In_Flight_Loop() {

  SLAVE.onReceive(cmdFunc);

  float raw_data[14] = {0};
  float data[8] = {0};
  float alt_sum = 0;
  float vel_sum = 0;
  float acc_sum = 0;
  int i = 0;
  for(int j = 0, j < 10; j++) {
    while (Serial2.available()) //fill raw_data array with received data
    {
    raw_data[i] = Serial2.read();
    i++;
    }

    unsigned long deltaT = raw_data[13] - raw_data[6];

    //get total angle to z-axis
      float z_angle =+ (raw_data[1] + raw_data[8]) * 0.5 * deltaT; //integrate using trapezoidal rule which states: Integral += (sample1 + sample2) * 0.5 * deltaT
    
    float z_acc1 = raw_data[0] * sin(z_angle); //get absolute z-acceleration
    float z_acc2 = raw_data[7] * sin(z_angle); 
    acc_sum =+ z_acc1 + z_acc2;
    
  
    float z_vel =+ (z_acc1 + z_acc2) * 0.5 * deltaT;
    vel_sum =+ z_vel; //sum of 10 different vel values for moving avg

    float alt1 = z_vel * deltaT;
    float alt2 = 288.15/0.0065 * (1 - pow(raw_data[2]/p0, 1/5.255)); //altitude according to barometric height (using standard atmosphere temps -> maybe should be changed)
    float alt3 = 288.15/0.0065 * (1 - pow(raw_data[3]/p0, 1/5.255));
    float alt4 = 288.15/0.0065 * (1 - pow(raw_data[4]/p0, 1/5.255));
    float alt5 = 288.15/0.0065 * (1 - pow(raw_data[5]/p0, 1/5.255));

    alt_sum =+ alt1 + alt2 + alt3+ alt4+ alt5; //sum of 10 different alt values for moving avg
  } 

  float alt =+ (alt_sum/50); //avg of 50 altitude datapoints
  float alt_old = alt;
  float vel = vel_sum/10;
  float acc = acc_sum/20;
  float acc_old = acc;

  if(acc < 0 && status == 0x04) //burnout detected?
    status = 0x05;
  
  unsigned long now = millis();
  boolean timer = false;
  boolean drogue = false;

  if(alt_old - alt < 0 && vel < 0 && status == 0x05) //if altitude difference is negative and velocity is negative
    status = 0x06;
  else if ((now-start) > TIMERSETPOINT)
  {
    timer = true;
  }
  else if(cmd = 'q')
    drogue = true;

  if(status == 0x06 || timer == true || drogue == true) {
    digitalWrite(41, HIGH);
    delay(300);
    digitalWrite(41, LOW);
    status = 0x08
  }

  if(alt < mainSetPoint && status == 0x08) {
    digitalWrite(42, HIGH);
    delay(300);
    digitalWrite(42, LOW);
    status = 0x10;
  }
   
  if((acc - acc_old) < 1 && (alt - alt_old) < 2){
    status = 0x13
    break;
  }
    
  
}

void loop() {
  
}

