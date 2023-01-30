//#include <Adafruit_BMP085_U.h>

//#include <Adafruit_BMP085_U.h>

#include "DHT.h"
#define DHTPIN 2    
#define DHTTYPE DHT11   // DHT 11
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#define DEVICE (0x53) //ADXL345 device address
#define TO_READ (6) //num of bytes we are going to read each time (two bytes for each axis)
 
#define offsetX -10.5 // place your OFFSET values here
#define offsetY -2.5
#define offsetZ -4.5
 
#define gainX 257.5 // place your GAIN factors
#define gainY 254.5
#define gainZ 248.5


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHTPIN, DHTTYPE);

byte buff[TO_READ] ; //6 bytes buffer for saving data read from the device
char str[512]; //string buffer to transform data before sending it to the serial port

//pulse sensor
//  Variables
int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED13 = 13;   //  The on-board Arduion LED


int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.


void setup() 
{
  Serial.begin(9600);
  //bt.begin(9600); /* Define baud rate for software serial communication */
  Serial.println(F("DHTxx test!"));
  dht.begin();

  Serial.println("Arduino MLX90614 Testing");  
  mlx.begin();  

  Wire.begin(); // join i2c bus (address optional for master)

  //Turning on the ADXL345
writeTo(DEVICE, 0x2D, 0);
writeTo(DEVICE, 0x2D, 16);
writeTo(DEVICE, 0x2D, 8);

//pulse sensor

pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
   Serial.begin(9600);         // Set's up Serial Communication at certain speed.
}

void loop() {
  delay(2000);

  
  float h = dht.readHumidity();
//   Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

//   Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
 Serial.print(F("°F  Heat index: "));
Serial.print(hic);
 Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
  delay(500);


 // __________ himidity + tempwerature_________________//

  Serial.print("Ambient = "); 
  Serial.print(mlx.readAmbientTempC()); 
  Serial.print("*C\tObject = "); 
  Serial.print(mlx.readObjectTempC()); 
  Serial.println("*C");
  Serial.print("Ambient = "); 
  Serial.print(mlx.readAmbientTempF()); 
  Serial.print("*F\tObject = "); 
  Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  Serial.println();
  delay(500);

  //_________________x______________//ADXL

  int regAddress = 0x32; //first axis-acceleration-data register on the ADXL345
int x, y, z;
 
readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
 
//each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
//thus we are converting both bytes in to one int
x = (((int)buff[1]) << 8) | buff[0];
y = (((int)buff[3])<< 8) | buff[2];
z = (((int)buff[5]) << 8) | buff[4];
 
//we send the x y z values as a string to the serial port
sprintf(str, "%d %d %d", x, y, z);
Serial.print(str);
Serial.print(10, byte());
//It appears that delay is needed in order not to clog the port
delay(100);
  
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
Wire.beginTransmission(device); //start transmission to device
Wire.write(address); // send register address
Wire.write(val); // send value to write
Wire.endTransmission(); //end transmission
}
 
//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
Wire.beginTransmission(device); //start transmission to device
Wire.write(address); //sends address to read from
Wire.endTransmission(); //end transmission
 
Wire.beginTransmission(device); //start transmission to device
Wire.requestFrom(device, num); // request 6 bytes from device
 
int i = 0;
while(Wire.available()) //device may send less than requested (abnormal)
{
buff[i] = Wire.read(); // receive a byte
i++;
}
Wire.endTransmission(); //end transmission

//pulse sensor

Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
                                              // Assign this value to the "Signal" variable.

   Serial.println(Signal);                    // Send the Signal value to Serial Plotter.


   if(Signal > Threshold){                          // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
     digitalWrite(LED13,HIGH);
   } else {
     digitalWrite(LED13,LOW);                //  Else, the sigal must be below "550", so "turn-off" this LED.
   }


delay(10);


}



 
