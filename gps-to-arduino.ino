/*
  SoftwareSerial Echo
  software-serial-echo.ino
  Echos data from SoftwareSerial
  Use to display GPS raw data
  Results on serial monitor
 
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/
 
 // Define SoftwareSerial Connection  
#define swsTX 3 // Transmit FROM GPS
#define swsRX 4 // Receive TO GPS
 
//GPS Baud rate
#define GPSBaud 9600
 
//Serial Monitor Baud Rate
#define Serial_Monitor_Baud 9600  
 
// Include and set up the SoftwareSerial Library
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
SoftwareSerial GPSserial(swsRX, swsTX);  

// Create a TinyGPS++ object
TinyGPSPlus gps;

String gpsdata;
 
void setup()
{
 //Start Serial Monitor
 Serial.begin(Serial_Monitor_Baud);
 
 // Start SoftwareSerial  
 GPSserial.begin(GPSBaud);
}
   
void loop()
{
  // Write SoftwareSerial data to Serial Monitor
  while (GPSserial.available() > 0)
  {
    // Check Serial data from TX pin. For viewing NEMA sentences 
    //char data = GPSserial.read();
    //Serial.write(data);
    if (gps.encode(GPSserial.read()))
    {
      

      // If a new sentence is successfully parsed, print the data to the serial monitor
      if (gps.location.isValid() && gps.location.isUpdated())
      {
        // Data Logging
        //Serial.println("-------------------");
        //Serial.print("Timestamp: ");
        //Serial.println(gps.time.value());
        //Serial.print("Satellites: ");
        //Serial.println(gps.satellites.value());
        //Serial.print("Latitude: ");
        //Serial.println(gps.location.lat(), 6);
        //Serial.print("Longitude: ");
        //Serial.println(gps.location.lng(), 6);

        // Print to Serial for comm
        gpsdata = String(gps.location.lat(), 6);
        gpsdata += (",");
        gpsdata += String(gps.location.lng(), 6);
        gpsdata += (",");
        gpsdata += String(gps.time.value());
        gpsdata += (",");
        gpsdata += String(gps.satellites.value());
        gpsdata += (",");
        gpsdata += String(gps.hdop.value());
        Serial.println(gpsdata);
      }
      //Serial.println("Invalid GPS or location not updated");
    }
  }
  //Serial.println("GPS Serial not available");
}