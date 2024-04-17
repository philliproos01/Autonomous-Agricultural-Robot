#include <Arduino.h>
#include <WiFi.h>
#include <QMC5883LCompass.h>
#include <Adafruit_GPS.h>
#include <math.h>

#define GPSSerial Serial2
// change this to your own SSID and password
#define ssid "Alex"
#define pass "orangegreen"
#define server "pcr.bounceme.net"

//wheel parameters
const unsigned long SPEED_TIMEOUT = 500000;   // Time used to determine wheel is not spinning
const unsigned int UPDATE_TIME = 500;         // Time used to output serial data
const unsigned int BUFFER_SIZE = 16;          // Serial receive buffer size
const double BAUD_RATE = 115200;              // Serial port baud rate
const double WHEEL_DIAMETER_IN = 6.5;         // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 22.25;  // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 16.5;        // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 56.5;   // Motor wheel circumference (centimeters)

//left motor
uint16_t motorDirection1 = 33;
uint16_t motorDriver1 = 32;

//right motor
uint16_t motorDirection2 = 4;  //set to 4
uint16_t motorDriver2 = 26;

bool _dir = 0;
bool _dir2 = 1;

const int SPEED = 25;
const int PIN_SPEED = 14;
const int PIN_SPEED2 = 12;

uint8_t LED1pin = 18;
bool LED1status = LOW;

uint8_t LED2pin = 19;
bool LED2status = LOW;

bool forward_status = LOW;
bool backward_status = LOW;
bool left_status = LOW;
bool right_status = LOW;
bool spin_status = LOW;

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

struct coord {
  double latitude;
  double longitude;
};

QMC5883LCompass compass;

WiFiClient client;
coord latLongs[5] = {};

void setup() {
  Serial.begin(115200);
  //starting the GPS
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //starting the compass
  compass.init();
  delay(500);
  compass.setCalibrationOffsets(512.00, 1093.00, -658.00);  //Compass calibration required for accurancy
  compass.setCalibrationScales(1.03, 0.79, 1.31);           //The default calibration may need to change


  while (!Serial) {
    delay(100);  // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA());    // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
      return;                        // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 2000) {
    timer = millis();  // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }

    Serial.print("Fix: ");
    Serial.println((int)GPS.fix);
    if (GPS.fix) {


      if (WiFi.status() != WL_CONNECTED) {
        wifi_init();
      }

      bool coordinatesUpdated = populate_coords();
      if (coordinatesUpdated) {
        run();
      }
    }
  }
  /*
  
  delay(5000);
*/
}

void wifi_init() {

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    delay(1000);
  }

  print_wifi_status();
}

void print_wifi_status() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

bool populate_coords() {
  if (!client.connect(server, 80)) {
    Serial.println("connection failed");
    return false;
  }
  Serial.println("Connected to server");
  // Make a HTTP request:
  client.println("GET /test/GUI/test/robot.txt HTTP/1.1");
  client.println("Host: pcr.bounceme.net");
  client.println("Connection: close");
  client.println();

  // block until ready to read
  while (!client.connected() || !client.available())
    ;

  int i = 0;
  coord newLatLongs[5] = {};

  while (client.available()) {
    String line = client.readStringUntil('\n');
    // if first character is not numeric, skip this line, it's the HTTP header or fluff before our coordinates
    if (line.length() == 0 || !isdigit(line[0])) {
      continue;
    }
    char* lat = strtok((char*)line.c_str(), ", ");
    char* lng = strtok(NULL, ", ");
    if (i < 5) {
      newLatLongs[i++] = (coord){
        .latitude = strtod(lat, NULL),
        .longitude = strtod(lng, NULL),
      };
    }
  }

  Serial.println("fetched data, now disconnecting from server.");
  client.stop();

  // compare for equality, if not equal, update latLongs[]
  if (memcmp(latLongs, newLatLongs, i * sizeof(coord)) == 0) {
    return false;
  }

  // not equal, copy newLatLongs to latLongs and return true
  memcpy(latLongs, newLatLongs, 5 * sizeof(coord));
  return true;
}

void run() {
  Serial.println("new data!");
  for (int i = 0; i < 5; i++) {
    Serial.print("data lat long #");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(latLongs[i].latitude, 10);
    Serial.print(", ");
    Serial.println(latLongs[i].longitude, 10);
  }

  double rotation = determine_rotation_clockwise();
  if (rotation < 0) {
    rotation += 360;
  }
  // takes 2000 ms to turn 360
  // calculate how long it will take to turn the amount we need to turn
  int duration = int(rotation / 360) * 2000;
  Serial.print("Turning right by");
  Serial.print(rotation);
  Serial.println(" degrees.");
    right(50);
  slowdown();
  delay(duration);
  forward_until_destination();
}

void forward_until_destination() {
  coord position = get_current_lat_long();
  coord destination = latLongs[0];
  forward();
  double distance = get_distance(position, destination);
  // while distance > 5m, keep going forward
  while (distance > 0.005) {
    Serial.print("Distance is (in km): ");
    Serial.println(distance, 8);
    position = get_current_lat_long();
    Serial.print("new lat is: ");
    Serial.println(position.latitude, 8);
    Serial.print("new long is: ");
    Serial.println(position.longitude, 8);
    distance = get_distance(position, destination);
    delay(1000);
  }
  slowdown();
  Serial.println("Destination reached.");
}

double get_distance(coord position, coord destination) {
  double R = 6371;                                                  // Radius of the earth in km
  double dLat = deg2rad(destination.latitude - position.latitude);  // deg2rad below
  double dLon = deg2rad(destination.longitude - position.longitude);
  double a =
    sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(position.latitude)) * cos(deg2rad(destination.latitude)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;  // Distance in km
}

double deg2rad(double deg) {
  return deg * (M_PI / 180);
}

double determine_rotation_clockwise() {
  // find current lat/long
  // compare to desired lat/long
  // find current orientation vs true north
  // calculate angle to turn
  coord position = get_current_lat_long();
  coord destination = latLongs[0];
  double currentOrientation = get_azimuth();
  double angleFromNorth = atan2(destination.longitude - position.longitude, destination.latitude - position.latitude) * (180.0 / PI);
  // return degrees we need to turn to the right
  Serial.print("angle from north : ");
  Serial.println(angleFromNorth);
  Serial.print("Amount to rotate: ");
  Serial.println((angleFromNorth - currentOrientation));
  Serial.print("Location: ");
  Serial.print(GPS.latitude, 4);
  Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4);
  Serial.println(GPS.lon);
  return angleFromNorth - currentOrientation;
}

coord get_current_lat_long() {
  int gps_continue = 1;
  while(gps_continue) {
    GPS.read();
    if (GPS.newNMEAreceived()) {   
      if (GPS.parse(GPS.lastNMEA()) && GPS.fix) {
        gps_continue = 0;
      }
    }
  }

  // keep first 2 digits of lat / long, this is "hours"
  // for remaining digits (6), divide by 60
  // finally, flip sign if S / W lat/long respectively
  int lat_degrees = ((int) GPS.latitude) / 100;
  int long_degrees = ((int) GPS.longitude) / 100;
  double lat_minutes = GPS.latitude - (double) lat_degrees * 100;
  double long_minutes = GPS.longitude - (double) long_degrees * 100;
  double latitude = (double) lat_degrees + lat_minutes / 60.0;
  double longitude = (double) long_degrees + long_minutes / 60.0;
  if (GPS.lat == 'S') {
    latitude = -latitude;
  }
  if (GPS.lon == 'W') {
    longitude = -longitude;
  }
  return (coord){
    .latitude = latitude,
    .longitude = longitude,
    //debug values
    //.latitude = 42.40586789498043,
    //.longitude = -71.11705388640986,
  };
}

double get_azimuth() {
  compass.read();
  return compass.getAzimuth();
  // eventually this will be a call to the compass module, right now hardcoded as true north
  //return 0;
}


void forward() {
  Serial.println("Going Forward");
  digitalWrite(motorDirection1, 1);
  digitalWrite(motorDirection2, 0);
  delay(5);
  analogWrite(motorDriver1, SPEED);
  analogWrite(motorDriver2, SPEED);
}

void backward() {
  Serial.println("Going Back");
  digitalWrite(motorDirection1, LOW);
  digitalWrite(motorDirection2, HIGH);
  delay(5);
  analogWrite(motorDriver1, SPEED);
  analogWrite(motorDriver2, SPEED);
}

void left(int period) {
  Serial.println("Going Left");
  digitalWrite(motorDirection1, 0);
  digitalWrite(motorDirection2, 0);
  delay(period);
  analogWrite(motorDriver1, SPEED * 1.4);
  analogWrite(motorDriver2, SPEED * 1.4);
}

void right(int period) {
  Serial.println("Going Right");
  digitalWrite(motorDirection1, HIGH);
  digitalWrite(motorDirection2, HIGH);
  delay(period);
  analogWrite(motorDriver1, SPEED * 1.4);
  analogWrite(motorDriver2, SPEED * 1.4);
}


//this is NOT a brake
void slowdown() {
  analogWrite(motorDriver1, 0);
  analogWrite(motorDriver2, 0);
}
