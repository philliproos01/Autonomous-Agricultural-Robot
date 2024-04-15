#include <Arduino.h>
#include <WiFi.h>
#include <QMC5883LCompass.h>
#include <Adafruit_GPS.h>

// obviously change this to your own SSID and password
#define ssid "TMOBILE-3422"
#define pass "chip018big6268hits"
#define server "pcr.bounceme.net"

#define GPSSerial Serial2

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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
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
  compass.setCalibrationOffsets(512.00, 1093.00, -658.00); //Compass calibration required for accurancy
  compass.setCalibrationScales(1.03, 0.79, 1.31); //The default calibration may need to change


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
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    
    Serial.print("Fix: "); Serial.println((int)GPS.fix);
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
  while (!client.connected() || !client.available());

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
  Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
  return angleFromNorth - currentOrientation;
}

coord get_current_lat_long() {
  return (coord) {
      .latitude = GPS.latitude,
      .longitude = GPS.longitude,
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
