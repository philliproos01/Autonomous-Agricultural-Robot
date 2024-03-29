#include <Arduino.h>
#include <WiFi.h>

// obviously change this to your own SSID and password
#define ssid "Tufts_Robot"
#define pass ""
#define server "pcr.bounceme.net"

struct coord {
  double latitude;
  double longitude;
};

WiFiClient client;
coord latLongs[5] = {};

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    delay(100);  // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    wifi_init();
  }

  bool coordinatesUpdated = populate_coords();
  if (coordinatesUpdated) {
    run();
  }
  delay(5000);

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

  double rotation = determine_rotation();
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
  Serial.print(angleFromNorth);
  return angleFromNorth - currentOrientation;
}

coord get_current_lat_long() {
  // eventually this will be a call to the GPS module, right now hardcoded as mock
  return (coord) {
    .latitude = 42.40586789498043,
    .longitude = -71.11705388640986,
  };
}

double get_azimuth() {
  // eventually this will be a call to the compass module, right now hardcoded as true north
  return 0;
}