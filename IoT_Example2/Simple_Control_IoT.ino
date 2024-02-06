#include <WiFi.h>
#include <WebServer.h>

/* Put your SSID & Password */
const char* ssid = "Hasta la vista, baby";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

uint8_t LED1pin = 4;
bool LED1status = LOW;

uint8_t LED2pin = 5;
bool LED2status = LOW;

bool forward_status = LOW;
bool backward_status = LOW;
bool left_status = LOW;
bool right_status = LOW;
bool spin_status = LOW;


void setup() {
  Serial.begin(115200);
  pinMode(LED1pin, OUTPUT);
  pinMode(LED2pin, OUTPUT);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  
  server.on("/", handle_OnConnect);
  server.on("/led1on", handle_led1on);
  server.on("/led1off", handle_led1off);
  server.on("/led2on", handle_led2on);
  server.on("/led2off", handle_led2off);

  //movement commands
  server.on("/forwardon", handle_forwardon);
  server.on("/forwardoff", handle_forwardoff);
  server.on("/backwardon", handle_backwardon);
  server.on("/backwardoff", handle_backwardoff);
  server.on("/lefton", handle_lefton);
  server.on("/leftoff", handle_leftoff);
  server.on("/righton", handle_righton);
  server.on("/rightoff", handle_rightoff);  
  server.on("/spinon", handle_spinon);
  server.on("/spinoff", handle_spinoff);    

  
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
  if(LED1status)
  {digitalWrite(LED1pin, HIGH);}
  else
  {digitalWrite(LED1pin, LOW);}
  
  if(LED2status)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_forwardon)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_forwardoff)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_backwardon)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_backwardoff)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_lefton)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_leftoff)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_righton)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_rightoff)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_spinon)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}

  if(handle_spinoff)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}
}

void handle_OnConnect() {
  LED1status = LOW;
  LED2status = LOW;
  forward_status = LOW;
  backward_status = LOW;
  left_status = LOW;
  right_status = LOW;
  spin_status = LOW;
  Serial.println("GPIO4 Status: OFF | GPIO5 Status: OFF");
  server.send(200, "text/html", SendHTML(LED1status,LED2status)); 
}

void handle_led1on() {
  LED1status = HIGH;
  Serial.println("GPIO4 Status: ON");
  server.send(200, "text/html", SendHTML(true,LED2status)); 
}

void handle_led1off() {
  LED1status = LOW;
  Serial.println("GPIO4 Status: OFF");
  server.send(200, "text/html", SendHTML(false,LED2status)); 
}

void handle_led2on() {
  LED2status = HIGH;
  Serial.println("GPIO5 Status: ON");
  server.send(200, "text/html", SendHTML(LED1status,true)); 
}

void handle_led2off() {
  LED2status = LOW;
  Serial.println("GPIO5 Status: OFF");
  server.send(200, "text/html", SendHTML(LED1status,false)); 
}

void handle_forwardon() {
  forward_status = HIGH;
  Serial.println("Forward on");
  server.send(200, "text/html", SendHTML(forward_status,true)); 
}

void handle_forwardoff() {
  forward_status = LOW;
  Serial.println("Forward off");
  server.send(200, "text/html", SendHTML(forward_status,false)); 
}

void handle_backwardon() {
  backward_status = HIGH;
  Serial.println("Backward on");
  server.send(200, "text/html", SendHTML(backward_status,true)); 
}

void handle_backwardoff() {
  backward_status = LOW;
  Serial.println("Backward off");
  server.send(200, "text/html", SendHTML(backward_status,false)); 
}

void handle_lefton() {
  left_status = HIGH;
  Serial.println("Left on");
  server.send(200, "text/html", SendHTML(left_status,true)); 
}

void handle_leftoff() {
  left_status = LOW;
  Serial.println("Left off");
  server.send(200, "text/html", SendHTML(left_status,false)); 
}

void handle_righton() {
  right_status = HIGH;
  Serial.println("Right on");
  server.send(200, "text/html", SendHTML(right_status,true)); 
}

void handle_rightoff() {
  right_status = LOW;
  Serial.println("Right off");
  server.send(200, "text/html", SendHTML(right_status,false)); 
}

void handle_spinon() {
  spin_status = HIGH;
  Serial.println("Spin on");
  server.send(200, "text/html", SendHTML(spin_status,true)); 
}

void handle_spinoff() {
  spin_status = LOW;
  Serial.println("Spin off");
  server.send(200, "text/html", SendHTML(spin_status,false)); 
}


void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(uint8_t led1stat,uint8_t led2stat){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>Agrobot Control</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  /* if(led1stat)
  {ptr +="<p>LED1 Status: ON</p><a class=\"button button-off\" href=\"/led1off\">OFF</a>\n";}
  else
  {ptr +="<p>LED1 Status: OFF</p><a class=\"button button-on\" href=\"/led1on\">ON</a>\n";}

  if(led2stat)
  {ptr +="<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n";}
  else
  {ptr +="<p>LED2 Status: OFF</p><a class=\"button button-on\" href=\"/led2on\">ON</a>\n";}
  */
  //added
  if(forward_status)
  {ptr +="<p>Forward Status: ON</p><a class=\"button button-off\" href=\"/forwardoff\">OFF</a>\n";}
  else
  {ptr +="<p>Forward Status: OFF</p><a class=\"button button-on\" href=\"/forwardon\">ON</a>\n";}

  if(backward_status)
  {ptr +="<p>Backward Status: ON</p><a class=\"button button-off\" href=\"/backwardoff\">OFF</a>\n";}
  else
  {ptr +="<p>Backward Status: OFF</p><a class=\"button button-on\" href=\"/backwardon\">ON</a>\n";}

  if(left_status)
  {ptr +="<p>Left Status: ON</p><a class=\"button button-off\" href=\"/leftoff\">OFF</a>\n";}
  else
  {ptr +="<p>Left Status: OFF</p><a class=\"button button-on\" href=\"/lefton\">ON</a>\n";}

  if(right_status)
  {ptr +="<p>Right Status: ON</p><a class=\"button button-off\" href=\"/rightoff\">OFF</a>\n";}
  else
  {ptr +="<p>Right Status: OFF</p><a class=\"button button-on\" href=\"/righton\">ON</a>\n";}

  if(spin_status)
  {ptr +="<p>Spin Status: ON</p><a class=\"button button-off\" href=\"/spinoff\">OFF</a>\n";}
  else
  {ptr +="<p>Spin Status: OFF</p><a class=\"button button-on\" href=\"/spinon\">ON</a>\n";}

  


  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
