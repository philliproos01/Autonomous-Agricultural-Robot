#include <WiFi.h>
#include <WebServer.h>

//wheel parameters
const unsigned long SPEED_TIMEOUT = 500000;   // Time used to determine wheel is not spinning
const unsigned int UPDATE_TIME = 500;         // Time used to output serial data
const unsigned int BUFFER_SIZE = 16;          // Serial receive buffer size
const double BAUD_RATE = 115200;              // Serial port baud rate
const double WHEEL_DIAMETER_IN = 6.5;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 22.25;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 16.5;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 56.5;      // Motor wheel circumference (centimeters)

const int _pwmCtrl = 25;

/* Put your SSID & Password */
const char* ssid = "Hasta la vista, baby";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

double _freq;               // Frequency of the signal on the speed pin
double _rpm;                // Wheel speed in revolutions per minute
double _mph;                // Wheel speed in miles per hour
double _kph;                // Wheel speed in kilometers per hour

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);
//left motor
uint16_t motorDirection1 = 33;
uint16_t motorDriver1 = 32;

//right motor
uint16_t motorDirection2 = 4; //set to 4
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


void setup() {
  Serial.begin(115200);
  while(!Serial);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  
  server.on("/", handle_OnConnect);

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
 // Set DIR pin to output
    pinMode(motorDirection1, OUTPUT);
    pinMode(motorDirection2, OUTPUT);
    // Set PWM pin to output
    pinMode(motorDriver1, OUTPUT);
    pinMode(motorDriver2, OUTPUT);
  
    //feedback pin for motor 1
    pinMode(PIN_SPEED, INPUT);
    digitalWrite(motorDirection1, _dir);
    digitalWrite(motorDirection2, _dir2);
    analogWrite(motorDriver1, 0);
    analogWrite(motorDriver2, 0);
    Serial.println("Starting motor");

}
void loop() {
  server.handleClient();
  
  if(forward_status) {
      forward();
  } else if(backward_status) {
    backward();
  } else if (left_status) {
    left(50);
  } else if (right_status) {
    right(50);
  } else if (spin_status) {
    spin(); 
  } else {
    slowdown();
  }
}

void handle_OnConnect() {
  forward_status = LOW;
  backward_status = LOW;
  left_status = LOW;
  right_status = LOW;
  spin_status = LOW;
  Serial.println("GPIO4 Status: OFF | GPIO5 Status: OFF");
  server.send(200, "text/html", SendHTML(forward_status,false)); 
  server.send(200, "text/html", SendHTML(backward_status,false)); 
  server.send(200, "text/html", SendHTML(right_status,false)); 
  server.send(200, "text/html", SendHTML(left_status,false)); 
  server.send(200, "text/html", SendHTML(spin_status,false));  
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
    analogWrite(motorDriver1, SPEED);
    analogWrite(motorDriver2, SPEED);
    
    

    //slowdown();
}

void right(int period) {
    Serial.println("Going Right");
    digitalWrite(motorDirection1, HIGH);
    digitalWrite(motorDirection2, HIGH);
    delay(period); 
    analogWrite(motorDriver1, SPEED);
    analogWrite(motorDriver2, SPEED);
}


void spin() {
    Serial.println("Spinning");
    digitalWrite(motorDirection1, 1);
    digitalWrite(motorDirection2, 1);
    delay(100);
    analogWrite(motorDriver1, SPEED);
    analogWrite(motorDriver2, SPEED);

}


//this is NOT a brake
void slowdown() {
    analogWrite(motorDriver1, 0);
    analogWrite(motorDriver2, 0);
    /*digitalWrite(motorDirection1, _dir);
    digitalWrite(motorDirection2, _dir);
    analogWrite(motorDriver1, 0);
    analogWrite(motorDriver2, 0);*/

}
/*
void ReadSpeed(int PIN)
{
    static bool lastState = false;    // Saves the last state of the speed pin
    static unsigned long last_uS;     // The time (µs) when the speed pin changes
    static unsigned long timeout_uS;  // Timer used to determine the wheel is not spinning

    // Read the current state of the input pin
    bool state = digitalRead(PIN);

    // Check if the pin has changed state
    if (state != lastState)
    {
      // Calculate how long has passed since last transition
      unsigned long current_uS = micros();
      unsigned long elapsed_uS = current_uS - last_uS;

      // Calculate the frequency of the input signal
      double period_uS = elapsed_uS * 2.0;
      _freq = (1 / period_uS) * 1E6;

      // Calculate the RPM
      _rpm = _freq / 45 * 60;

      // If RPM is excessively high then ignore it.
      if (_rpm > 5000) _rpm = 0;

      // Calculate the miles per hour (mph) based on the wheel diameter or circumference
      //_mph = (WHEEL_DIAMETER_IN * PI * _rpm * 60) / 63360;
      _mph = (WHEEL_CIRCUMFERENCE_IN * _rpm * 60) / 63360; 
  
      // Calculate the miles per hour (kph) based on the wheel diameter or circumference
      //_kph = (WHEEL_DIAMETER_CM * PI * _rpm * 60) / 1000;
      _kph = (WHEEL_CIRCUMFERENCE_CM * _rpm * 60) / 100000; 

      // Save the last state and next timeout time
      last_uS = current_uS;
      timeout_uS = last_uS + SPEED_TIMEOUT;
      lastState = state;
    }
    // If too long has passed then the wheel has probably stopped
    else if (micros() > timeout_uS)
    {
        _freq = 0;
        _rpm = 0;
        _mph = 0;
        _kph = 0;
        last_uS = micros();
    }
}

void WriteToSerial()
{
    // Local variables
    static unsigned long updateTime;
    
    if (millis() > updateTime)
    {
        // Write data to the serial port
        Serial.print((String)"Freq:" + _freq + " ");
        Serial.print((String)"RPM:" + _rpm + " ");
        Serial.print((String)"MPH:" + _mph + " ");
        Serial.println((String)"KPH:" + _kph + " ");

        // Calculate next update time
        updateTime = millis() + UPDATE_TIME;
    }
}
*/

