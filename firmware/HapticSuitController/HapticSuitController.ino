#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

// Include the PID class
#include "PID.cpp"

// Define the number of motors
#define NMOTORS 2

// Define the wifi details. Leave blank for no wifi
char ssid[] = "ViconNetwork";   
char password[] = "LessLabUVA1234";  

// Define the wifi status
int status = WL_IDLE_STATUS;

// Define variable for communication
unsigned int localPort = 8888;     
char packetBuffer[SOCKET_BUFFER_UDP_SIZE + 1]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// Define the communication protocal
WiFiUDP Udp;

// Define variables for the motors
Servo esc_1;
Servo esc_2;
int motor_angle[NMOTORS]    = {0,0};
int motor_throttle[NMOTORS] = {0,0};

// 70 rpm, around 1:150 gear ratio
const int PPR = 1050;

// Create the IMU 
BNO080 myIMU1; //Open I2C ADR jumper - goes to address 0x4B
BNO080 myIMU2; //Closed I2C ADR jumper - goes to address 0x4A

// Create variables to hold the IMU data
float current_roll[] = {0.0, 0.0};

//pins
const int pwm[] = {5,11};
const int in1[] = {12,14};
const int in2[] = {6,13};

// Globals
long previous_time = 0; // prevT

// PID class instances
SimplePID pid[NMOTORS];

// Define setup
void setup()
{
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);

  analogWriteResolution(8);
  esc_1.attach(9, 1000, 2000);
  esc_2.attach(10, 1000, 2000);
  esc_1.write(0);
  esc_2.write(0);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to the network
    if(strlen(password) == 0)
    {
      status = WiFi.begin(ssid);
    } 
    else
    {
      status = WiFi.begin(ssid, password);
    }
      
    // wait 10 seconds for connection:
    delay(10000);
  }
  
  // Print you are connected to wifi and print the status (includes the ip address)
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");

  // Start a UDP port to accept commands
  Udp.begin(localPort);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  for(int k = 0; k < NMOTORS; k++){
    // Set the different pins
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    // Set the different PID paramters
    pid[0].setParams(4,0.072,0,0,255);
    pid[1].setParams(2,0.036,0,0,255);
    // pid[0].setParams(4,0.2,0,50,255);
    // pid[1].setParams(2,0.1,0,20,255);
    // pid[0].setParams(6,0.108,0,0,255);
    // pid[1].setParams(4,0.072,0,0,255);
  }

  // Start both IMU sensors
  myIMU1.begin(0x4B);
  myIMU2.begin(0x4A);

  // Set the update to 5ms
  myIMU1.enableRotationVector(5); 
  myIMU2.enableRotationVector(5); 

  Serial.println("\nSetup finished");
}

void loop()
{

  Serial.println("\nStarting Loop");

  // Check if there is a UDP packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {

    // Print the package size and ip address and port
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // Read the packet into packetBufffer
    int len = Udp.read(packetBuffer, SOCKET_BUFFER_UDP_SIZE);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    char *t1 = subStr(packetBuffer, 1);
    char *t2 = subStr(packetBuffer, 2);
    char *m1 = subStr(packetBuffer, 3);
    char *m2 = subStr(packetBuffer, 4);

    Serial.println("Values:");
    Serial.println(t1);
    Serial.println(t2);
    Serial.println(m1);
    Serial.println(m2);
    
    motor_throttle[0] = atoi(&t1[0]);
    motor_throttle[1] = atoi(&t2[0]);
    motor_angle[0]    = atoi(&m1[0]);
    motor_angle[1]    = atoi(&m2[0]);

    // Send a reply
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write("Throttle 1: ");
    Udp.print(motor_throttle[0]);
    Udp.write("\t Throttle 2: ");
    Udp.print(motor_throttle[1]);
    Udp.write("\t Motor Speed 1: ");
    Udp.print(motor_angle[0]);
    Udp.write("\t Motor Speed 2: ");
    Udp.println(motor_angle[1]);
    Udp.endPacket();
  }

  // Look for reports from the IMU1
  if (myIMU1.dataAvailable() == true)
  {
    current_roll[0] = (myIMU1.getRoll()) * 180.0 / PI; // Convert roll to degrees
  }
  if (myIMU2.dataAvailable() == true)
  {
    current_roll[1] = (myIMU2.getRoll()) * 180.0 / PI; // Convert roll to degrees
  }

  Serial.println(String("\ncurrent_roll[0]: ") + String(current_roll[0], 2) + String("\tcurrent_roll[1]: ") + String(current_roll[1], 2));

  // // Set a target position
  // float target[NMOTORS];
  // for(int k=0; k<NMOTORS; k++)
  // {
  //   target[k] = map (motor_angle[k], 0, 180, 0, PPR) - map (current_roll[k], 0, 180, 0, PPR);
  // }

  // // Get the time difference
  // long currT = micros();
  // float deltaT = ((float) (currT - previous_time))/( 1.0e6 );
  // previous_time = currT;

  // // Loop through the motors
  // for(int k = 0; k < NMOTORS; k++)
  // {
  //   int pwr, dir;
  //   // Evaluate the control signal
  //   pid[k].evalu(target[k],deltaT,pwr,dir);
  //   // Signal the motor
  //   setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  // }

  // esc_1.write(motor_throttle[0]);
  // esc_2.write(motor_throttle[1]);
}

void printWiFiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW); 
  }  
}

// Segment the strings
char* subStr (char* input_string, int segment_number) 
{
  char *act, *sub, *ptr;
  static char copy[100];
  int i;
 
  strcpy(copy, input_string);
  for (i = 1, act = copy; i <= segment_number; i++, act = NULL) 
  {
    sub = strtok_r(act, ",", &ptr);
    if (sub == NULL) break;
  }
  return sub;
}
