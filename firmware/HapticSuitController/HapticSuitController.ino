#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

// Include the PID class
#include "PID.cpp"

// Define the number of motors
#define NUMBER_ARMS 2
// Define the IMU start address (note they go in decreasing order from this point)
#define IMU_START_ADDRESS 0x4B
// Define the wifi SSID and password (set to "" for no password)
#define WIFI_SSID "ViconNetwork"
#define WIFI_PASSWORD "LessLabUVA1234"
// Define if you want to enable wifi (makes debugging easier)
#define ENABLE_WIFI false
// Define the UDP communication port
#define UDP_PORT 8888
// Define the gear ratio of (1:150) ~70RPM
#define PPR 1050

// Define the wifi status
int status = WL_IDLE_STATUS;

// Define variable for communication
unsigned int localPort = UDP_PORT;     
char packetBuffer[SOCKET_BUFFER_UDP_SIZE + 1]; 
char  ReplyBuffer[] = "acknowledged";

// Define the communication protocal
WiFiUDP Udp;

// Define variables for the motors
Servo esc_array[NUMBER_ARMS];
int   motor_angle[NUMBER_ARMS]    = {0,0};
int   motor_throttle[NUMBER_ARMS] = {0,0};


// Create the IMU (imu1 goes to 0x4B, imu2 goes to 0x4A)
BNO080 imu_array[NUMBER_ARMS];

// Create variables to hold the IMU data
float current_roll[NUMBER_ARMS];
float previous_roll[NUMBER_ARMS];
float global_roll[NUMBER_ARMS];
int   roll_state[NUMBER_ARMS];

// Declare the pins
const int pwm[] = {5,  11};
const int in1[] = {12, 14};
const int in2[] = {6,  13};

// Globals
long previous_time = 0; // prevT

// PID class instances
SimplePID pid[NUMBER_ARMS];

// Define setup
void setup()
{
  // Allow time for startup
  delay(5000);

  // Configure communication
  Serial.begin(115200);

  // Start the setup
  delay(100);
  Serial.println();
  Serial.println("Program Setup Starting");

  // Init variables
  Serial.println("Init variables");
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    current_roll[k] = 0;
    previous_roll[k] = 0;
    global_roll[k] = 0;
    roll_state[k] = 0;
  }

  // Start wifi
  if (ENABLE_WIFI)
  {
    Serial.println("Setting up WIFI");
    // Configure pins for Adafruit ATWINC1500 Feather
    WiFi.setPins(8, 7, 4, 2);

    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD)
    {
      while (1)
      {
        Serial.println("WiFi shield not present");
        delay(1000);
      }
    }

    // attempt to connect to WiFi network:
    while ( status != WL_CONNECTED) {
      Serial.println("Attempting to connect to SSID: " + String(WIFI_SSID) + "\n");
      // Connect to the network
      if(strlen(WIFI_PASSWORD) == 0)
      {
        // No password
        status = WiFi.begin(WIFI_SSID);
      } 
      else
      {
        // Password
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
        
      // wait 10 seconds for connection:
      delay(10000);
    }
    
    // Print you are connected to wifi and print the status (includes the ip address)
    Serial.println("Connected to wifi");
    printWiFiStatus();

    // Start a UDP port to accept commands
    Udp.begin(localPort);
  }

  // Setup the esc motors
  Serial.println("Attaching motors");
  analogWriteResolution(8);
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    esc_array[k].attach(9+k, 1000, 2000);
    esc_array[k].write(0);

    // Set the different pins
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
  }
  delay(100);


  // Set the different pins to control the motors
  Serial.println("Creating controllers");
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    // Set the different PID paramters
    pid[0].setParams(4, 0.072, 0, 0, 255);
    pid[1].setParams(2, 0.036, 0, 0, 255);
  }
  delay(100);

  //Increase I2C data rate to 400kHz
  Serial.println("Increasing I2C rate");
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  // Start the IMU sensors
  Serial.println("Connecting to IMUs");
  bool imu_success[NUMBER_ARMS];
  int imu_address = IMU_START_ADDRESS;
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    delay(1000);
    imu_success[k] = imu_array[k].begin(imu_address);
    imu_address = imu_address -1;
    delay(1000);
  }

  // Check the IMU was a success
  if ((imu_success[0] == false) or (imu_success[1] == false))
  {
    while (1)
    {
      if (imu_success[0] == false)
      {
        Serial.println("BNO080 not detected at 0x4B I2C address. Check your jumpers and the hookup guide. Freezing...");
      }
      if (imu_success[1] == false)
      {
        Serial.println("BNO080 not detected at 0x4A I2C address. Check your jumpers and the hookup guide. Freezing...");
      }
      delay(1000);
    }
  }

  Serial.println("Enabling rotation Vectors on IMUs");
  // Set IMU's update to 5ms
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    imu_array[k].enableRotationVector(5);
    delay(100); 
  }

  // Print finished
  Serial.println("Setup finished\n----------------------\n");
}

void loop()
{

  // If wifi is available - Check if there is a UDP packet
  if (ENABLE_WIFI)
  {
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
  }

  // Get the latest IMU data
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    if (imu_array[k].dataAvailable() == true)
    {
      // Get the current roll
      current_roll[k] = (imu_array[k].getRoll()) * 180.0 / PI; 
      // Keep track of the global roll value
      global_roll[k] = update_global_roll(current_roll[k], previous_roll[k], &roll_state[k]);
      // Update the previous roll
      previous_roll[k] = current_roll[k];
    }
    Serial.println(String("IMU") + String(k) + " reading: " + String(global_roll[k], 2));
  }
  Serial.println("---------------------");

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
  delay(1000);
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
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
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

// Computes the global roll value (i.e removes the 180 discontinuty)
double update_global_roll(double cur_reading, double prev_reading, int* state)
{
  int current_state = *state;
  double diff = cur_reading - prev_reading;
  // If the sign has changed
  if (abs(diff) > 180)
  {
    if (diff > 0)
    {
      // Update the state
      current_state = current_state - 1;
    }
    else
    {
      // Update the state
      current_state = current_state + 1;
    }
  }
  // Save the state
  *state = current_state;

  float global_roll = (current_state * 360) + cur_reading;

  return global_roll;
}
