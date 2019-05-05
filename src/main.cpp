#include <Arduino.h>
#include <PubSubClient.h>
#include "BasicStepperDriver.h"
#include <ESP8266WiFi.h>
#include "secret.h"

// Motor steps per revolution.
#define MOTOR_STEPS 200
#define RPM 20 //We need to go slow to stop the motor from skipping

// Microstepping
// 1 = full step, 2 = half step etc..
// Consider changing the setting to increase avaliable torque
#define MICROSTEPS 16

// All the wires needed for full functionality
#define DIR 8 // TODO
#define STEP 9 // TODO
#define ENABLE 6 // TODO
#define REED_PIN 10 // TODO, needs interrupt

// 3-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

// MQTT Definitions
WiFiClient espClient;
PubSubClient client(espClient);

// Global variable definitions
enum STATE{
  UNFOLD,
  FOLD,
  ROTATE,
  WAIT_FOR_COMMAND,
  ERROR,
  CONNECT_WIFI,
  CONNECT_MQTT
};
enum ROLLER_STATE{
  FOLDED,
  UNFOLDED
};

int state;
int motor_pos;
int rotation_count; 
int full_extend_rotation_count;
int full_stowed_rotation_count;
const int SAFETY_STOP = 1000;
int safety_millis;
int previous_position;
int target_position;

// Global Function Declarations
void InterruptHandler();
void MQTTcallback(char* topic, byte* payload, unsigned int length);

void setup() {
  // Stepper Motor initialization
  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(LOW);
  ///////////////////////////////////////////////////////////////////

  // MQTT Initialization
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(MQTTcallback);
  ///////////////////////////////////////////////////////////////////

  // Attach Interrupt to reed sensor
  rotation_count = 0;
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), InterruptHandler, FALLING);
  ///////////////////////////////////////////////////////////////////

  // State Machine Initialization
  state = WAIT_FOR_COMMAND;
  full_extend_rotation_count = 15;
  full_stowed_rotation_count = 0;
  motor_pos = 0;
  
  ///////////////////////////////////////////////////////////////////
}

void loop() {
  // Main State Machine

  switch(state){
    case UNFOLD:
      // First enable the stepper      
      stepper.enable();

      safety_millis = millis(); // Mark the start time of the rotation
      previous_position = rotation_count;
      
      stepper.startRotate(20 * 360); // Start rotating
      // Initialize the counter to keep track of performed rotations
      rotation_count = 0;
      target_position = full_extend_rotation_count;
      state = ROTATE;
      break;
    case FOLD:
      // First enable the stepper      
      stepper.enable();

      safety_millis = millis(); // Mark the start time of the rotation
      previous_position = rotation_count;
      
      stepper.startRotate(-20 * 360); // Start rotating
      // Initialize the counter to keep track of performed rotations
      rotation_count = 0;
      target_position = full_extend_rotation_count;
      state = ROTATE;
      break;
    case ROTATE:
      motor_pos = rotation_count;
      if(motor_pos - target_position <= 0){
        // If the current positon of roller is less than the number of
        // rotations to reach the full extension keep rotating the stepper
        stepper.stop();
        stepper.disable();
        // Go back to WAIT state
        state = WAIT_FOR_COMMAND;
      }

      if(millis()-safety_millis > SAFETY_STOP){
        // If one revolution takes more than perscribed ammount
        // the stepper is likely blocked, in this case stop rotation
        stepper.stop();
        stepper.disable();
        // Go to ERROR state
        state = ERROR;
      }

      if(rotation_count > previous_position){
        // If we rotated at least one revolution reset the timer
        safety_millis = 0;
      }


      break;
    case WAIT_FOR_COMMAND:
      // TODO
      break;
    case ERROR:
      // TODO
      break;
    case CONNECT_WIFI:
      delay(10);
      // We start by connecting to a WiFi network
      WiFi.begin(WIFI_SSID, WIFI_PASS);

      while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      //Serial.print(".");
      }

      //Serial.println("");
      //Serial.println("WiFi connected");
      //Serial.println("IP address: ");
      //Serial.println(WiFi.localIP());
      break;
    case CONNECT_MQTT:
      // Loop until we're reconnected
      while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP8266Client")) {
          Serial.println("connected");
          // Once connected, publish an announcement...
          client.publish("outTopic", "hello world");
          // ... and resubscribe
          client.subscribe("inTopic");
        } else {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          // Wait 5 seconds before retrying
          delay(5000);
        }
      break;
    default:
      // TODO
      break;
  }

}

// Global Function Definitions
void InterruptHandler() {
  rotation_count++;
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}