#include <Arduino.h>
#include <PubSubClient.h>
#include "BasicStepperDriver.h"
#include <ESP8266WiFi.h>
#include "secret.h"
#include <string.h>

// Motor steps per revolution.
#define MOTOR_STEPS 200
#define RPM 20 //We need to go slow to stop the motor from skipping

// Microstepping
// 1 = full step, 2 = half step etc..
// Consider changing the setting to increase avaliable torque
#define MICROSTEPS 16

// All the wires needed for full functionality
#define DIR 8       // TODO
#define STEP 9      // TODO
#define ENABLE 6    // TODO
#define REED_PIN 10 // TODO, needs interrupt

// 3-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

// MQTT Definitions
#define MQTT_TOPIC_CMD_PREFIX "cmd"
#define MQTT_TOPIC_STATUS_PREFIX "stat"
#define MQTT_DEV_ID "EspMotorController"

WiFiClient espClient;
PubSubClient client(espClient);

// Global variable definitions
enum STATE
{
  UNFOLD,
  FOLD,
  ROTATE,
  WAIT_FOR_COMMAND,
  ERROR,
  CONNECT_WIFI,
  CONNECT_MQTT
};
enum ROLLER_STATE
{
  FOLDED,
  UNFOLDED,
  IN_TRANSITION
};

enum DIRECTION
{
  DIR_FOLD = 1,
  DIR_UNFOLD = -1
};

int state;
int rotor_position;
int rotation_count;
int full_extend_rotation_count;
int full_stowed_rotation_count;
const int SAFETY_STOP = 1000;
int safety_millis;
int previous_position;
int target_position;
int direction;

// Global Function Declarations
void InterruptHandler();
void MQTTCallback(char *topic, byte *payload, unsigned int length);

void setup()
{
  // Stepper Motor initialization
  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(LOW);
  ///////////////////////////////////////////////////////////////////

  // MQTT Initialization
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(MQTTCallback);
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
  rotor_position = 0;

  ///////////////////////////////////////////////////////////////////
}

void loop()
{
  // Main State Machine

  switch (state)
  {
  case UNFOLD:
    // First enable the stepper
    stepper.enable();

    safety_millis = millis(); // Mark the start time of the rotation
    previous_position = rotation_count;

    stepper.startRotate(20 * 360); // Start rotating
    target_position = full_extend_rotation_count;
    direction = DIR_UNFOLD;
    state = ROTATE;
    break;

  case FOLD:
    // First enable the stepper
    stepper.enable();

    safety_millis = millis(); // Mark the start time of the rotation
    previous_position = rotation_count;

    stepper.startRotate(-20 * 360); // Start rotating
    target_position = full_stowed_rotation_count;
    direction = DIR_FOLD;

    state = ROTATE;
    break;

  case ROTATE:
    rotor_position = rotation_count;
    if (rotor_position - direction*target_position <= 0)
    {
      // If the current position reached the target, stop the motor
      stepper.stop();
      stepper.disable();
      // Go back to WAIT state
      state = WAIT_FOR_COMMAND;
    }

    if (millis() - safety_millis > SAFETY_STOP)
    {
      // If one revolution takes more than perscribed amount
      // the stepper is likely blocked, in this case stop rotation
      stepper.stop();
      stepper.disable();
      // Go to ERROR state
      state = ERROR;
    }

    if (abs(rotation_count - previous_position) > 1)
    {
      // If we rotated at least one revolution reset the timer
      safety_millis = millis();
    }

    // TODO need to add the motor loop to program main loop
    break;

  case WAIT_FOR_COMMAND:
    // TODO

    // If we are not connected to Wifi -> connect
    // TODO add timer to stop it running too often
    if (WiFi.status() != WL_CONNECTED)
    {
      state = CONNECT_WIFI;
    }

    // If we are not connected to MQTT -> connect
    // TODO add timer to stop it running too often
    if (!client.connected())
    {
      state = CONNECT_MQTT;
    }

    break;

  case ERROR:
    // TODO
    break;

  case CONNECT_WIFI:
    // We start by connecting to a WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    //Serial.println("");
    //Serial.println("WiFi connected");
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());

    // Finish by going to default state (WAIT_FOR_COMMAND)
    state = WAIT_FOR_COMMAND;

    break;

  case CONNECT_MQTT:
    Serial.print("Attempting MQTT connection...");

    // Prepare topic for subscription

    char *subtopic = "";
    strcat(subtopic, MQTT_TOPIC_CMD_PREFIX);
    strcat(subtopic, "/");
    strcat(subtopic, MQTT_DEV_ID);
    strcat(subtopic, "/#");

    // Attempt to connect
    if (client.connect(MQTT_DEV_ID, MQTT_USER, MQTT_PASS))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(subtopic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
    // Finish by going to default state (WAIT_FOR_COMMAND)
    state = WAIT_FOR_COMMAND;
    break;

  default:
    // TODO
    break;
  }
  // Section containing calls to functions that need to be run periodically ///
  client.loop();

  unsigned wait_time_micros = stepper.nextAction();
  // 0 wait time indicates the motor has stopped
  if (wait_time_micros <= 0)
  {
    stepper.disable(); // comment out to keep motor powered
    delay(3600000);
  }
  // (optional) execute other code if we have enough time
  if (wait_time_micros > 100)
  {
    // other code here
  }
  /////////////////////////////////////////////////////////////////////////////
}

// Global Function Definitions
void InterruptHandler()
{
  // Depending on motor direction, keep track of true position of the rotor
  rotation_count += direction;
}

void MQTTCallback(char *topic, byte *payload, unsigned int length)
{
  // handle message arrived
}