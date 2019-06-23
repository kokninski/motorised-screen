#include <Arduino.h>
#include <PubSubClient.h>
#include "BasicStepperDriver.h"
#include <ESP8266WiFi.h>
#include "secret.h"
#include <string.h>

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

// Motor steps per revolution.
#define MOTOR_STEPS 200
#define RPM 20 //We need to go slow to stop the motor from skipping

// Microstepping
// 1 = full step, 2 = half step etc..
// Consider changing the setting to increase avaliable torque
#define MICROSTEPS 16

// All the wires needed for full functionality
#define DIR 5       // TODO
#define STEP 4      // TODO
#define ENABLE 13   // TODO
#define REED_PIN 12 // TODO, needs interrupt

// 3-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

// MQTT Definitions
#define MQTT_TOPIC_CMD_PREFIX "cmnd"
#define MQTT_TOPIC_STATUS_PREFIX "stat"
#define MQTT_DEV_ID "EspMotorController"

#define MQTT_TOPIC_SUBSCRIBE MQTT_TOPIC_CMD_PREFIX "/" MQTT_DEV_ID "/"
#define MQTT_TOPIC_SUBSCRIBE_WILDCARD MQTT_TOPIC_SUBSCRIBE "#"
#define MQTT_TOPIC_SUBSCRIBE_POWER MQTT_TOPIC_SUBSCRIBE "power"
#define MQTT_TOPIC_STATUS MQTT_TOPIC_STATUS_PREFIX "/" MQTT_DEV_ID "/"
#define MQTT_TOPIC_STATUS_STATE MQTT_TOPIC_STATUS "state"

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
char status[10];
long mqtt_update_period;
long mqtt_time_variable = 0;

// Global Function Definitions
ICACHE_RAM_ATTR void InterruptHandler()
{
  // Depending on motor direction, keep track of true position of the rotor
  rotation_count += direction;
}

void MQTTCallback(char *topic, byte *payload, unsigned int length)
{
  DEBUG_PRINT("Received Message");
  DEBUG_PRINT(topic);
  DEBUG_PRINT((char *)payload);
  DEBUG_PRINT(length);
  DEBUG_PRINT("-------");

  if (strcmp(topic, MQTT_TOPIC_SUBSCRIBE_POWER) == 0)
  {
    if (memcmp(payload, "1", length) == 0 || memcmp(payload, "fold", length) == 0 || memcmp(payload, "FOLD", length) == 0)
    {
      state = FOLD;
      DEBUG_PRINT("Going to FOLD");
    }
    else if (memcmp(payload, "0", length) == 0 || memcmp(payload, "unfold", length) == 0 || memcmp(payload, "UNFOLD", length) == 0)
    {
      state = UNFOLD;
      DEBUG_PRINT("Going to UNFOLD");
    }
    else
    {
      DEBUG_PRINT("Unknown Message");
      client.publish(MQTT_TOPIC_STATUS_STATE, "UnknownMessage");
    }
  }
  // handle message arrived
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  DEBUG_PRINT("Initializing Firmware...");

  // Stepper Motor initialization
  DEBUG_PRINT("Stepper Motor Initialization");
  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(LOW);
  ///////////////////////////////////////////////////////////////////

  // MQTT Initialization
  DEBUG_PRINT("MQTT Initialization");
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(MQTTCallback);
  ///////////////////////////////////////////////////////////////////

  // WiFi Initialization
  DEBUG_PRINT("WiFi Initialization");
  DEBUG_PRINT(WiFi.isConnected());
  ///////////////////////////////////////////////////////////////////

  // Attach Interrupt to reed sensor
  DEBUG_PRINT("Reed Initialization");
  rotation_count = 0;
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), InterruptHandler, FALLING);
  ///////////////////////////////////////////////////////////////////

  // State Machine Initialization
  DEBUG_PRINT("State Machine Initialization");
  state = WAIT_FOR_COMMAND;
  full_extend_rotation_count = 15;
  full_stowed_rotation_count = 0;
  rotor_position = 0;

  ///////////////////////////////////////////////////////////////////
}

void loop()
{
  // Main State Machine
  DEBUG_PRINT("in th loop");
  DEBUG_PRINT(state);

  switch (state)
  {
  case UNFOLD:
    DEBUG_PRINT("In the UNFOLD routine");
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
    DEBUG_PRINT("In the FOLD routine");
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
    DEBUG_PRINT("In the ROTATE routine");
    rotor_position = rotation_count;
    if (rotor_position - direction * target_position <= 0)
    {
      // If the current position reached the target, stop the motor
      stepper.stop();
      stepper.disable();
      // Go back to WAIT state
      state = WAIT_FOR_COMMAND;
      mqtt_update_period = 300000L;

      if (rotor_position == full_extend_rotation_count)
      {
        strncpy(status, "Extended", sizeof(status));
      }
      else if (rotor_position == full_stowed_rotation_count)
      {
        strncpy(status, "Folded", sizeof(status));
      }
      else
      {
        sprintf(status, "%d", rotation_count);
      }
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
      // Reset the previous position
      previous_position = rotation_count;
    }

    strncpy(status, "InMotion", sizeof(status));
    mqtt_update_period = 1000L; // Increase the update period while in motion
    // TODO need to add the motor loop to program main loop  -- DONE?
    break;

  case WAIT_FOR_COMMAND:
    DEBUG_PRINT("Waiting for command");
    // TODO
    // TODO provide check to fold only when unfolded and vice versa

    // If we are not connected to Wifi -> connect
    // TODO add timer to stop it running too often
    DEBUG_PRINT(WiFi.status());
    if (WiFi.status() != WL_CONNECTED)
    {
      state = CONNECT_WIFI;
      break;
    }

    // If we are not connected to MQTT -> connect
    // TODO add timer to stop it running too often
    if (!client.connected())
    {
      state = CONNECT_MQTT;
      break;
    }
    break;
  case ERROR:
    DEBUG_PRINT("Error occurred");
    // TODO
    break;

  case CONNECT_WIFI:
    // We start by connecting to a WiFi network
    DEBUG_PRINT("In the WiFi setup routine");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      DEBUG_PRINT("Waiting for WiFi...");
    }

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

    // Attempt to connect
    if (client.connect(MQTT_DEV_ID, MQTT_USER, MQTT_PASS))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(MQTT_TOPIC_STATUS_STATE, "CONNECTED");
      // ... and resubscribe
      client.subscribe(MQTT_TOPIC_SUBSCRIBE_WILDCARD);
      DEBUG_PRINT("Subscribing to:");
      DEBUG_PRINT(MQTT_TOPIC_SUBSCRIBE_WILDCARD);
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
  DEBUG_PRINT("Periodic function calls");

  unsigned wait_time_micros = stepper.nextAction();
  // 0 wait time indicates the motor has stopped
  if (wait_time_micros <= 0)
  {
    stepper.disable(); // comment out to keep motor powered
    // delay(1000);
  }
  // (optional) execute other code if we have enough time
  if (wait_time_micros > 100)
  {
    // other code here
  }

  if (millis() - mqtt_time_variable > mqtt_update_period)
  {
    client.publish(MQTT_TOPIC_STATUS_STATE, status);
    mqtt_time_variable = millis();
  }
#ifdef DEBUG
  delay(2000);
#endif
  /////////////////////////////////////////////////////////////////////////////
}
