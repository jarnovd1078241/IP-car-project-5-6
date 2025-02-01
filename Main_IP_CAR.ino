/*
  ESP32 Robot Car - Enhanced 5-Step Forced Sequence with Dynamic Turn Direction and Obstacle Detection Delay:
    1) BACKUP
    2) TURN_AWAY (Dynamic based on sensor input)
    3) DRIVE_FORWARD_1
    4) FINAL_TURN
    5) DRIVE_FORWARD_2
    => DONE => back to MANUAL

  Additional Features:
    - 2-second confirmation delay before switching to AVOID state when an obstacle is detected.
    - Manual override is completely disabled during the 2-second delay and AVOID state.
  
  While performing each step, we do minimal checks:
    - If all sensors are blocked => forcedPivotOrBackup => remain in same step
    - If dangerously close => forcedPivotOrBackup => remain in same step

  Times are TOTALLY up to you to tune for your hardware.
*/

#include <Arduino.h>
#include <Bluepad32.h>

// -----------------------------
// Pin Definitions
// -----------------------------
#define DIR_PIN_RIGHT    23
#define BRAKE_PIN_RIGHT  22
#define STOP_PIN_RIGHT   21

#define DIR_PIN_LEFT     19
#define BRAKE_PIN_LEFT   18
#define STOP_PIN_LEFT    17

// Lights for sensors
#define leftlight        5
#define middlelight      16
#define rightlight       2

// Ultrasonic pins
#define TRIG_PIN_FRONT   14
#define ECHO_PIN_FRONT   12
#define TRIG_PIN_LEFT    27
#define ECHO_PIN_LEFT    26
#define TRIG_PIN_RIGHT   15
#define ECHO_PIN_RIGHT   25

// -----------------------------
// Sensor Thresholds
// -----------------------------
#define STOP_THRESHOLD_CM       40   // Danger zone
#define CAUTION_THRESHOLD_CM    40   // "blocked" if < 40cm

// -----------------------------
// Step Durations (adjust!)
// -----------------------------
#define BACKUP_TIME_MS        800   // Step1
#define TURN_AWAY_TIME_MS     450   // Step2
#define FORWARD_1_TIME_MS     1500  // Step3
#define FINAL_TURN_TIME_MS    900   // Step4
#define FORWARD_2_TIME_MS     1200  // Step5

// forcedPivotOrBackup if all blocked
#define FORCED_BACKUP_MS      500
#define FORCED_TURN_MS        500

// Task intervals, joystick threshold
#define MAINLOOP_DELAY_MS         20
#define SENSOR_TASK_DELAY_MS      50
#define CONTROLLER_TASK_DELAY_MS  20
#define JOY_THRESHOLD             300

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];
bool controllerConnected = false;

// Sensor data
volatile long distanceFront = 999;
volatile long distanceLeft  = 999;
volatile long distanceRight = 999;

// Joystick data
volatile int16_t joyLeftY  = 0;
volatile int16_t joyRightX = 0;

// Lights
volatile bool frontLightOn = false;
volatile bool leftLightOn  = false;
volatile bool rightLightOn = false;

//----------------------------------
// Robot States
//----------------------------------
enum RobotState {
  MANUAL,
  AVOID
};
volatile RobotState currentState = MANUAL;

//----------------------------------
// AVOID sub-states (5 forced steps + done)
//----------------------------------
enum AvoidSubState {
  AVOID_BACKUP,
  AVOID_TURN_AWAY,
  AVOID_DRIVE_FORWARD_1,
  AVOID_FINAL_TURN,
  AVOID_DRIVE_FORWARD_2,
  AVOID_DONE
};
AvoidSubState avoidSubState = AVOID_BACKUP;

// Additional enum for turn direction
enum TurnDirection {
  LEFT,
  RIGHT
};
volatile TurnDirection turnDirection = LEFT; // Default

// We'll track the time we enter a sub-state
unsigned long subStateStartTime = 0;
bool subStateFirstRun = true; // to do "init" on first pass

// -----------------------------
// Obstacle Detection Delay Variables
// -----------------------------
unsigned long obstacleDetectedStartTime = 0;
bool obstaclePending = false;
const unsigned long OBSTACLE_DELAY_MS = 2000; // 2 seconden delay

//--------------------------------------------------------
// Forward Declarations
//--------------------------------------------------------
void sensorTask(void* param);
void controllerTask(void* param);
void mainLoopTask(void* param);

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
bool isAnyControllerConnected();

void initializeMotorPins();
void stopAllMotors();
void driveForward();
void driveBackward();
void turnLeftInPlace();
void turnRightInPlace();
void stopMotors();

long readUltrasonicDistance(int trigPin,int echoPin);

bool isMultipleBlocked();
bool isDangerClose();

void forcedPivotOrBackup();
void setSubState(AvoidSubState s);
void doAvoidLogic();

//--------------------------------------------------------
// setup
//--------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Verbeterde 5-stappen Forced Approach met Obstakeldetectie Vertraging ===");

  initializeMotorPins();
  stopMotors();

  pinMode(TRIG_PIN_FRONT, OUTPUT); digitalWrite(TRIG_PIN_FRONT, LOW);
  pinMode(ECHO_PIN_FRONT, INPUT);

  pinMode(TRIG_PIN_LEFT, OUTPUT);  digitalWrite(TRIG_PIN_LEFT, LOW);
  pinMode(ECHO_PIN_LEFT, INPUT);

  pinMode(TRIG_PIN_RIGHT, OUTPUT); digitalWrite(TRIG_PIN_RIGHT, LOW);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  pinMode(leftlight, OUTPUT);
  pinMode(middlelight, OUTPUT);
  pinMode(rightlight, OUTPUT);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Initialize controller array
  for(int i=0; i<BP32_MAX_CONTROLLERS; i++) {
    myControllers[i] = nullptr;
  }

  // Start FreeRTOS tasks
  xTaskCreatePinnedToCore(sensorTask,     "sensorTask",     2048, NULL,1,NULL,0);
  xTaskCreatePinnedToCore(controllerTask, "controllerTask", 2048, NULL,1,NULL,1);
  xTaskCreatePinnedToCore(mainLoopTask,   "mainLoopTask",   4096, NULL,2,NULL,1);
}

void loop(){
  // Not used. We rely on FreeRTOS tasks.
  delay(1000);
}

//--------------------------------------------------------
// Bluetooth callbacks
//--------------------------------------------------------
void onConnectedController(ControllerPtr ctl){
  for(int i=0; i<BP32_MAX_CONTROLLERS; i++){
    if(myControllers[i]==nullptr){
      myControllers[i]=ctl;
      controllerConnected=true;
      Serial.printf("Controller verbonden idx=%d\n",i);
      break;
    }
  }
  controllerConnected = isAnyControllerConnected();
}

void onDisconnectedController(ControllerPtr ctl){
  for(int i=0;i<BP32_MAX_CONTROLLERS;i++){
    if(myControllers[i]==ctl){
      myControllers[i]=nullptr;
      Serial.printf("Controller verbroken idx=%d\n", i);
      break;
    }
  }
  controllerConnected = isAnyControllerConnected();
}

bool isAnyControllerConnected(){
  for(int i=0; i<BP32_MAX_CONTROLLERS;i++){
    if(myControllers[i]!=nullptr) return true;
  }
  return false;
}

//--------------------------------------------------------
// sensorTask
//--------------------------------------------------------
void sensorTask(void* param) {
  for(;;){
    long f = readUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    long l = readUltrasonicDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    long r = readUltrasonicDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    distanceFront = f;
    distanceLeft  = l;
    distanceRight = r;

    frontLightOn  = (f < CAUTION_THRESHOLD_CM);
    leftLightOn   = (l < CAUTION_THRESHOLD_CM);
    rightLightOn  = (r < CAUTION_THRESHOLD_CM);

    digitalWrite(middlelight, frontLightOn ? HIGH : LOW);
    digitalWrite(leftlight,   leftLightOn  ? HIGH : LOW);
    digitalWrite(rightlight,  rightLightOn ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_DELAY_MS));
  }
}

//--------------------------------------------------------
// controllerTask
//--------------------------------------------------------
void controllerTask(void* param){
  for(;;){
    BP32.update();

    if(!controllerConnected){
      joyLeftY = 0; 
      joyRightX = 0;
    } else {
      // Read from the first connected gamepad
      for(int i=0; i<BP32_MAX_CONTROLLERS; i++){
        ControllerPtr ctl = myControllers[i];
        if(!ctl) continue;
        if(ctl->isConnected() && ctl->isGamepad()){
          // Only process joystick inputs if in MANUAL state and obstaclePending == false
          if(currentState == MANUAL && !obstaclePending){
            joyLeftY  = ctl->axisY();
            joyRightX = ctl->axisRX();
          }
          else{
            // Ignore joystick inputs during obstaclePending or AVOID
            joyLeftY  = 0;
            joyRightX = 0;
          }
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROLLER_TASK_DELAY_MS));
  }
}

//--------------------------------------------------------
// mainLoopTask - High-level FSM
//--------------------------------------------------------
void mainLoopTask(void* param){
  for(;;){
    switch(currentState){
      case MANUAL:
      {
        // Check for obstacle detection
        if(distanceFront < CAUTION_THRESHOLD_CM
        || distanceLeft  < CAUTION_THRESHOLD_CM
        || distanceRight < CAUTION_THRESHOLD_CM)
        {
          if(!obstaclePending){
            // First time obstacle detected, start timer
            obstacleDetectedStartTime = millis();
            obstaclePending = true;
            stopMotors();
            
            // Gedetailleerde logging voor welke sensor(s) het obstakel detecteert
            Serial.print("[MANUAL] Obstakel gedetecteerd door: ");
            bool anySensor = false;
            if (distanceFront < CAUTION_THRESHOLD_CM) {
              Serial.print("Front ");
              anySensor = true;
            }
            if (distanceLeft < CAUTION_THRESHOLD_CM) {
              Serial.print("Links ");
              anySensor = true;
            }
            if (distanceRight < CAUTION_THRESHOLD_CM) {
              Serial.print("Rechts ");
              anySensor = true;
            }
            if (!anySensor) {
              Serial.print("Onbekende sensor ");
            }
            Serial.println("Wachten op bevestiging...");
          }
          else{
            // Check if obstacle has been present for at least 2 seconds
            if((millis() - obstacleDetectedStartTime) >= OBSTACLE_DELAY_MS){
              // Confirmed obstacle, switch to AVOID
              setSubState(AVOID_BACKUP); // Step 1
              currentState = AVOID;
              Serial.println("[MANUAL] => AVOID (Obstakel bevestigd)");
              obstaclePending = false; // Reset
            }
            else{
              // Optioneel: Log resterende tijd voor bevestiging
              unsigned long remaining = OBSTACLE_DELAY_MS - (millis() - obstacleDetectedStartTime);
              Serial.printf("[MANUAL] Wachten op bevestiging... (%lu ms resterend)\n", remaining);
            }
          }
        }
        else{
          // No obstacle detected
          if(obstaclePending){
            Serial.println("[MANUAL] Obstakel verwijderd voor bevestiging.");
            obstaclePending = false;
          }

          // Normal joystick drive only if not in obstaclePending
          if(!obstaclePending){
            float normLeftY  = -(joyLeftY / 32767.0f);
            float normRightX =  (joyRightX / 32767.0f);

            // Small deadzone
            if(fabs(normLeftY) < (JOY_THRESHOLD / 32767.0f)) normLeftY = 0;
            if(fabs(normRightX) < (JOY_THRESHOLD / 32767.0f)) normRightX = 0;

            float leftVal  = normLeftY + normRightX;
            float rightVal = normLeftY - normRightX;

            // Left motor control
            if(leftVal > 0) {
              digitalWrite(DIR_PIN_LEFT, LOW);
              digitalWrite(BRAKE_PIN_LEFT, LOW);
              digitalWrite(STOP_PIN_LEFT, HIGH);
            }
            else if(leftVal < 0) {
              digitalWrite(DIR_PIN_LEFT, HIGH);
              digitalWrite(BRAKE_PIN_LEFT, LOW);
              digitalWrite(STOP_PIN_LEFT, HIGH);
            }
            else {
              digitalWrite(BRAKE_PIN_LEFT, HIGH);
              digitalWrite(STOP_PIN_LEFT, LOW);
            }

            // Right motor control
            if(rightVal > 0) {
              digitalWrite(DIR_PIN_RIGHT, LOW);
              digitalWrite(BRAKE_PIN_RIGHT, LOW);
              digitalWrite(STOP_PIN_RIGHT, HIGH);
            }
            else if(rightVal < 0){
              digitalWrite(DIR_PIN_RIGHT, HIGH);
              digitalWrite(BRAKE_PIN_RIGHT, LOW);
              digitalWrite(STOP_PIN_RIGHT, HIGH);
            }
            else {
              digitalWrite(BRAKE_PIN_RIGHT, HIGH);
              digitalWrite(STOP_PIN_RIGHT, LOW);
            }
          }
        }
      }
      break;

      case AVOID:
      {
        doAvoidLogic();
      }
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(MAINLOOP_DELAY_MS));
  }
}

//--------------------------------------------------------
// doAvoidLogic - the 5 forced steps (plus done)
//--------------------------------------------------------
void doAvoidLogic(){
  // No manual override allowed

  // If all sensors blocked => forced pivot or backup
  if(isMultipleBlocked()){
    Serial.println("[doAvoidLogic] Alle sensoren zijn geblokkeerd.");
    forcedPivotOrBackup();
    return; // Remain in same sub-state
  }

  // If dangerously close => forced pivot as well
  if(isDangerClose()){
    // Prevent continuous obstacle collision
    Serial.println("[doAvoidLogic] Obstakel is gevaarlijk dichtbij.");
    forcedPivotOrBackup();
    return;
  }

  unsigned long now = millis();

  switch(avoidSubState){
    // STEP 1: BACKUP
    case AVOID_BACKUP:
    {
      if(subStateFirstRun){
        Serial.println("[AVOID_BACKUP] => Begin met achteruit rijden");
        subStateStartTime = now; // Initialize start time
        subStateFirstRun = false;
      }
      driveBackward();
      if((now - subStateStartTime) >= BACKUP_TIME_MS){
        stopMotors();
        Serial.println("[AVOID_BACKUP] Klaar => TURN_AWAY");
        setSubState(AVOID_TURN_AWAY);
      }
    }
    break;

    // STEP 2: TURN_AWAY
    // Turns left if right sensor detected obstacle,
    // Turns right if left sensor detected obstacle,
    // Defaults based on sensor distances if both or neither.
    case AVOID_TURN_AWAY:
    {
      if(subStateFirstRun){
        // Decide turn direction based on sensor
        if(distanceRight < CAUTION_THRESHOLD_CM && distanceLeft >= CAUTION_THRESHOLD_CM){
          turnDirection = LEFT;
        }
        else if(distanceLeft < CAUTION_THRESHOLD_CM && distanceRight >= CAUTION_THRESHOLD_CM){
          turnDirection = RIGHT;
        }
        else {
          // Both sensors are blocked or neither specifically
          // Decide based on which side is closer
          if(distanceRight < distanceLeft){
            turnDirection = LEFT;
          }
          else{
            turnDirection = RIGHT;
          }
        }

        Serial.printf("[AVOID_TURN_AWAY] => Draaien naar %s gedwongen...\n", turnDirection == LEFT ? "links" : "rechts");
        subStateStartTime = now; // Initialize start time
        subStateFirstRun = false;
      }

      if(turnDirection == LEFT){
        turnLeftInPlace();
      }
      else{
        turnRightInPlace();
      }

      if((now - subStateStartTime) >= TURN_AWAY_TIME_MS){
        stopMotors();
        Serial.println("[AVOID_TURN_AWAY] Klaar => DRIVE_FORWARD_1");
        setSubState(AVOID_DRIVE_FORWARD_1);
      }
    }
    break;

    // STEP 3: DRIVE_FORWARD_1
    case AVOID_DRIVE_FORWARD_1:
    {
      if(subStateFirstRun){
        Serial.println("[AVOID_DRIVE_FORWARD_1] => Gedwongen vooruit rijden...");
        subStateStartTime = now; // Initialize start time
        subStateFirstRun = false;
      }
      driveForward();
      if((now - subStateStartTime) >= FORWARD_1_TIME_MS){
        stopMotors();
        Serial.println("[AVOID_DRIVE_FORWARD_1] Klaar => FINAL_TURN");
        setSubState(AVOID_FINAL_TURN);
      }
    }
    break;

    // STEP 4: FINAL_TURN
    // For example, turn right for FINAL_TURN_TIME_MS
    case AVOID_FINAL_TURN:
    {
      if(subStateFirstRun){
        Serial.println("[AVOID_FINAL_TURN] => Eindeutige draai naar rechts gedwongen...");
        subStateStartTime = now; // Initialize start time
        subStateFirstRun = false;
      }
      turnRightInPlace();
      if((now - subStateStartTime) >= FINAL_TURN_TIME_MS){
        stopMotors();
        Serial.println("[AVOID_FINAL_TURN] Klaar => DRIVE_FORWARD_2");
        setSubState(AVOID_DRIVE_FORWARD_2);
      }
    }
    break;

    // STEP 5: DRIVE_FORWARD_2
    // Another forced forward
    case AVOID_DRIVE_FORWARD_2:
    {
      if(subStateFirstRun){
        Serial.println("[AVOID_DRIVE_FORWARD_2] => Laatste gedwongen vooruit beweging...");
        subStateStartTime = now; // Initialize start time
        subStateFirstRun = false;
      }
      driveForward();
      if((now - subStateStartTime) >= FORWARD_2_TIME_MS){
        stopMotors();
        Serial.println("[AVOID_DRIVE_FORWARD_2] => AVOID_DONE");
        setSubState(AVOID_DONE);
      }
    }
    break;

    // DONE => back to MANUAL
    case AVOID_DONE:
    {
      Serial.println("[AVOID_DONE] => Overschakelen naar MANUAL modus");
      currentState = MANUAL;
    }
    break;
  }
}

//--------------------------------------------------------
// setSubState
//--------------------------------------------------------
void setSubState(AvoidSubState s){
  avoidSubState = s;
  subStateStartTime = millis();
  subStateFirstRun = true;
}

//--------------------------------------------------------
// forcedPivotOrBackup => short backup+turn if all blocked or dangerously close
//--------------------------------------------------------
void forcedPivotOrBackup(){
  Serial.println("[forcedPivotOrBackup] Uitvoeren gedwongen achteruit en draaien...");

  // 1) Backup
  unsigned long t0 = millis();
  unsigned long endTime = t0 + FORCED_BACKUP_MS;
  while(millis() < endTime){
    driveBackward();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  stopMotors();
  Serial.println("[forcedPivotOrBackup] Achteruit rijden voltooid.");

  // 2) Pivot (turn right, for example)
  t0 = millis();
  endTime = t0 + FORCED_TURN_MS;
  while(millis() < endTime){
    turnRightInPlace(); 
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  stopMotors();
  Serial.println("[forcedPivotOrBackup] Draaien voltooid.");

  // Remain in the same sub-state to keep attempting the forced step
}

//--------------------------------------------------------
// Additional checks
//--------------------------------------------------------
bool isMultipleBlocked(){
  // front + left + right all < CAUTION
  return (distanceFront < CAUTION_THRESHOLD_CM
       && distanceLeft  < CAUTION_THRESHOLD_CM
       && distanceRight < CAUTION_THRESHOLD_CM);
}

bool isDangerClose(){
  // if ANY sensor < STOP_THRESHOLD => we consider it dangerously close
  return (distanceFront < STOP_THRESHOLD_CM
       || distanceLeft  < STOP_THRESHOLD_CM
       || distanceRight < STOP_THRESHOLD_CM);
}

//--------------------------------------------------------
// Motor Control
//--------------------------------------------------------
void initializeMotorPins(){
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(BRAKE_PIN_RIGHT, OUTPUT);
  pinMode(STOP_PIN_RIGHT, OUTPUT);

  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(BRAKE_PIN_LEFT, OUTPUT);
  pinMode(STOP_PIN_LEFT, OUTPUT);

  stopAllMotors();
}

void stopAllMotors(){
  digitalWrite(BRAKE_PIN_LEFT, HIGH);
  digitalWrite(STOP_PIN_LEFT, LOW);

  digitalWrite(BRAKE_PIN_RIGHT, HIGH);
  digitalWrite(STOP_PIN_RIGHT, LOW);
}

void driveForward(){
  digitalWrite(DIR_PIN_LEFT, LOW);
  digitalWrite(BRAKE_PIN_LEFT, LOW);
  digitalWrite(STOP_PIN_LEFT, HIGH);

  digitalWrite(DIR_PIN_RIGHT, LOW);
  digitalWrite(BRAKE_PIN_RIGHT, LOW);
  digitalWrite(STOP_PIN_RIGHT, HIGH);
}

void driveBackward(){
  digitalWrite(DIR_PIN_LEFT, HIGH);
  digitalWrite(BRAKE_PIN_LEFT, LOW);
  digitalWrite(STOP_PIN_LEFT, HIGH);

  digitalWrite(DIR_PIN_RIGHT, HIGH);
  digitalWrite(BRAKE_PIN_RIGHT, LOW);
  digitalWrite(STOP_PIN_RIGHT, HIGH);
}

void turnLeftInPlace(){
  // Left motor backward, right motor forward
  digitalWrite(DIR_PIN_LEFT, HIGH);
  digitalWrite(BRAKE_PIN_LEFT, LOW);
  digitalWrite(STOP_PIN_LEFT, HIGH);

  digitalWrite(DIR_PIN_RIGHT, LOW);
  digitalWrite(BRAKE_PIN_RIGHT, LOW);
  digitalWrite(STOP_PIN_RIGHT, HIGH);
}

void turnRightInPlace(){
  // Left motor forward, right motor backward
  digitalWrite(DIR_PIN_LEFT, LOW);
  digitalWrite(BRAKE_PIN_LEFT, LOW);
  digitalWrite(STOP_PIN_LEFT, HIGH);

  digitalWrite(DIR_PIN_RIGHT, HIGH);
  digitalWrite(BRAKE_PIN_RIGHT, LOW);
  digitalWrite(STOP_PIN_RIGHT, HIGH);
}

void stopMotors(){
  stopAllMotors();
}

//--------------------------------------------------------
// readUltrasonicDistance
//--------------------------------------------------------
long readUltrasonicDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if(duration == 0){
    return 999; // Treat as far away
  }
  return (duration * 0.034) / 2;
}
