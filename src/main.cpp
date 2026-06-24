/*
 * Arduino code to control solar follower actuator.
 * Move motors until both sensors reach the same level.
 */

/*
 * Hardware configuration
 */
// Analog inputs for light sensors
/*
 *        Bleu
 * Rouge       Blanc
 *        Noir
 */
#define LEFT_SENSOR_PIN A3
#define RIGHT_SENSOR_PIN A1
#define TOP_SENSOR_PIN A4
#define BOTTOM_SENSOR_PIN A2

// Motors output
#define BOTTOM_TOP_MOTOR_PWN_PIN 3
#define BOTTOM_TOP_MOTOR_DIR_PIN 12
#define LEFT_RIGHT_MOTOR_PWN_PIN 11
#define LEFT_RIGHT_MOTOR_DIR_PIN 13

/*
 * Project configuration
 */
// Minimum level at which control should start (0-1024)
#define MIN_SENSOR_LEVEL_OPERATION 1012
// Minimum diff between sensors values to enable control
#define MIN_SENSOR_DIFF 3

// Long run from evening to morning (ms)
#define LONG_RUN_TIME 10000
// Minimum diff between sensors values to stop long run
#define LONG_RUN_MIN_SENSOR_DIFF 4

// Change motor direction (0 or 1)
#define LEFT_RIGHT_DIRECTION 1
#define BOTTOM_TOP_DIRECTION 1

// Motor speeds (0 to 255)
#define BOTTOM_TOP_MOTOR_SPEED 255
#define LEFT_RIGHT_MOTOR_SPEED 255

// Sleep time between controls (s)
#define SLEEP_TIME 20

// Assert sleep time: time before 2 controls (ms)
#define ASSERV_SLEEP_TIME 200

// Security shutdown timeout (ms)
#define SECURITY_SHUTDOWN_TIMEOUT 240000
// Noise margin for wrong direction detection
#define NOISE_MARGIN 3
#define DEBUG

void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  pinMode(BOTTOM_TOP_MOTOR_PWN_PIN, OUTPUT);
  pinMode(LEFT_RIGHT_MOTOR_PWN_PIN, OUTPUT);
  pinMode(BOTTOM_TOP_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_RIGHT_MOTOR_DIR_PIN, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(BOTTOM_TOP_MOTOR_PWN_PIN, LOW);
  digitalWrite(LEFT_RIGHT_MOTOR_PWN_PIN, LOW);
  digitalWrite(BOTTOM_TOP_MOTOR_DIR_PIN, LOW);
  digitalWrite(LEFT_RIGHT_MOTOR_DIR_PIN, LOW);
}

struct SensorData {
  int time;
  int value;
};

bool controlMotor(int sensorPin1, int sensorPin2, int motorDirPin, int motorPwmPin, int direction, int motorSpeed, const char* debugTag) {
  int input1 = analogRead(sensorPin1);
  int input2 = analogRead(sensorPin2);
  int target = input1 - input2;

  #ifdef DEBUG
    Serial.print("Init ");
    Serial.print(debugTag);
    Serial.print(" Sensor1: ");
    Serial.print(input1);
    Serial.print(" Sensor2: ");
    Serial.println(input2);
  #endif

  if (abs(target) < MIN_SENSOR_DIFF) {
    #ifdef DEBUG
      Serial.println("Not moving motor due to MIN_SENSOR_DIFF not fulfilled");
    #endif

    return false;
  }

  if ((input1 >= MIN_SENSOR_LEVEL_OPERATION || input2 >= MIN_SENSOR_LEVEL_OPERATION) && target != 0) {
    // Set motor direction
    if (target < 0) {
      digitalWrite(motorDirPin, direction ? LOW : HIGH);
    } else {
      digitalWrite(motorDirPin, direction ? HIGH : LOW);
    }

    // Start motor
    #ifdef DEBUG
      Serial.print("Start ");
      Serial.print(debugTag);
      Serial.println(" motor");
    #endif
    analogWrite(motorPwmPin, motorSpeed);

    long runningTime = 0;
    SensorData targetValues[3] = {
        {0, abs(input1 - input2)},
        {-1, 0},
        {-1, 0}
    };

    do {
      delay(ASSERV_SLEEP_TIME);
      runningTime += ASSERV_SLEEP_TIME; // Not perfect but avoid overflow mess to handle with real time ...
      input1 = analogRead(sensorPin1);
      input2 = analogRead(sensorPin2);

      #ifdef DEBUG
        Serial.print(debugTag);
        Serial.print(" Sensor1: ");
        Serial.print(input1);
        Serial.print(" Sensor2: ");
        Serial.println(input2);
      #endif

      // Ensure diff between input1 and input2 if decreasing over time
      if (runningTime - targetValues[0].time >= 1000) {
        // Store value every second in 3 last values history
        targetValues[2] = targetValues[1];
        targetValues[1] = targetValues[0];
        targetValues[0] = {runningTime, abs(input1 - input2)};
      }
      if (targetValues[2].time >= 0 && targetValues[0].value - NOISE_MARGIN > targetValues[2].value) {
        #ifdef DEBUG
          Serial.println("Moving in wrong direction, STOP");
        #endif
        break;
      }

      // Check for security lock
      if (runningTime > SECURITY_SHUTDOWN_TIMEOUT) {
        analogWrite(LEFT_RIGHT_MOTOR_PWN_PIN, 0);
        analogWrite(BOTTOM_TOP_MOTOR_PWN_PIN, 0);
        while (true) {
          #ifdef DEBUG
            Serial.println("Locked in security mode");
          #endif

          delay(60000);
        }
      }

      // Long run stop
      if (runningTime > LONG_RUN_TIME && !(target < 0 && input1 + LONG_RUN_MIN_SENSOR_DIFF < input2 || target > 0 && input1 > input2 + LONG_RUN_MIN_SENSOR_DIFF)) {
        #ifdef DEBUG
          Serial.print("Stop (Long run)");
          Serial.print(debugTag);
          Serial.println(" motor");
        #endif

        break;
      }
    } while (
        // Stop if target is reached
        (target < 0 && input1 < input2 || target > 0 && input1 > input2)
        // Stop if both input are lower than minimum required level
        && (input1 > MIN_SENSOR_LEVEL_OPERATION || input2 > MIN_SENSOR_LEVEL_OPERATION));

    // Stop motor
    #ifdef DEBUG
      Serial.print("Stop ");
      Serial.print(debugTag);
      Serial.println(" motor");
    #endif
    analogWrite(motorPwmPin, 0);

    return true;
  }

  return false;
}

void loop() {
  bool hasMoved;
  int i = 0;

  do {
    hasMoved = false;
    i++;

    // Call the reusable function for Left/Right control
    hasMoved |= controlMotor(
        LEFT_SENSOR_PIN,
        RIGHT_SENSOR_PIN,
        LEFT_RIGHT_MOTOR_DIR_PIN,
        LEFT_RIGHT_MOTOR_PWN_PIN,
        LEFT_RIGHT_DIRECTION,
        LEFT_RIGHT_MOTOR_SPEED,
        "Left/Right");

    // Call the reusable function for Bottom/Top control
    hasMoved |= controlMotor(
        BOTTOM_SENSOR_PIN,
        TOP_SENSOR_PIN,
        BOTTOM_TOP_MOTOR_DIR_PIN,
        BOTTOM_TOP_MOTOR_PWN_PIN,
        BOTTOM_TOP_DIRECTION,
        BOTTOM_TOP_MOTOR_SPEED,
        "Bottom/Top");
  } while (hasMoved && i <= 2);

  delay(SLEEP_TIME * 1000);
}