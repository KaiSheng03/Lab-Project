#define FORWARD_LEFT_DIR_1 8
#define FORWARD_LEFT_DIR_2 7
#define FORWARD_LEFT_PWM 6

#define FORWARD_RIGHT_DIR_1 A4
#define FORWARD_RIGHT_DIR_2 A3
#define FORWARD_RIGHT_PWM 9

#define REAR_LEFT_DIR_1 4
#define REAR_LEFT_DIR_2 A5
#define REAR_LEFT_PWM 5

#define REAR_RIGHT_DIR_1 A1
#define REAR_RIGHT_DIR_2 A2
#define REAR_RIGHT_PWM 3

#define SERVO_PIN 2

#define ULTRASONIC_TRIG_PIN 1
#define ULTRASONIC_ECHO_PIN A0

#define BUZZER_PIN 0

#define SPEED 255

enum{
  UP = 16,
  LEFT = 128,
  RIGHT = 32,
  DOWN = 64,
  SQUARE = 32768,
  TRIANGLE = 4096,
  CIRCLE = 8192,
  CROSS = 16384,
  L1 = 1024,
  L2 = 256,
  L3 = 2,
  R1 = 2048,
  R2 = 512,
  R3 = 4
};

enum{
  Forward,
  Backward,
  Left,
  Right,
  SouthEast,
  SouthWest,
  NorthEast,
  NorthWest,
  Stop,
  RotateClockwise,
  RotateAntiClockwise
};