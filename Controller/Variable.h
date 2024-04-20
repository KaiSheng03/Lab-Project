#include <TimedAction.h>
#include <Servo.h>

bool UP_Pressed = false;
bool LEFT_Pressed = false;
bool RIGHT_Pressed = false;
bool DOWN_Pressed = false;

bool L1_Pressed = false;
bool L2_Pressed = false;
bool L3_Pressed = false;
bool R1_Pressed = false;
bool R2_Pressed = false;
bool R3_Pressed = false;

bool TRIANGLE_Pressed = false;
bool SQUARE_Pressed = false;
bool CIRCLE_Pressed = false;
bool CROSS_Pressed = false;

bool lightOpen = false;

int32_t buttonState;
uint32_t currentState;

uint32_t Front_Left_Dir_1;
uint32_t Front_Left_Dir_2;
int32_t Front_Left_PWM;

uint32_t Front_Right_Dir_1;
uint32_t Front_Right_Dir_2;
int32_t Front_Right_PWM;
  
uint32_t Rear_Left_Dir_1;
uint32_t Rear_Left_Dir_2;
int32_t Rear_Left_PWM;
  
uint32_t Rear_Right_Dir_1;
uint32_t Rear_Right_Dir_2;
int32_t Rear_Right_PWM;

uint32_t servoStep;

uint32_t distance;

Servo cameraServo;

int lateralSpeed = 0;
int forwardSpeed = 0;
int centerRPM = 0;

void Input();
void Process();
void Output();
void Ultrasonic();
void ClearButtonStatus();

TimedAction InputTask = TimedAction(1, Input);
TimedAction ProcessTask = TimedAction(1, Process);
TimedAction OutputTask = TimedAction(1, Output);
TimedAction UltrasonicTask = TimedAction(1, Ultrasonic);