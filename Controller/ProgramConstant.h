// PS2 Address
#define PS4_SHARE 0x01
#define PS4_UNKNOWN1 0x02
#define PS4_UNKNOWN2 0x04
#define PS4_OPTION 0x08
#define DPAD_UP 0x10
#define DPAD_RIGHT 0x20
#define DPAD_DOWN 0x40
#define DPAD_LEFT 0x80
#define L2_PRESSED 0x01
#define L1_PRESSED 0x04
#define R1_PRESSED 0x08
#define R2_PRESSED 0x02
#define GREEN_TRIANGLE_PRESSED 0x10
#define RED_CIRCLE_PRESSED 0x20
#define BLUE_CROSS_PRESSED 0x40
#define PINK_SQUARE_PRESSED 0x80

#define SPI_CLK 52 //  13
#define SPI_MISO 50 //  12
#define SPI_MOSI 51 //  11
#define SlaveSelect 53 //  10
#define SlaveAck 3
#define ModbusWriteNotReadPin 5
#define BufferSize 9

// Motor Pin
#define FORWARD_LEFT_DIR_1 1
#define FORWARD_LEFT_DIR_2 2
#define FORWARD_LEFT_PWM 10

#define FORWARD_RIGHT_DIR_1 3 
#define FORWARD_RIGHT_DIR_2 4
#define FORWARD_RIGHT_PWM 11

#define REAR_LEFT_DIR_1 5
#define REAR_LEFT_DIR_2 6
#define REAR_LEFT_PWM 12

#define REAR_RIGHT_DIR_1 7
#define REAR_RIGHT_DIR_2 8
#define REAR_RIGHT_PWM 13

// Slave
/*
#define SLAVE_BROADCAST_ADDRESS 0x63
#define SLAVE_FRONT_LEFT_ADDRESS 0x01
#define SLAVE_FRONT_RIGHT_ADDRESS 0x02
#define SLAVE_REAR_LEFT_ADDRESS 0x03
#define SLAVE_REAR_RIGHT_ADDRESS 0x04
*/
#define SPEED 255

enum{
  USB_Detect_Hold,
  WaitStart,
  CheckActionToDo
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

uint32_t Front_Left_Dir_1;
uint32_t Front_Left_Dir_2;
uint32_t Front_Left_PWM;

uint32_t Front_Right_Dir_1;
uint32_t Front_Right_Dir_2;
uint32_t Front_Right_PWM;
  
uint32_t Rear_Left_Dir_1;
uint32_t Rear_Left_Dir_2;
uint32_t Rear_Left_PWM;
  
uint32_t Rear_Right_Dir_1;
uint32_t Rear_Right_Dir_2;
uint32_t Rear_Right_PWM;