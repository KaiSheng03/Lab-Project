bool UP_Pressed = false;
bool LEFT_Pressed = false;
bool RIGHT_Pressed = false;
bool DOWN_Pressed = false;

bool L1_Pressed = false;
bool L2_Pressed = false;
bool R1_Pressed = false;
bool R2_Pressed = false;

bool TRIANGLE_Pressed = false;
bool SQUARE_Pressed = false;
bool CIRCLE_Pressed = false;
bool CROSS_Pressed = false;

bool USB_Detected = false;

bool LED_Light = false;

String DebugMessage = "";

void LED_Blink_Task_Code();
void DebugMessageTaskCode();
void PS4_Repeat_Init_Code();
void ClearButtonStatus();
void PS2_SPI_SEND(byte*, uint8_t);
void Sync_Basic_Task();
void Input();
void Process();
void Output();

TimedAction InputTask = TimedAction(1, Input);
TimedAction ProcessTask = TimedAction(1, Process);
TimedAction OutputTask = TimedAction(1, Output);
TimedAction DebugMessageTask = TimedAction(1, DebugMessageTaskCode);
TimedAction LED_Blink_Task = TimedAction(1, LED_Blink_Task_Code);
TimedAction PS4_Repeat_Init_Task = TimedAction(1, PS4_Repeat_Init_Code);

uint32_t OperatingState = 0;
uint32_t ButtonState = 0;

uint8_t SPI_Packet[BufferSize] = { 0 };
static byte ReadAllData[] = { 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static byte Enter_Config[] = { 0x01, 0x43, 0x00, 0x01, 0x00 };
static byte Turn_ON_Analog_Mode[] = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
static byte Maps_Motor[] = { 0x01, 0x4D, 0x00, 0x00, 0x01 };
static byte Exit_Config[] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };