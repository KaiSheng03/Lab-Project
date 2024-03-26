#include <SPI.h>
#include <TimedAction.h>

//#include "ModbusMaster.h"
#include "ProgramConstant.h"
#include "Variable.h"

// Modbus network
//ModbusMaster ModbusNetwork1(&Serial1, 115200, ModbusWriteNotReadPin);

// Vector control
//uint16_t Payload[10] = { 0 };

uint32_t starting_time = 0;
bool starting = true;

// bool write_complete = false;
// uint32_t write_complete_millis = 0;

void LED_Blink_Task_Code()
{
    if (!LED_Light)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        LED_Light = true;
    }

    else
    {
        digitalWrite(LED_BUILTIN, LOW);
        LED_Light = false;
    }
}

void DebugMessageTaskCode()
{
    if (DebugMessage != "")
    {
        Serial.println(DebugMessage);
        DebugMessage = "";
    }
}

void PS4_Repeat_Init_Code()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(100000, LSBFIRST, SPI_MODE3));
    // pinMode(SPI_MISO, INPUT_PULLUP);
    // pinMode(SlaveAck, INPUT_PULLUP);
    // pinMode(SPI_CLK, OUTPUT); // configure ports
    // pinMode(SPI_MOSI, OUTPUT);
    // pinMode(SlaveSelect, OUTPUT);
    //digitalWrite(SlaveSelect, HIGH);
    PS2_SPI_SEND(Enter_Config, 5);
    PS2_SPI_SEND(Turn_ON_Analog_Mode, 9);
    PS2_SPI_SEND(Exit_Config, 9);
    PS2_SPI_SEND(ReadAllData, BufferSize);
    if (SPI_Packet[2] != 0x5A)
    {
        USB_Detected = false;
        DebugMessage = F("\nNo controller found, check wiring");
        PS4_Repeat_Init_Task.enable();
    }
    else
    {
        USB_Detected = true;
        DebugMessage = F("Found Controller, configured successful");
        PS4_Repeat_Init_Task.disable();
    }
}

void Input()
{
    PS2_SPI_SEND(ReadAllData, BufferSize);
    if (SPI_Packet[3] ^ 0xFF)
    {
        if (~SPI_Packet[3] & DPAD_UP)
            UP_Pressed = 1;
        if (~SPI_Packet[3] & DPAD_DOWN)
            DOWN_Pressed = 1;
        if (~SPI_Packet[3] & DPAD_LEFT)
            LEFT_Pressed = 1;
        if (~SPI_Packet[3] & DPAD_RIGHT)
            RIGHT_Pressed = 1;
    }
    if (SPI_Packet[4] ^ 0xFF)
    {
        if (~SPI_Packet[4] & L1_PRESSED)
            L1_Pressed = 1;
        if (~SPI_Packet[4] & L2_PRESSED)
            L2_Pressed = 1;
        if (~SPI_Packet[4] & R1_PRESSED)
            R1_Pressed = 1;
        if (~SPI_Packet[4] & R2_PRESSED)
            R2_Pressed = 1;
        if (~SPI_Packet[4] & GREEN_TRIANGLE_PRESSED)
            TRIANGLE_Pressed = 1;
        if (~SPI_Packet[4] & RED_CIRCLE_PRESSED)
            CIRCLE_Pressed = 1;
        if (~SPI_Packet[4] & BLUE_CROSS_PRESSED)
            CROSS_Pressed = 1;
        if (~SPI_Packet[4] & PINK_SQUARE_PRESSED)
            SQUARE_Pressed = 1;
    }
}

void ClearButtonStatus()
{
    UP_Pressed = false;
    RIGHT_Pressed = false;
    DOWN_Pressed = false;
    LEFT_Pressed = false;
    L1_Pressed = false;
    R1_Pressed = false;
    SQUARE_Pressed = false;
    CIRCLE_Pressed = false;
    TRIANGLE_Pressed = false;
    CROSS_Pressed = false;
    L2_Pressed = false;
    R2_Pressed = false;
}

void Process()
{
    switch (OperatingState)
    {
    case USB_Detect_Hold:
        if (USB_Detected = true)
        {
            OperatingState = WaitStart;
        }
        else
            DebugMessage = F("Detecting PS4 from USB Host");
        break;

    case WaitStart:
        if (CROSS_Pressed)
        {
            OperatingState = CheckActionToDo;
            InputTask.reset();
            ProcessTask.reset();
            Serial.println("\nCross pressed, start receiving command...");
            delay(1000);
        }
        ClearButtonStatus();
        break;

    case CheckActionToDo:
        if (CROSS_Pressed)
        {
            if (ButtonState != SouthEast)
            {
                ButtonState = SouthEast;
                Serial.println("SouthEast");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = HIGH;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = HIGH;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;

            }
            ClearButtonStatus();
        }
        else if (CIRCLE_Pressed)
        {
            if (ButtonState != NorthEast)
            {
                ButtonState = NorthEast;
                Serial.println("NorthEast");
                Front_Left_Dir_1 = HIGH;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = HIGH;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (SQUARE_Pressed)
        {
            if (ButtonState != SouthWest)
            {
                ButtonState = SouthWest;
                Serial.println("SouthWest");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = HIGH;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = HIGH;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (TRIANGLE_Pressed)
        {
            if (ButtonState != NorthWest)
            {
                ButtonState = NorthWest;
                Serial.println("NorthWest");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = HIGH;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = HIGH;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (UP_Pressed)
        {
            if (ButtonState != Forward)
            {
                ButtonState = Forward;
                Serial.println("Moveforward");
                Front_Left_Dir_1 = HIGH;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = HIGH;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = HIGH;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = HIGH;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (DOWN_Pressed)
        {
            if (ButtonState != Backward)
            {
                ButtonState = Backward;
                Serial.println("MoveBackward");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = HIGH;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = HIGH;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = HIGH;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = HIGH;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (LEFT_Pressed)
        {
            if (ButtonState != Left)
            {
                ButtonState = Left;
                Serial.println("Moveleft");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = HIGH;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = HIGH;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = HIGH;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = HIGH;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (RIGHT_Pressed)
        {
            if (ButtonState != Right)
            {
                ButtonState = Right;
                Serial.println("MoveRight");
                Front_Left_Dir_1 = HIGH;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = HIGH;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = HIGH;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = HIGH;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (L1_Pressed)
        {
            if (ButtonState != RotateAntiClockwise)
            {
                ButtonState = RotateAntiClockwise;
                Serial.println("RotateCounterClockwise");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = HIGH;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = HIGH;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = HIGH;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = HIGH;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (R1_Pressed)
        {
            if (ButtonState != RotateClockwise)
            {
                ButtonState = RotateClockwise;
                Serial.println("RotateClockwise");
                Front_Left_Dir_1 = HIGH;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = HIGH;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = HIGH;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = HIGH;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        else if (L2_Pressed)
        {
            if (ButtonState != Stop)
            {
                ButtonState = Stop;
                Serial.println("Stop");
                Front_Left_Dir_1 = LOW;
                Front_Left_Dir_2 = LOW;
                Front_Left_PWM = SPEED;

                Front_Right_Dir_1 = LOW;
                Front_Right_Dir_2 = LOW;
                Front_Right_PWM = SPEED;

                Rear_Left_Dir_1 = LOW;
                Rear_Left_Dir_2 = LOW;
                Rear_Left_PWM = SPEED;

                Rear_Right_Dir_1 = LOW;
                Rear_Right_Dir_2 = LOW;
                Rear_Right_PWM = SPEED;
            }
            ClearButtonStatus();
        }
        break;

    default:
        OperatingState = WaitStart;
        // Serial.println("\nWarning : Serious Error Occur, Entering Unexpected state");
        ClearButtonStatus();
    }
}

void PS2_SPI_SEND(byte* Data, uint8_t DataSize)
{
    uint8_t i = 0;
    digitalWrite(SlaveSelect, LOW);
    while (i < DataSize)
    {
        SPI_Packet[i] = SPI.transfer(Data[i]);
        delayMicroseconds(16);
        i = i + 1;
    }
    digitalWrite(SlaveSelect, HIGH);
}

void Sync_Basic_Task()
{
    // The following code is purposely to control when each task is being execute
    // Without the following code, program can still run, but programmer have no idea which Task will be execute first.
    LED_Blink_Task.reset();
    // delay(3);
    InputTask.reset(); // Input Task Run from 10ms from this point onwards
    delay(3); // Process Task Shall Execute 3ms after Input Task
    ProcessTask.reset(); // Process Task Run from 10ms from this point onwards
    //delay(3); // Output Task Shall Execute 3ms after Process Task
    //MotorControllerNetworkTask.reset(); // Output Task Run from 10ms from this point onwards
    // The above code is purposely to control when each task is being execute
}

void setup()
{
    OperatingState = USB_Detect_Hold;
    USB_Detected = false;

    pinMode(SPI_MISO, INPUT_PULLUP);
    pinMode(SlaveAck, INPUT_PULLUP);
    pinMode(SPI_CLK, OUTPUT); // configure ports
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SlaveSelect, OUTPUT);

    pinMode(FORWARD_LEFT_DIR_1, OUTPUT);
    pinMode(FORWARD_LEFT_DIR_2, OUTPUT);
    pinMode(FORWARD_LEFT_PWM, OUTPUT);

    pinMode(FORWARD_RIGHT_DIR_1, OUTPUT);
    pinMode(FORWARD_RIGHT_DIR_2, OUTPUT);
    pinMode(FORWARD_RIGHT_PWM, OUTPUT);  
  
    pinMode(REAR_LEFT_DIR_1, OUTPUT);
    pinMode(REAR_LEFT_DIR_2, OUTPUT);
    pinMode(REAR_LEFT_PWM, OUTPUT);
    
    pinMode(REAR_RIGHT_DIR_1, OUTPUT);
    pinMode(REAR_RIGHT_DIR_2, OUTPUT);
    pinMode(REAR_RIGHT_PWM, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    //ModbusNetwork1.Start();

    DebugMessageTask.reset();
    Sync_Basic_Task();

    InputTask.enable();
    ProcessTask.enable();
    LED_Blink_Task.enable();
    ButtonState = Stop;
    DebugMessageTask.enable();

    PS4_Repeat_Init_Code();
}

void loop()
{
    PS4_Repeat_Init_Task.check();
    if (USB_Detected)
    {
        //ModbusNetwork1.MasterListen();
        LED_Blink_Task.check();
        //ModbusNetwork1.MasterListen();
        InputTask.check();
        //ModbusNetwork1.MasterListen();
        ProcessTask.check();
        //ModbusNetwork1.MasterListen();
        //MotorControllerNetworkTask.check();
        //ModbusNetwork1.MasterListen();
    }
    else
    {
        Sync_Basic_Task();
    }
    DebugMessageTask.check();
}

void Output(){
  analogWrite(FORWARD_LEFT_PWM, SPEED);
  analogWrite(FORWARD_RIGHT_PWM, SPEED);
  analogWrite(REAR_LEFT_PWM, SPEED);
  analogWrite(REAR_RIGHT_PWM, SPEED);

  digitalWrite(FORWARD_LEFT_DIR_1, Front_Left_Dir_1);
  digitalWrite(FORWARD_LEFT_DIR_2, Front_Left_Dir_2);

  digitalWrite(FORWARD_RIGHT_DIR_1, Front_Right_Dir_1);
  digitalWrite(FORWARD_RIGHT_DIR_2, Front_Right_Dir_2);

  digitalWrite(REAR_LEFT_DIR_1, Rear_Left_Dir_1);
  digitalWrite(REAR_LEFT_DIR_2, Rear_Left_Dir_2);
  
  digitalWrite(REAR_RIGHT_DIR_1, Rear_Right_Dir_1);
  digitalWrite(REAR_RIGHT_DIR_2, Rear_Right_Dir_2);
}
