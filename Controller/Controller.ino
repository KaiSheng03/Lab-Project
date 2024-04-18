/*******************************************************************************
 * This file is part of PsxNewLib.                                             *
 *                                                                             *
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * PsxNewLib is free software: you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * PsxNewLib is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with PsxNewLib. If not, see http://www.gnu.org/licenses.              *
 *******************************************************************************
 *
 * This sketch will dump to serial whatever is done on a PSX controller. It is
 * an excellent way to test that all buttons/sticks are read correctly.
 *
 * It's missing support for analog buttons, that will come in the future.
 *
 * This example drives the controller through the hardware SPI port, so pins are
 * fixed and depend on the board/microcontroller being used. For instance, on an
 * Arduino Uno connections must be as follows:
 *
 * CMD: Pin 11
 * DATA: Pin 12
 * CLK: Pin 13
 *
 * Any pin can be used for ATTN, but please note that most 8-bit AVRs require
 * the HW SPI SS pin to be kept as an output for HW SPI to be in master mode, so
 * using that pin for ATTN is a natural choice. On the Uno this would be pin 10.
 *
 * It also works perfectly on OpenPSX2AmigaPadAdapter boards (as it's basically
 * a modified Uno).
 *
 * There is another similar one using a bitbanged protocol implementation that
 * can be used on any pins/board.
 */

/*
  UP: 16
  LEFT: 128
  RIGHT: 32
  DOWN: 64
  SQUARE: 32768
  TRIANGLE: 4096
  CIRCLE: 8192
  CROSS: 16384
  L1: 1024
  L2: 256
  L3: 2
  R1: 2048
  R2: 512
  R3: 4
*/

#include <DigitalIO.h>
#include <PsxControllerHwSpi.h>
#include "Variable.h"
#include <avr/pgmspace.h>
#include <TimedAction.h>
#include "ProgramConstant.h"

typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper*>(s)

// This can be changed freely but please see above
const byte PIN_PS2_ATT = 10;

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
  buttonSelectName,
  buttonL3Name,
  buttonR3Name,
  buttonStartName,
  buttonUpName,
  buttonRightName,
  buttonDownName,
  buttonLeftName,
  buttonL2Name,
  buttonR2Name,
  buttonL1Name,
  buttonR1Name,
  buttonTriangleName,
  buttonCircleName,
  buttonCrossName,
  buttonSquareName
};

void ClearButtonStatus() {
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

void Input() {
  switch (buttonState) {
    case UP:
      UP_Pressed = true;
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
      break;

    case LEFT:
      LEFT_Pressed = true;
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
      break;

    case RIGHT:
      RIGHT_Pressed = true;
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
      break;

    case DOWN:
      DOWN_Pressed = true;
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
      break;

    case SQUARE:
      SQUARE_Pressed = true;
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
      break;

    case TRIANGLE:
      TRIANGLE_Pressed = true;
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
      break;

    case CIRCLE:
      CIRCLE_Pressed = true;
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
      break;

    case CROSS:
      CROSS_Pressed = true;
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
      break;

    case L1:
      L1_Pressed = true;
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
      break;

    case L2:
      L2_Pressed = true;
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
      break;

    case L3:
      L3_Pressed = true;
      break;

    case R1:
      R1_Pressed = true;
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
      break;

    case R2:
      R2_Pressed = true;
      break;

    case R3:
      R3_Pressed = true;
      break;
  }
  ClearButtonStatus();
}

void Process() {
}

void Output() {
  analogWrite(FORWARD_LEFT_PWM, Front_Left_PWM);
  analogWrite(FORWARD_RIGHT_PWM, Front_Right_PWM);
  analogWrite(REAR_LEFT_PWM, Rear_Left_PWM);
  analogWrite(REAR_RIGHT_PWM, Rear_Right_PWM);

  digitalWrite(FORWARD_LEFT_DIR_1, Front_Left_Dir_1);
  digitalWrite(FORWARD_LEFT_DIR_2, Front_Left_Dir_2);

  digitalWrite(FORWARD_RIGHT_DIR_1, Front_Right_Dir_1);
  digitalWrite(FORWARD_RIGHT_DIR_2, Front_Right_Dir_2);

  digitalWrite(REAR_LEFT_DIR_1, Rear_Left_Dir_1);
  digitalWrite(REAR_LEFT_DIR_2, Rear_Left_Dir_2);

  digitalWrite(REAR_RIGHT_DIR_1, Rear_Right_Dir_1);
  digitalWrite(REAR_RIGHT_DIR_2, Rear_Right_Dir_2);
}

byte psxButtonToIndex(PsxButtons psxButtons) {
  byte i;

  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }

    psxButtons >>= 1U;
  }

  return i;
}

FlashStr getButtonName(PsxButtons psxButton) {
  FlashStr ret = F("");

  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
    ret = PSTR_TO_F(bName);
  }

  return ret;
}

void dumpButtons(PsxButtons psxButtons) {
  static PsxButtons lastB = 0;

  if (psxButtons != lastB) {
    lastB = psxButtons;  // Save it before we alter it
    buttonState = lastB;
    Serial.print(F("Pressed: "));

    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);
      if (b < PSX_BUTTONS_NO) {
        PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
        Serial.print(PSTR_TO_F(bName));
      }

      psxButtons &= ~(1 << b);

      if (psxButtons != 0) {
        Serial.print(F(", "));
      }
    }

    Serial.println();
  }
}

void dumpAnalog(const char* str, const byte x, const byte y) {
  Serial.print(str);
  Serial.print(F(" analog: x = "));
  Serial.print(x);
  Serial.print(F(", y = "));
  Serial.println(y);

  // if (x == 128 && y == 128) {
  //   Front_Left_PWM = 0;
  //   Front_Right_PWM = 0;
  //   Rear_Left_PWM = 0;
  //   Rear_Right_PWM = 0;
  //   Front_Left_Dir_1 = LOW;
  //   Front_Left_Dir_2 = LOW;

  //   Front_Right_Dir_1 = LOW;
  //   Front_Right_Dir_2 = LOW;

  //   Rear_Left_Dir_1 = LOW;
  //   Rear_Left_Dir_2 = LOW;

  //   Rear_Right_Dir_1 = LOW;
  //   Rear_Right_Dir_2 = LOW;
  // } else {
  //   if (forwardSpeed > 0) {
  //     Front_Left_Dir_1 = HIGH;
  //     Front_Left_Dir_2 = LOW;

  //     Front_Right_Dir_1 = LOW;
  //     Front_Right_Dir_2 = HIGH;

  //     Rear_Left_Dir_1 = HIGH;
  //     Rear_Left_Dir_2 = LOW;

  //     Rear_Right_Dir_1 = LOW;
  //     Rear_Right_Dir_2 = HIGH;
  //   } else {
  //     Front_Left_Dir_1 = LOW;
  //     Front_Left_Dir_2 = HIGH;

  //     Front_Right_Dir_1 = HIGH;
  //     Front_Right_Dir_2 = LOW;

  //     Rear_Left_Dir_1 = LOW;
  //     Rear_Left_Dir_2 = HIGH;

  //     Rear_Right_Dir_1 = HIGH;
  //     Rear_Right_Dir_2 = LOW;
  //   }
  //   if (lateralSpeed > 0) {
  //     Front_Left_Dir_1 = HIGH;
  //     Front_Left_Dir_2 = LOW;

  //     Front_Right_Dir_1 = HIGH;
  //     Front_Right_Dir_2 = LOW;

  //     Rear_Left_Dir_1 = LOW;
  //     Rear_Left_Dir_2 = HIGH;

  //     Rear_Right_Dir_1 = LOW;
  //     Rear_Right_Dir_2 = HIGH;
  //   } else {
  //     Front_Left_Dir_1 = LOW;
  //     Front_Left_Dir_2 = HIGH;

  //     Front_Right_Dir_1 = LOW;
  //     Front_Right_Dir_2 = HIGH;

  //     Rear_Left_Dir_1 = HIGH;
  //     Rear_Left_Dir_2 = LOW;

  //     Rear_Right_Dir_1 = HIGH;
  //     Rear_Right_Dir_2 = LOW;
  //   }
  // }
}

const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
  ctrlTypeUnknown,
  ctrlTypeDualShock,
  ctrlTypeDsWireless,
  ctrlTypeGuitHero,
  ctrlTypeOutOfBounds
};

PsxControllerHwSpi<PIN_PS2_ATT> psx;

boolean haveController = false;

void setup() {
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

  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);

  Serial.begin(115200);
  Serial.println(F("Ready!"));
}

void loop() {
  static byte slx, sly, srx, sry;

  fastDigitalWrite(PIN_HAVECONTROLLER, haveController);

  if (!haveController) {
    if (psx.begin()) {
      Serial.println(F("Controller found!"));
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype) : PSCTRL_MAX])));
        Serial.print(F("Controller Type is: "));
        Serial.println(PSTR_TO_F(cname));

        if (!psx.enableAnalogSticks()) {
          Serial.println(F("Cannot enable analog sticks"));
        }

        //~ if (!psx.setAnalogMode (false)) {
        //~ Serial.println (F("Cannot disable analog mode"));
        //~ }
        //~ delay (10);

        if (!psx.enableAnalogButtons()) {
          Serial.println(F("Cannot enable analog buttons"));
        }

        if (!psx.exitConfigMode()) {
          Serial.println(F("Cannot exit config mode"));
        }
      }

      haveController = true;
    }
  } else {
    if (!psx.read()) {
      Serial.println(F("Controller lost :("));
      haveController = false;
    } else {
      InputTask.check();
      ProcessTask.check();
      OutputTask.check();

      fastDigitalWrite(PIN_BUTTONPRESS, !!psx.getButtonWord());
      dumpButtons(psx.getButtonWord());

      byte lx, ly;
      psx.getLeftAnalog(lx, ly);
      if (lx != slx || ly != sly) {
        dumpAnalog("Left", lx, ly);
        slx = lx;
        sly = ly;
      }

      byte rx, ry;
      psx.getRightAnalog(rx, ry);
      if (rx != srx || ry != sry) {
        dumpAnalog("Right", rx, ry);
        srx = rx;
        sry = ry;
      }
      forwardSpeed = 128 - sly;
      lateralSpeed = slx - 128;
      centerRPM = srx - 128;
      
      Front_Left_PWM = (1.0/0.58) * (0.7071 * lateralSpeed + 0.7071 * forwardSpeed + 0.58 * centerRPM);
      Front_Right_PWM = (1.0/0.58) * (0.7071 * lateralSpeed + -0.7071 * forwardSpeed + 0.58 * centerRPM);
      Rear_Left_PWM = (1.0/0.58) * (-0.7071 * lateralSpeed + 0.7071 * forwardSpeed + 0.58 * centerRPM);
      Rear_Right_PWM = (1.0/0.58) * (-0.7071 * lateralSpeed + -0.7071 * forwardSpeed + 0.58 * centerRPM);

      if(Front_Left_PWM > 0){
        Front_Left_Dir_1 = HIGH;
        Front_Left_Dir_2 = LOW;
      }
      else{
        Front_Left_Dir_1 = LOW;
        Front_Left_Dir_2 = HIGH;
      }

      if(Front_Right_PWM > 0){
        Front_Right_Dir_1 = HIGH;
        Front_Right_Dir_2 = LOW;
      }
      else{
        Front_Right_Dir_1 = LOW;
        Front_Right_Dir_2 = HIGH;
      }

      if(Rear_Left_PWM > 0){
        Rear_Left_Dir_1 = HIGH;
        Rear_Left_Dir_2 = LOW;
      }
      else{
        Rear_Left_Dir_1 = LOW;
        Rear_Left_Dir_2 = HIGH;
      }

      if(Rear_Right_PWM > 0){
        Rear_Right_Dir_1 = HIGH;
        Rear_Right_Dir_2 = LOW;
      }
      else{
        Rear_Right_Dir_1 = LOW;
        Rear_Right_Dir_2 = HIGH;
      }
      Serial.print("Front Left: ");
      Serial.print(Front_Left_PWM);
      Serial.print(" Front Right: ");
      Serial.print(Front_Right_PWM);
      Serial.print(" Rear Left: ");
      Serial.print(Rear_Left_PWM);
      Serial.print(" Rear Right: ");
      Serial.println(Rear_Right_PWM);
      Front_Left_PWM = abs(Front_Left_PWM);
      Front_Right_PWM = abs(Front_Right_PWM);
      Rear_Left_PWM = abs(Rear_Left_PWM);
      Rear_Right_PWM = abs(Rear_Right_PWM);

    }
  }
  delay(1000 / 60);
}
