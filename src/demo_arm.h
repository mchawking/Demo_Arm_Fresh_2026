#pragma once
#include <stdint.h>

void    demoArmSetup();
void    demoArmLoop();
uint16_t getChannel(uint8_t ch);  // ch 1-16, returns 172-1811 (center 992)
bool    hasSignal();
bool    canBusReady();
float   getOdrivePositionSetpoint();
uint8_t getOdriveNodeId();
uint32_t getOdriveCanBaud();
