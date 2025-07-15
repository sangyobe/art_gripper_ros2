
#ifndef DEVICE_MAN_HH
#define DEVICE_MAN_HH

#include "sysData.h"
#include "ecMaster.h"
#include "ecSlaves.h"

int initDevice(SysData *sysData);
int closeDevice(SysData *sysData);
int writeDevice(RobotData *robotData);
int readDevice(RobotData *robotData);
void CheckKeyboardInput(SysData *sysData);

#endif