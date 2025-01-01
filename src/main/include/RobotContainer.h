// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>

#include "subsystems/Linear.h"
#include "subsystems/Pan.h"
#include "subsystems/Tilt.h"
 
class RobotContainer {
 public:
  RobotContainer();
  //Assumes controller on channel 0
  frc::Joystick m_controller{0};
  //Robot subsystems
  Linear m_linear;
  Pan m_pan;
  Tilt m_tilt;
};
