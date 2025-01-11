// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include "RobotContainer.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include <math.h>

enum RobotState {
  STANDBY,

  LINEAR_ADJUST,
  TILT_ADJUST,
  LOADING,
  PAN_ADJUST,

  PREPARE,
  ROTATE_COLLECT
};

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  RobotContainer m_container;
  static constexpr double kCameraElevation = 1.0; //meters
  static constexpr double kStartDistance = 0.1; //meters
  static constexpr double kEndDistance = 0.51; //3.51 meters
  static constexpr double kStepDistance = 0.1; //0.05 meters
  enum RobotState m_state;
  double m_position;
  bool m_lastAButton = false;
  bool m_lastBButton = false;
  bool m_lastXButton = false;
  bool m_lastYButton = false;
  bool m_lastKillButton = false;
  double CalcTilt(double distance);

  nt::StringPublisher m_statePub;
  nt::DoublePublisher m_tiltPub;
  nt::DoublePublisher m_panPub;
  nt::DoublePublisher m_linearPub;
  nt::DoublePublisher m_slopePub;
};
