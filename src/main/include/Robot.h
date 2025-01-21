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

  MANUAL_ADJUST,

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
  double CalcTilt(double groundDistance);
  double CalcSlopeDistance(double groundDistance);

 private:
  RobotContainer m_container;
  static constexpr double kCameraElevation = 1.0; //meters 
  static constexpr double kGroundStartDistance = 0.0; //meters
  static constexpr double kGroundEndDistance = 3.61;  // 3.67 is the physical limit (slope=3.805)
  static constexpr double kGroundStepDistance = 0.1; //0.05 meters

  // start before initial angle by 5 deg and end after final angle by same
  // perform 5 rotations to give the data collection more time
  static constexpr double kPanStartAngle = -5.0;
  static constexpr double kPanEndRotations = 2.0;
  static constexpr double kPanEndAngle = kPanEndRotations * 360.0 + 5.0;

  enum RobotState m_state;
  double m_groundPosition;
  double m_slopePosition;
  bool m_lastAButton = false;
  bool m_lastBButton = false;
  bool m_lastXButton = false;
  bool m_lastYButton = false;
  bool m_lastKillButton = false;

  nt::StringPublisher m_statePub;
  nt::DoublePublisher m_tiltPub;
  nt::DoublePublisher m_panPub;
  nt::DoublePublisher m_linearPub;
  nt::DoublePublisher m_slopePub;
};
