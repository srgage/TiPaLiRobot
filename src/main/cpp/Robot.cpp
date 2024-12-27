// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("TiPaLi");
  m_statePub = table->GetStringTopic("State").Publish();
  m_tiltPub = table->GetDoubleTopic("TiltAngle").Publish();
  m_panPub = table->GetDoubleTopic("PanAngle").Publish();
  m_linearPub = table->GetDoubleTopic("LinearDistance").Publish();

  m_statePub.Set("null");
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_statePub.Set("null");
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_statePub.Set("null");
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_state = STANDBY;
  m_statePub.Set("STANDBY");
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  double leftHorizAxis = m_container.m_controller.GetRawAxis(0); //pan
  double leftVertAxis = m_container.m_controller.GetRawAxis(1); //tilt
  double rightVertAxis = m_container.m_controller.GetRawAxis(3); //linear
  
  bool aButton = m_aDebouncer.Calculate(m_container.m_controller.GetRawButton(0));
  bool bButton = m_bDebouncer.Calculate(m_container.m_controller.GetRawButton(1));
  bool xButton = m_xDebouncer.Calculate(m_container.m_controller.GetRawButton(2));
  bool yButton = m_yDebouncer.Calculate(m_container.m_controller.GetRawButton(3));

  bool aEvent = aButton && !m_lastAButton;
  bool bEvent = bButton && !m_lastBButton;
  bool xEvent = xButton && !m_lastXButton;
  bool yEvent = yButton && !m_lastYButton;

  m_lastAButton = aButton;
  m_lastBButton = bButton;
  m_lastXButton = xButton;
  m_lastYButton = yButton;

  switch(m_state) {
    case STANDBY:
      if (yEvent) {
        m_state = LINEAR_ADJUST;
        m_statePub.Set("LINEAR_ADJUST");
        m_container.m_linear.StartAdjustment();
      }
      else if (xEvent) {
        m_state = PREPARE;
        m_statePub.Set("PREPARE");
        m_position = kStartDistance;
        double tilt = CalcTilt(kStartDistance);
        m_container.m_linear.SetDistance(kStartDistance);
        m_container.m_pan.SetAngle(0.0, false);
        m_container.m_tilt.SetAngle(tilt);
      }
      break;

    case LINEAR_ADJUST:
      if (aEvent) {
        m_container.m_linear.EndAdjustment(kStartDistance);
        m_state = TILT_ADJUST;
        m_statePub.Set("TILT_ADJUST");
        m_container.m_tilt.StartAdjustment();
      }
      else {
        m_container.m_linear.SetSpeed(rightVertAxis);
      }
      break;

    case TILT_ADJUST:
      if (aEvent) {
        m_container.m_tilt.EndAdjustment();
        m_state = LOADING;
        m_statePub.Set("LOADING");
      }
      else {
        m_container.m_tilt.SetSpeed(leftVertAxis);
      }
      break;

    case LOADING:
      if (aEvent) {
        m_state = PAN_ADJUST;
        m_statePub.Set("PAN_ADJUST");
        m_container.m_pan.StartAdjustment();
      }
      break;

    case PAN_ADJUST:
      if (aEvent) {
        m_container.m_pan.EndAdjustment();
        m_state = STANDBY;
        m_statePub.Set("STANDBY");
      }
      else {
        m_container.m_pan.SetSpeed(leftHorizAxis);
      }
      break;

    case PREPARE:
      if (m_container.m_linear.CheckGoal() && m_container.m_pan.CheckGoal() && m_container.m_tilt.CheckGoal()) {
        m_state = ROTATE_COLLECT;
        m_statePub.Set("ROTATE_COLLECT");
        m_container.m_pan.SetAngle(360.0, true);
      }
      break;

    case ROTATE_COLLECT:
      if (m_container.m_pan.CheckGoal()) {
        m_position += kStepDistance;
        if (m_position > kEndDistance) {
          m_state = STANDBY;
          m_statePub.Set("STANDBY");
        }
        else {
          double tilt = CalcTilt(m_position);
          m_container.m_linear.SetDistance(m_position);
          m_container.m_pan.SetAngle(0.0, false);
          m_container.m_tilt.SetAngle(tilt);
          m_state = PREPARE;
          m_statePub.Set("STANDBY");
        }
      }
      break;
  }
  m_tiltPub.Set(m_container.m_tilt.GetAngle());
  m_panPub.Set(m_container.m_pan.GetAngle());
  m_linearPub.Set(m_container.m_linear.GetDistance());
}

double Robot::CalcTilt(double distance) {
  return atan2(kCameraElevation, distance) * 180.0 / std::numbers::pi;
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
