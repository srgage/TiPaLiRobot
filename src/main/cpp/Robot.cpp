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
  m_linearPub = table->GetDoubleTopic("GroundDistance").Publish();
  m_slopePub = table->GetDoubleTopic("SlopeDistance").Publish();

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
  m_container.m_linear.Init();
  m_container.m_pan.Init();
  m_container.m_tilt.Init();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  double leftHorizAxis = m_container.m_controller.GetRawAxis(0); //pan
  double leftVertAxis = m_container.m_controller.GetRawAxis(1); //tilt
  double rightVertAxis = m_container.m_controller.GetRawAxis(5); //linear

  bool killButton = m_container.m_controller.GetRawButton(5); //kill - Left Bumper
  bool killEvent = killButton && !m_lastKillButton;
  m_lastKillButton = killButton;
  
  bool aButton = m_container.m_controller.GetRawButton(1);
  bool bButton = m_container.m_controller.GetRawButton(2);
  bool xButton = m_container.m_controller.GetRawButton(3);
  bool yButton = m_container.m_controller.GetRawButton(4);

  bool aEvent = aButton && !m_lastAButton;
  bool bEvent = bButton && !m_lastBButton;
  bool xEvent = xButton && !m_lastXButton;
  bool yEvent = yButton && !m_lastYButton;

  m_lastAButton = aButton;
  m_lastBButton = bButton;
  m_lastXButton = xButton;
  m_lastYButton = yButton;

  if (killEvent) {
    m_state = STANDBY;
    m_statePub.Set("STANDBY");
    m_container.m_tilt.SetSpeed(0.0);
    m_container.m_pan.SetSpeed(0.0);
    m_container.m_linear.SetSpeed(0.0);
  }
  else {
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
          m_groundPosition = kGroundStartDistance;
          m_slopePosition = CalcSlopeDistance(kGroundStartDistance);
          double tilt = CalcTilt(kGroundStartDistance);
          m_container.m_linear.SetDistance(m_slopePosition);
          m_container.m_pan.SetAngle(kPanStartAngle, false);
          m_container.m_tilt.SetAngle(tilt);
        }
        else if (bEvent) {
          m_state = MANUAL_ADJUST;
          m_statePub.Set("MANUAL_ADJUST");
          m_container.m_tilt.StartAdjustment();
          m_container.m_pan.StartAdjustment();
          m_container.m_linear.StartAdjustment();
        }
        break;

      case MANUAL_ADJUST:
        if (bEvent) {
          m_state = STANDBY;
          m_statePub.Set("STANDBY");
          m_container.m_tilt.CancelAdjustment();
          m_container.m_pan.CancelAdjustment();
          m_container.m_linear.CancelAdjustment();
        }
        else if (aEvent) {
          m_container.m_linear.EndAdjustment(CalcSlopeDistance(kGroundStartDistance));
          m_container.m_linear.StartAdjustment();
        }
        else {
          m_container.m_tilt.SetSpeed(leftVertAxis*0.25);
          m_container.m_pan.SetSpeed(leftHorizAxis*0.25);
          m_container.m_linear.SetSpeed(rightVertAxis);
        }
        break;

      case LINEAR_ADJUST:
        if (aEvent) {
          m_container.m_linear.EndAdjustment(CalcSlopeDistance(kGroundStartDistance));
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
          m_container.m_tilt.SetSpeed(leftVertAxis*0.25);
        }
        break;

      case LOADING:
        if (aEvent) {
          m_state = PAN_ADJUST;
          m_statePub.Set("PAN_ADJUST");
          m_container.m_pan.StartAdjustment();
          m_container.m_tilt.SetAngle(90.0);
        }
        break;

      case PAN_ADJUST:
        if (aEvent) {
          m_container.m_pan.EndAdjustment();
          m_state = STANDBY;
          m_statePub.Set("STANDBY");
        }
        else {
          m_container.m_pan.SetSpeed(leftHorizAxis*0.25);
        }
        break;

      case PREPARE:
        if (m_container.m_linear.CheckGoal() && m_container.m_pan.CheckGoal() && m_container.m_tilt.CheckGoal()) {
          m_state = ROTATE_COLLECT;
          m_statePub.Set("ROTATE_COLLECT");
          m_container.m_pan.SetAngle(kPanEndAngle, true);
        }
        break;

      case ROTATE_COLLECT:
        if (m_container.m_pan.CheckGoal()) {
          m_groundPosition += kGroundStepDistance;
          if (m_groundPosition > kGroundEndDistance) {
            m_state = STANDBY;
            m_statePub.Set("STANDBY");
          }
          else {
            // normalize pan angle to -180..180
            m_container.m_pan.NormalizeAngle();
            m_slopePosition = CalcSlopeDistance(m_groundPosition);
            double tilt = CalcTilt(m_groundPosition);
            m_container.m_linear.SetDistance(m_slopePosition);
            m_container.m_pan.SetAngle(kPanStartAngle, false);
            m_container.m_tilt.SetAngle(tilt);
            m_state = PREPARE;
            m_statePub.Set("PREPARE");
          }
        }
        break;
    }
  }
  m_tiltPub.Set(m_container.m_tilt.GetAngle());
  m_panPub.Set(m_container.m_pan.GetAngle());
  m_slopePub.Set(m_container.m_linear.GetDistance());
  m_linearPub.Set(sqrt(abs(pow(m_container.m_linear.GetDistance(), 2) - pow(kCameraElevation, 2))));
}

double Robot::CalcTilt(double groundDistance) {
  return atan2(kCameraElevation, groundDistance) * 180.0 / std::numbers::pi;
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

double Robot::CalcSlopeDistance(double groundDistance) {
  return sqrt(pow(groundDistance, 2) + pow(kCameraElevation, 2));
}