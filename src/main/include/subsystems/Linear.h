#pragma once

#include <numbers>

#include <frc/xrp/XRPMotor.h>
#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>

#include "common.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
class Linear : public frc2::SubsystemBase {
 public:
  Linear();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Set the current distance.
   *
   * @param distance the commanded distance
   */

  void SetDistance(double distance); //Automatic in meters

  void SetSpeed(double speed); //Manual

  double GetDistance();

  bool CheckGoal();

  void StartAdjustment();
  void EndAdjustment(double distance);
  void CancelAdjustment();

  void Init();


 private:
 /*
  XRP motors are PWM (pulse width modulation), so "volts" here are always -1.0 to 1.0
  no load motor speed is 90rpm, so no load linear speed is pi * 0.06 * 90 / 60 = 0.28 mps
  kP is "volts" per meter of error, so 0.1 per cm = 10 V / m
 */
  static constexpr double kWheelDiameter = 0.06; //meters
  static constexpr int kCountsPerMotorRevolution = 585;
  static constexpr units::meters_per_second_t kMaxVelocity = 0.1_mps; //0.2
  static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.025_mps_sq; //0.05

  static constexpr double kP = 10.0;
  static constexpr double kI = 10.0; //0.0
  static constexpr double kD = 0.0; //0.1
  static constexpr units::volt_t kS = 0.2_V; //minimum voltage to move motor
  static constexpr auto kV = 0.0_V / 0.28_mps;
  static constexpr auto kA = 0.0_V / 0.28_mps_sq;

  static constexpr units::meter_t kTolerancePos = 0.001_m;
  static constexpr units::meters_per_second_t kToleranceVel = 0.001_mps;
  
  static constexpr units::second_t kDt = 20_ms;

  frc::XRPMotor m_linearMotor{3}; //Special
  frc::Encoder m_linearEncoder{10, 11}; //Special

  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{kMaxVelocity, kMaxAcceleration};

  frc::ProfiledPIDController<units::meters> m_controller{kP, kI, kD, m_constraints, kDt};
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{kS, kV, kA};

  enum SubSystemState m_state;
  double m_holdDistance;

  double m_offset = 0.0;

  nt::StringPublisher m_statePub;
  nt::DoublePublisher m_backPub;
  nt::DoublePublisher m_setPointVelPub;
  nt::DoublePublisher m_setPointPosPub;
};