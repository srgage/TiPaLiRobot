#pragma once

#include <numbers>

#include <frc/xrp/XRPMotor.h>
#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
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
class Pan : public frc2::SubsystemBase {
 public:
  Pan();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angle the commanded angle
   */

  void SetAngle(double angle, bool slow); //Automatic in deg

  void SetSpeed(double speed); //Manual

  double GetAngle();

  bool CheckGoal();

  void StartAdjustment();
  void EndAdjustment();
  void CancelAdjustment();

  void Init();

 private:
 /*
  XRP motors are PWM (pulse width modulation), so "volts" here are always -1.0 to 1.0
  no load motor speed is 90rpm, so no load pan speed is 360 * 90 / (5 * 60) = 108 deg_per_s
  kP is "volts" per degree of error, so 0.1 is reasonable
 */

  static constexpr double kGearRatio = 5.0;
  static constexpr int kCountsPerMotorRevolution = 585;
  static constexpr units::degrees_per_second_t kMaxVelocity = 45_deg_per_s;
  static constexpr units::degrees_per_second_t kSlowVelocity = 10_deg_per_s; //Special - 5
  static constexpr units::degrees_per_second_squared_t kMaxAcceleration = 5_deg_per_s_sq;

  static constexpr double kP = 0.05; // 0.1
  static constexpr double kI = 0.01; // 0.03
  static constexpr double kD = 0.0; // 0.0005

  static constexpr units::volt_t kS = 0.0_V; //minimum voltage to move motor 0.2
  static constexpr auto kV = 0.0_V / 108_deg_per_s;
  static constexpr auto kA = 0.0_V / 108_deg_per_s_sq;

  static constexpr units::degree_t kTolerancePos = 1.0_deg;
  static constexpr units::degrees_per_second_t kToleranceVel = 0.5_deg_per_s;

  static constexpr units::second_t kDt = 20_ms;

  frc::XRPMotor m_panMotor{1}; //Special
  frc::Encoder m_panEncoder{6, 7}; //Special

  frc::TrapezoidProfile<units::degrees>::Constraints m_fastConstraints{kMaxVelocity, kMaxAcceleration}; //Special
  frc::TrapezoidProfile<units::degrees>::Constraints m_slowConstraints{kSlowVelocity, kMaxAcceleration}; //Special

  frc::ProfiledPIDController<units::degrees> m_controller{kP, kI, kD, m_fastConstraints, kDt};
  frc::SimpleMotorFeedforward<units::degrees> m_feedforward{kS, kV, kA};

  enum SubSystemState m_state;
  double m_holdAngle;

  nt::StringPublisher m_statePub;
  nt::DoublePublisher m_backPub;
  nt::DoublePublisher m_setPointVelPub;
  nt::DoublePublisher m_setPointPosPub;
};