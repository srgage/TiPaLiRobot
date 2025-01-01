#include "subsystems/Tilt.h"

#include <math.h>

Tilt::Tilt() {
  m_tiltEncoder.SetDistancePerPulse(360.0 / (kCountsPerMotorRevolution * kGearRatio));
  m_tiltEncoder.Reset();
  m_holdAngle = 0.0;
  m_state = HOLD;
  m_statePub.Set("HOLD");

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("Tilt");
  m_statePub = table->GetStringTopic("State").Publish();
  m_forwardPub = table->GetDoubleTopic("Feedforward").Publish();
  m_backPub = table->GetDoubleTopic("Feedback").Publish();
  m_setPointVelPub = table->GetDoubleTopic("SetPointVel").Publish();
  m_setPointPosPub = table->GetDoubleTopic("SetPointPos").Publish();

  m_forwardPub.Set(0.0);
  m_backPub.Set(0.0);
  m_setPointVelPub.Set(0.0);
  m_setPointPosPub.Set(0.0);
}

void Tilt::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        m_statePub.Set("HOLD");
        m_tiltMotor.Set(0.0);
        m_holdAngle = m_controller.GetGoal().position.value();
      }
      else {
        double back = m_controller.Calculate(units::degree_t{m_tiltEncoder.GetDistance()});
        auto setpoint = m_controller.GetSetpoint();
        double forward = m_feedforward.Calculate(setpoint.velocity).value();
        //forward = signum(velocity)*kS.value();
        m_tiltMotor.Set(forward + back);
        m_forwardPub.Set(forward);
        m_backPub.Set(back);
        m_setPointVelPub.Set(setpoint.velocity.value());
        m_setPointPosPub.Set(setpoint.position.value());
      }
      break;
    
    case MANUAL_MOVING:
      break;
    
    case HOLD:
      //assume static friction will hold motor in place
      break;
  }
}

void Tilt::SetAngle(double angle) {
  if (m_state != AUTO_MOVING) {
    m_state = AUTO_MOVING;
    m_statePub.Set("AUTO_MOVING");
    m_controller.Reset(units::degree_t{GetAngle()});
    m_controller.SetTolerance(kTolerancePos, kToleranceVel);
    m_controller.SetGoal(units::degree_t{angle});
  }
}

void Tilt::SetSpeed(double speed){
  if (m_state == MANUAL_MOVING) {
    m_tiltMotor.Set(speed);
  }
}

double Tilt::GetAngle() {
  return m_tiltEncoder.GetDistance();
}

bool Tilt::CheckGoal() {
  return m_state == HOLD;
}

void Tilt::StartAdjustment() {
  m_state = MANUAL_MOVING;
  m_statePub.Set("MANUAL_MOVING");
}
void Tilt::EndAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_statePub.Set("HOLD");
    m_holdAngle = 0.0;
    m_tiltEncoder.Reset();
  }
}