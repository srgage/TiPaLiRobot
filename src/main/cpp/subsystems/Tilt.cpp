#include "subsystems/Tilt.h"

Tilt::Tilt() {
  m_tiltEncoder.SetDistancePerPulse(360.0 / (kCountsPerMotorRevolution * kGearRatio));
  m_tiltEncoder.Reset();
  m_holdAngle = 0.0;
  m_state = HOLD;
}

void Tilt::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        SetSpeed(0.0);
        m_holdAngle = m_controller.GetGoal().position.value();
      }
      else {
        m_tiltMotor.Set(
          m_controller.Calculate(units::degree_t{m_tiltEncoder.GetDistance()}) + 
          m_feedforward.Calculate(m_controller.GetSetpoint().velocity).value()
        );
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
}
void Tilt::EndAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_holdAngle = 0.0;
    m_tiltEncoder.Reset();
  }
}