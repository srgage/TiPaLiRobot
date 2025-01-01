#include "subsystems/Pan.h"

Pan::Pan() {
  m_panEncoder.SetDistancePerPulse(360.0 / (kCountsPerMotorRevolution * kGearRatio));
  m_panEncoder.Reset();
  m_holdAngle = 0.0;
  m_state = HOLD;
}

void Pan::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        m_panMotor.Set(0.0);
        m_holdAngle = m_controller.GetGoal().position.value();
      }
      else {
        m_panMotor.Set(
          m_controller.Calculate(units::degree_t{m_panEncoder.GetDistance()}) + 
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

void Pan::SetAngle(double angle, bool slow) { //Special
  if (m_state != AUTO_MOVING) {
    if (slow) {
      m_controller.SetConstraints(m_slowConstraints);
    }
    else {
      m_controller.SetConstraints(m_fastConstraints);
    }

    m_state = AUTO_MOVING;
    m_controller.SetGoal(units::degree_t{angle});
  }
}

void Pan::SetSpeed(double speed){
  if (m_state == MANUAL_MOVING) {
    m_panMotor.Set(speed);
  }
}

double Pan::GetAngle() {
  return m_panEncoder.GetDistance();
}

bool Pan::CheckGoal() {
  return m_state == HOLD;
}

void Pan::StartAdjustment() {
  m_state = MANUAL_MOVING;
}
void Pan::EndAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_holdAngle = 0.0;
    m_panEncoder.Reset();
  }
}