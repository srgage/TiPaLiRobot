#include "subsystems/Linear.h"

Linear::Linear() {
  m_linearEncoder.SetDistancePerPulse(kWheelDiameter * std::numbers::pi / kCountsPerMotorRevolution);
  m_linearEncoder.Reset();
  m_holdDistance = 0.0;
  m_state = HOLD;
}

void Linear::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        SetSpeed(0.0);
        m_holdDistance = m_controller.GetGoal().position.value();
      }
      else {
        m_linearMotor.Set(
          m_controller.Calculate(units::meter_t{GetDistance()}) + 
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

void Linear::SetDistance(double distance) {
  if (m_state != AUTO_MOVING) {
    m_state = AUTO_MOVING;
    m_controller.SetGoal(units::meter_t{distance});
  }
}

void Linear::SetSpeed(double speed){
  if (m_state == MANUAL_MOVING) {
    m_linearMotor.Set(speed);
  }
}

double Linear::GetDistance() {
  return m_linearEncoder.GetDistance() + m_offset;
}

bool Linear::CheckGoal() {
  return m_state == HOLD;
}

void Linear::StartAdjustment() {
  m_state = MANUAL_MOVING;
}
void Linear::EndAdjustment(double distance) {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_holdDistance = distance;
    m_linearEncoder.Reset();
    m_offset = distance;
  }
}