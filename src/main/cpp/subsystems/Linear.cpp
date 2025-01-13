#include "subsystems/Linear.h"

Linear::Linear() {
  Init();

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("Linear");
  m_statePub = table->GetStringTopic("State").Publish();
  m_backPub = table->GetDoubleTopic("Feedback").Publish();
  m_setPointVelPub = table->GetDoubleTopic("SetPointVel").Publish();
  m_setPointPosPub = table->GetDoubleTopic("SetPointPos").Publish();

  m_backPub.Set(0.0);
  m_setPointVelPub.Set(0.0);
  m_setPointPosPub.Set(0.0);
}

void Linear::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        m_statePub.Set("HOLD");
        m_linearMotor.Set(0.0);
        m_holdDistance = m_controller.GetGoal().position.value();
      }
      else {
        double back = m_controller.Calculate(units::meter_t{GetDistance()});
        auto setpoint = m_controller.GetSetpoint();
        m_linearMotor.Set(back);
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

void Linear::SetDistance(double distance) {
  if (m_state != AUTO_MOVING) {
    m_state = AUTO_MOVING;
    m_statePub.Set("AUTO_MOVING");
    m_controller.Reset(units::meter_t{GetDistance()});
    m_controller.SetTolerance(kTolerancePos, kToleranceVel);
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
  m_statePub.Set("MANUAL_MOVING");
}
void Linear::EndAdjustment(double distance) {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_statePub.Set("HOLD");
    m_holdDistance = distance;
    m_linearEncoder.Reset();
    m_offset = distance;
  }
}
void Linear::CancelAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_statePub.Set("HOLD");
  }
}

void Linear::Init() {
  m_linearMotor.SetInverted(true);
  m_linearEncoder.SetDistancePerPulse(kWheelDiameter * std::numbers::pi / kCountsPerMotorRevolution);
  m_linearEncoder.Reset();
  m_holdDistance = 0.0;
  m_state = HOLD;
  m_statePub.Set("HOLD");
}