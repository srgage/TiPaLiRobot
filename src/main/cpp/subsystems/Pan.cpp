#include "subsystems/Pan.h"

Pan::Pan() {
  Init();

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("Pan");
  m_statePub = table->GetStringTopic("State").Publish();
  m_backPub = table->GetDoubleTopic("Feedback").Publish();
  m_setPointVelPub = table->GetDoubleTopic("SetPointVel").Publish();
  m_setPointPosPub = table->GetDoubleTopic("SetPointPos").Publish();

  m_backPub.Set(0.0);
  m_setPointVelPub.Set(0.0);
  m_setPointPosPub.Set(0.0);
}

void Pan::Periodic() {
  // This method will be called once per scheduler run.
  switch (m_state) {
    case AUTO_MOVING:
      if (m_controller.AtGoal()) {
        m_state = HOLD;
        m_statePub.Set("HOLD");
        m_panMotor.Set(0.0);
        m_holdAngle = m_controller.GetGoal().position.value();
      }
      else {
        double back = m_controller.Calculate(units::degree_t{m_panEncoder.GetDistance()});
        auto setpoint = m_controller.GetSetpoint();
        m_panMotor.Set(back);
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

void Pan::SetAngle(double angle, bool slow) { //Special
  if (m_state != AUTO_MOVING) {
    if (slow) {
      m_controller.SetConstraints(m_slowConstraints);
    }
    else {
      m_controller.SetConstraints(m_fastConstraints);
    }

    m_state = AUTO_MOVING;
    m_statePub.Set("AUTO_MOVING");
    m_controller.Reset(units::degree_t{GetAngle()});
    m_controller.SetTolerance(kTolerancePos, kToleranceVel);
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
  m_statePub.Set("MANUAL_MOVING");
}
void Pan::EndAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_statePub.Set("HOLD");
    m_holdAngle = 0.0;
    m_panEncoder.Reset();
  }
}
void Pan::CancelAdjustment() {
  if (m_state == MANUAL_MOVING) {
    m_state = HOLD;
    m_statePub.Set("HOLD");
  }
}
void Pan::Init() {
  m_panMotor.SetInverted(true);
  m_panEncoder.SetDistancePerPulse(360.0 / (kCountsPerMotorRevolution * kGearRatio));
  m_panEncoder.Reset();
  m_holdAngle = 0.0;
  m_state = HOLD;
  m_statePub.Set("HOLD");
}