// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveLeg.h"

MoveLeg::MoveLeg(Leg *leg, frc::XboxController *controller) : m_leg{leg}, m_controller{controller} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_leg});
}

// Called when the command is initially scheduled.
void MoveLeg::Initialize() {
  m_leg->Set(-radius, defaultHeight);
  double cumsum = 0;
  for(int i = 0; i < N; i++){
    if(cumsum < wpi::math::pi){
      xPos.push_back(-radius * std::cos(cumsum));
      yPos.push_back(defaultHeight - radius * std::sin(cumsum));
    } else {
      xPos.push_back(-radius * std::cos(cumsum));
      yPos.push_back(defaultHeight);
    }
    cumsum += angleStep;
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveLeg::Execute() {
  // x = -6.0 * m_controller->GetY(frc::GenericHID::kLeftHand);
  // y = 12.0 * m_controller->GetTriggerAxis(frc::GenericHID::kRightHand) + 6;
  // m_leg->Set(x, y);
  while(true){
    x = xPos[idx];
    y = yPos[idx];
    m_leg->Set(x, y);
    frc::Wait(0.1);
    idx = (idx != N-1) ? idx + 1 : 0;
  }
}

// Called once the command ends or is interrupted.
void MoveLeg::End(bool interrupted) {
  m_leg->Set(0,defaultHeight);
}

// Returns true when the command should end.
bool MoveLeg::IsFinished() {
  return false;
}
