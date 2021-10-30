// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <vector>
#include <cmath>
#include "subsystems/Leg.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveLeg
    : public frc2::CommandHelper<frc2::CommandBase, MoveLeg> {
 public:
  MoveLeg(Leg *leg, frc::XboxController *controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Leg* m_leg;
  frc::XboxController* m_controller;

  double x, y, idx;
  const double radius = 6;
  const double N = 16;
  const double angleStep = 2 * wpi::math::pi / N;
  const double defaultHeight = 12;
  std::vector<double> xPos, yPos;
};
