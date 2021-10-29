// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/math>
#include <cmath>
#include "ctre/Phoenix.h"

class Leg : public frc2::SubsystemBase {
 public:
  Leg(int, int);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Set(double, double);

  double sgn(double);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  TalonSRX shoulder;
  TalonSRX elbow;
  bool debug = true;
  double phi_1, phi_2, theta_1, theta_2;

  const double L1 = 9.5;
  const double L2 = 11.25;

  const double shoulderOffset = 975;
  const double elbowOffset = 1100;
  const double ticksPerRad = 2048 / wpi::math::pi;
};
