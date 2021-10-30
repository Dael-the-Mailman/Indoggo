// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Leg.h"

Leg::Leg(int shoulderPort, int elbowPort) : shoulder(shoulderPort), elbow(elbowPort){
    shoulder.ConfigFactoryDefault();
    shoulder.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 30);
    shoulder.Config_kP(0, 0.5);
    shoulder.Config_kI(0, 0.0);
    shoulder.Config_kD(0, 0.0);
    shoulder.Config_IntegralZone(0, 0.0);
    shoulder.SetSensorPhase(false);
    shoulder.SetInverted(true);

    elbow.ConfigFactoryDefault();
    elbow.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 30);
    elbow.Config_kP(0, 0.5);
    elbow.Config_kI(0, 0.0);
    elbow.Config_kD(0, 0.0);
    elbow.Config_IntegralZone(0, 0.0);
    elbow.SetSensorPhase(false);
    elbow.SetInverted(false);

    if(debug){
        shoulder.ConfigPeakOutputForward(0.1);
        shoulder.ConfigPeakOutputReverse(-0.1);
        elbow.ConfigPeakOutputForward(0.1);
        elbow.ConfigPeakOutputReverse(-0.1);
    }

    // shoulder.Set(ControlMode::Position, -shoulderOffset);
    // elbow.Set(ControlMode::Position, elbowOffset);
}

// This method will be called once per scheduler run
void Leg::Periodic() {
    frc::SmartDashboard::PutNumber("Shoulder Encoder", shoulder.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Elbow Encoder", elbow.GetSelectedSensorPosition());
}

void Leg::Set(double x, double y){
    if(!(x == 0 && y == 0)){
        phi_1 = std::atan2(x, y);
        phi_2 = std::acos((L1*L1 + L2*L2 - x*x - y*y)/(2*L1*L2));
        theta_1 = phi_1 - std::asin((L2*std::sin(phi_2))/(std::sqrt(x*x+y*y)));
        theta_2 = wpi::math::pi - phi_2; 

        shoulder.Set(ControlMode::Position, -shoulderOffset + theta_1*ticksPerRad);
        elbow.Set(ControlMode::Position, elbowOffset + theta_2*ticksPerRad);
    } else {
        shoulder.Set(ControlMode::Position, -shoulderOffset);
        elbow.Set(ControlMode::Position, elbowOffset);
    }

    frc::SmartDashboard::PutNumber("Phi 1", phi_1);
    frc::SmartDashboard::PutNumber("Phi 2", phi_2);
    frc::SmartDashboard::PutNumber("Theta 1", theta_1);
    frc::SmartDashboard::PutNumber("Theta 2", theta_2);
    frc::SmartDashboard::PutNumber("Shoulder Goal", -shoulderOffset + theta_1*ticksPerRad);
    frc::SmartDashboard::PutNumber("Elbow Goal", elbowOffset + theta_2*ticksPerRad);
}

double Leg::sgn(double n){
    return (n >= 0) ? 1 : -1;
}
