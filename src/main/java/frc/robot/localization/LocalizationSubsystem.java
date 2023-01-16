package frc.robot.localization;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LocalizationSubsystem extends SubsystemBase {
  double[] botpose = {0,0,0,0,0,0};
  
  public LocalizationSubsystem() {
  
  }

  @Override
  public void periodic() {
    double[] zeroArray ={0,0,0,0,0,0};
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(zeroArray);
    Logger.getInstance().recordOutput("Bot Position", botpose);
  }
}
