package frc.robot.localization;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.LifecycleSubsystem;

public class LocalizationSubsystem extends LifecycleSubsystem {

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private Pose2d botPose = new Pose2d();
  
  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu) {
    this.swerve = swerve;
    this.imu = imu;
  }

  @Override
  public void robotPeriodic() {
    double[] emptyArray ={};
    double[] rawPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(emptyArray);
    
    if (rawPose.length > 0) {
      botPose =  new Pose2d(rawPose[0], rawPose[1], Rotation2d.fromDegrees(rawPose[4]));
      Logger.getInstance().recordOutput("Bot Position", botPose);
    }
  }
}
