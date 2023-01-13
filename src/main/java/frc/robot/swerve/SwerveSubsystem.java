// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

public class SwerveSubsystem {
  private SwerveModule frontRight;
  private SwerveModule frontLeft;
  private SwerveModule backRight;
  private SwerveModule backLeft;

  public SwerveSubsystem(
      SwerveModule frontRight,
      SwerveModule frontLeft,
      SwerveModule backRight,
      SwerveModule backLeft) {
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backRight = backRight;
    this.backLeft = backLeft;
  }
}
