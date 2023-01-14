// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final Rotation2d angleOffset;
  public final SwerveCorner corner;
  public final boolean angleInversion;
  public final boolean driveInversion;

  public SwerveModuleConstants(
      Rotation2d angleOffset, SwerveCorner corner, boolean angleInversion, boolean driveInversion) {
    this.angleOffset = angleOffset;
    this.corner = corner;
    this.angleInversion = angleInversion;
    this.driveInversion = driveInversion;
  }
}
