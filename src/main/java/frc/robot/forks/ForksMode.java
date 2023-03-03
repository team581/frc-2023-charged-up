// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.forks;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ForksMode {
  STOPPED(Rotation2d.fromDegrees(0)),
  DOWN(Rotation2d.fromDegrees(110)),
  UP(Rotation2d.fromDegrees(5));

  public final Rotation2d angle;

  private ForksMode(Rotation2d angle) {
    this.angle = angle;
  }
}
