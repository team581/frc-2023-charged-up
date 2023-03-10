// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SuperstructurePosition {
  public final double height;
  public final Rotation2d angle;
  public final double earlyTransitionHeight;

  public SuperstructurePosition(
      double heightInches, Rotation2d angle, double earlyTransitionHeightInches) {
    this.height = Units.inchesToMeters(heightInches);
    this.angle = angle;
    this.earlyTransitionHeight =
        earlyTransitionHeightInches == -1 ? -1 : Units.inchesToMeters(earlyTransitionHeightInches);
  }
}
