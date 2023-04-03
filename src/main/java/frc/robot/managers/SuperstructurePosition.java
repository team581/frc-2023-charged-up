// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructurePosition {
  public final double height;
  public final Rotation2d angle;
  public final double earlyTransitionHeight;
  public final boolean skipCollisionAvoidance;

  public SuperstructurePosition(double height, Rotation2d angle, double earlyTransitionHeight) {
    this(height, angle, earlyTransitionHeight, false);
  }

  public SuperstructurePosition(
      double height,
      Rotation2d angle,
      double earlyTransitionHeight,
      boolean skipCollisionAvoidance) {
    this.height = height;
    this.angle = angle;
    this.earlyTransitionHeight = earlyTransitionHeight;
    this.skipCollisionAvoidance = skipCollisionAvoidance;
  }
}
