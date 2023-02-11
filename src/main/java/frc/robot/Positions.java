// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(25)); // change height back to 0.5
  public static final SuperstructurePosition FULL_EXTENSION =
      new SuperstructurePosition(
          20,
          Rotation2d.fromDegrees(
              10)); // Find full extension height for spike later, 20 is a placeholder so it doesnt
  // break

  public static final SuperstructurePosition INTAKING_CUBE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(136));
  public static final SuperstructurePosition CUBE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(127));
  public static final SuperstructurePosition CUBE_NODE_MID =
      new SuperstructurePosition(12, Rotation2d.fromDegrees(110));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(23, Rotation2d.fromDegrees(125));

  public static final SuperstructurePosition INTAKING_CONE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(130));
  public static final SuperstructurePosition CONE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(130));
  public static final SuperstructurePosition CONE_NODE_MID =
      new SuperstructurePosition(25, Rotation2d.fromDegrees(175));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(25, Rotation2d.fromDegrees(130));

  private Positions() {}
}
