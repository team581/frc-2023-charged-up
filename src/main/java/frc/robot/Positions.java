// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(1, Rotation2d.fromDegrees(10));
  public static final SuperstructurePosition FULL_EXTENSION =
      new SuperstructurePosition(32, Rotation2d.fromDegrees(10));

  public static final SuperstructurePosition INTAKING_CUBE =
      new SuperstructurePosition(3, Rotation2d.fromDegrees(130));
  public static final SuperstructurePosition CUBE_NODE_LOW =
      new SuperstructurePosition(4, Rotation2d.fromDegrees(127));
  public static final SuperstructurePosition CUBE_NODE_MID =
      new SuperstructurePosition(17, Rotation2d.fromDegrees(121));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(28, Rotation2d.fromDegrees(132));

  public static final SuperstructurePosition INTAKING_CONE =
      new SuperstructurePosition(7, Rotation2d.fromDegrees(158));
  public static final SuperstructurePosition CONE_NODE_LOW =
      new SuperstructurePosition(10, Rotation2d.fromDegrees(162));
  public static final SuperstructurePosition CONE_NODE_MID =
      new SuperstructurePosition(25, Rotation2d.fromDegrees(150));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(31, Rotation2d.fromDegrees(130));
}
