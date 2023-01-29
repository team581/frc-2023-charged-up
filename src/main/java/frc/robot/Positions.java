// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(1, Rotation2d.fromDegrees(10));

  public static final SuperstructurePosition INTAKING_CUBE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition OUTTAKING_CUBE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CUBE_NODE_HYBRID =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CUBE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));

  public static final SuperstructurePosition INTAKING_CONE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition OUTTAKING_CONE =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CONE_NODE_HYBRID =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CONE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));
}
