// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Config;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(25), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), -1);
  public static final SuperstructurePosition FULL_EXTENSION =
      new SuperstructurePosition(40, Rotation2d.fromDegrees(10), -1);

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(137), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(154), -1);
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(44, Rotation2d.fromDegrees(125), -1)
          : new SuperstructurePosition(1, Rotation2d.fromDegrees(145), -1);
  public static final SuperstructurePosition CUBE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(125), -1)
          : new SuperstructurePosition(1, Rotation2d.fromDegrees(125), -1);
  public static final SuperstructurePosition CUBE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(24, Rotation2d.fromDegrees(125), -1)
          : new SuperstructurePosition(34, Rotation2d.fromDegrees(146), -1);
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(48, Rotation2d.fromDegrees(125), -1)
          : new SuperstructurePosition(56, Rotation2d.fromDegrees(157), -1);

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(131), -1)
          : new SuperstructurePosition(6.78, Rotation2d.fromDegrees(161), -1);
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(46, Rotation2d.fromDegrees(131), -1)
          : new SuperstructurePosition(2, Rotation2d.fromDegrees(143), -1);
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(50), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), -1);
  public static final SuperstructurePosition CONE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(130), -1)
          : new SuperstructurePosition(2, Rotation2d.fromDegrees(145), -1);
  public static final SuperstructurePosition CONE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(42, Rotation2d.fromDegrees(155), -1)
          : new SuperstructurePosition(50, Rotation2d.fromDegrees(175), -1);
  public static final SuperstructurePosition CONE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(50, Rotation2d.fromDegrees(130), -1)
          : new SuperstructurePosition(62, Rotation2d.fromDegrees(155), -1);

  private Positions() {}
}
