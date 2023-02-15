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
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(25))
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(10));
  public static final SuperstructurePosition FULL_EXTENSION =
      new SuperstructurePosition(20, Rotation2d.fromDegrees(10));

  public static final SuperstructurePosition INTAKING_CUBE =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(136))
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(120));
  public static final SuperstructurePosition CUBE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(127))
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(100));
  public static final SuperstructurePosition CUBE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(12, Rotation2d.fromDegrees(110))
          : new SuperstructurePosition(17, Rotation2d.fromDegrees(121));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(23, Rotation2d.fromDegrees(125))
          : new SuperstructurePosition(28, Rotation2d.fromDegrees(132));

  public static final SuperstructurePosition INTAKING_CONE =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(130))
          : new SuperstructurePosition(1, Rotation2d.fromDegrees(118));
  public static final SuperstructurePosition CONE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(130))
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(120));
  public static final SuperstructurePosition CONE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(23, Rotation2d.fromDegrees(165))
          : new SuperstructurePosition(25, Rotation2d.fromDegrees(150));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(25, Rotation2d.fromDegrees(130))
          : new SuperstructurePosition(31, Rotation2d.fromDegrees(130));

  private Positions() {}
}
