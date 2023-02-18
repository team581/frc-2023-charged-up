// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Halfmarks {
  public static final Pose2d P1_RIGHT_GRID_TO_PRELOADS =
      new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
  public static final Pose2d P2_RIGHT_GRID_TO_PRELOADS =
      new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(180));
  // public static final Pose2d P1_PRELOADS_TO_RIGHT_GRID =
  // public static final Pose2d P2_PRELOADS_TO_RIGHT_GRID =
  public static final Pose2d P1_LEFT_GRID_TO_PRELOADS =
      new Pose2d(
          Units.inchesToMeters(578.5),
          Units.inchesToMeters(46.905),
          Rotation2d.fromDegrees(0));
  public static final Pose2d P2_LEFT_GRID_TO_PRELOADS =
      new Pose2d(
          Units.inchesToMeters(484),
          Units.inchesToMeters(45),
          Rotation2d.fromDegrees(180));

  private Halfmarks() {}
}
