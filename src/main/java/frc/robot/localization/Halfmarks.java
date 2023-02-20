// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Halfmarks {
  public static final Pose2d RED_GRID_RIGHT_CHARGE =
      new Pose2d(Units.inchesToMeters(550.5), Units.inchesToMeters(185), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_RIGHT_CHARGE =
      new Pose2d(Units.inchesToMeters(500), Units.inchesToMeters(185), Rotation2d.fromDegrees(90));
  public static final Pose2d RED_STAGING_MARKS_RIGHT_CHARGE =
      new Pose2d(
          Units.inchesToMeters(436.5), Units.inchesToMeters(185), Rotation2d.fromDegrees(180));

  public static final Pose2d RED_GRID_LEFT_CHARGE =
      new Pose2d(
          Units.inchesToMeters(565.5), Units.inchesToMeters(37.5), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_LEFT_CHARGE =
      new Pose2d(Units.inchesToMeters(484), Units.inchesToMeters(40), Rotation2d.fromDegrees(90));
  public static final Pose2d RED_STAGING_MARKS_LEFT_CHARGE =
      new Pose2d(
          Units.inchesToMeters(436.5), Units.inchesToMeters(40), Rotation2d.fromDegrees(180));

  private Halfmarks() {}
}