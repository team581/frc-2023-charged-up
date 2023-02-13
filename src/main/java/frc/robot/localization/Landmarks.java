
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Landmarks {
  public static final Pose2d APRILTAG_1 =
      new Pose2d(
          Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_2 =
      new Pose2d(
          Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_3 =
      new Pose2d(
          Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_4 =
      new Pose2d(
          Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_5 =
      new Pose2d(
          Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_6 =
      new Pose2d(
          Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_7 =
      new Pose2d(
          Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_8 =
      new Pose2d(
          Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0));

  public static final Pose2d BLUE_GRID_RIGHT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(42), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_CENTER =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(108), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_LEFT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(174), Rotation2d.fromDegrees(0));

  public static final Pose2d BLUE_GRID_LEFT_NODE_LEFT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(196), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_LEFT_NODE_CENTER =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(174), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_LEFT_NODE_RIGHT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(152), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_CENTER_NODE_LEFT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(130), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_CENTER_NODE_CENTER =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(108), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_CENTER_NODE_RIGHT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(86), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_RIGHT_NODE_LEFT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(64), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_RIGHT_NODE_CENTER =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(42), Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_GRID_RIGHT_NODE_RIGHT =
      new Pose2d(Units.inchesToMeters(54.05), Units.inchesToMeters(20), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_GRID_LEFT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(42), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_CENTER =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(108), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_RIGHT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(174), Rotation2d.fromDegrees(180));

  public static final Pose2d RED_GRID_LEFT_NODE_RIGHT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(64), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_LEFT_NODE_CENTER =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(42), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_LEFT_NODE_LEFT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(20), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_CENTER_NODE_RIGHT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(130), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_CENTER_NODE_CENTER =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(108), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_CENTER_NODE_LEFT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(86), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_RIGHT_NODE_RIGHT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(196), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_RIGHT_NODE_CENTER =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(174), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_GRID_RIGHT_NODE_LEFT =
      new Pose2d(
          Units.inchesToMeters(596.77), Units.inchesToMeters(152), Rotation2d.fromDegrees(180));

  public static final Pose2d BLUE_PRELOAD_FAR_RIGHT =
      new Pose2d(
          Units.inchesToMeters(278.05), Units.inchesToMeters(36.19), Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_PRELOAD_RIGHT =
      new Pose2d(
          Units.inchesToMeters(278.05), Units.inchesToMeters(84.19), Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_PRELOAD_LEFT =
      new Pose2d(
          Units.inchesToMeters(278.05), Units.inchesToMeters(132.19), Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_PRELOAD_FAR_LEFT =
      new Pose2d(
          Units.inchesToMeters(278.05), Units.inchesToMeters(180.19), Rotation2d.fromDegrees(180));

  public static final Pose2d RED_PRELOAD_FAR_LEFT =
      new Pose2d(
          Units.inchesToMeters(372.77), Units.inchesToMeters(36.19), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_PRELOAD_LEFT =
      new Pose2d(
          Units.inchesToMeters(372.77), Units.inchesToMeters(84.19), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_PRELOAD_RIGHT =
      new Pose2d(
          Units.inchesToMeters(372.77), Units.inchesToMeters(132.19), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_PRELOAD_FAR_RIGHT =
      new Pose2d(
          Units.inchesToMeters(372.77), Units.inchesToMeters(180.19), Rotation2d.fromDegrees(0));

  private Landmarks() {}
}
