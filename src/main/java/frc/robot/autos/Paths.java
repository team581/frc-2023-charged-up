// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.managers.SuperstructureManager;

public class Paths {
  private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(1, 4);

  private SuperstructureManager superstructure;

  private Paths(SuperstructureManager superstructure) {
    this.superstructure = superstructure;
  }

  public static final PathPlannerTrajectory BALANCE_AUTO =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(3.0, 3.0),
              Rotation2d.fromDegrees(45),
              Rotation2d.fromDegrees(-90)));

  public static final PathPlannerTrajectory DRIVE_BACKWARDS =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(-0.5, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory BACK_RIGHT_FORWARD =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(-1.21, 0.0),
              Rotation2d.fromDegrees(90),
              Rotation2d.fromDegrees(-90)),
          new PathPoint(
              new Translation2d(-1.21, -1.95),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(0.0, -1.95),
              Rotation2d.fromDegrees(-90),
              Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory ORIGIN =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory RIGHT_SIDE_GRID_TO_OPPOSITE_PIECE =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(-4.6, -1.1049),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)));

  public static final PathPlannerTrajectory NODE_TO_RIGHT =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(-4.6, -1.1049),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(0.0, -1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
}
