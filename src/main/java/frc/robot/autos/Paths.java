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
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.localization.Halfmarks;
import frc.robot.localization.Landmarks;
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

  public static final PathPlannerTrajectory RIGHT_NODE_TO_OPPOSITE_STAGING_MARK =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_RIGHT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_RIGHT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(442), Units.inchesToMeters(186)),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(421), Units.inchesToMeters(186)),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)));

  public static final PathPlannerTrajectory RIGHT_STAGING_MARK_TO_RED_GRID_RIGHT_CENTER =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(421), Units.inchesToMeters(132)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(442), Units.inchesToMeters(109)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_CENTER.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_CENTER.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory RIGHT_GRID_CENTER_TO_MIDDLE_RIGHT_STAGING_MARK =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_CENTER.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_CENTER.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(421), Units.inchesToMeters(132)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(442), Units.inchesToMeters(109)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_RIGHT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(270)));

  public static final PathPlannerTrajectory MIDDLE_RIGHT_STAGING_MARK_TO_RIGHT_GRID_LEFT =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_RIGHT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(270)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(442), Units.inchesToMeters(109)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(421), Units.inchesToMeters(132)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory LEFT_GRID_LEFT_TO_FAR_LEFT_STAGING_MARK =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_LEFT_NODE_LEFT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_LEFT_NODE_LEFT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_LEFT_CHARGE.getX(),
                  Halfmarks.RED_GRID_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Halfmarks.RED_GRID_LEFT_CHARGE.getRotation()),
          new PathPoint(
              new Translation2d(Units.inchesToMeters(578.5), Units.inchesToMeters(46.905)),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)));

  public static final PathPlannerTrajectory FAR_LEFT_STAGING_MARK_TO_LEFT_GRID_CENTER =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_LEFT_CHARGE.getX(),
                  Halfmarks.RED_GRID_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Halfmarks.RED_GRID_LEFT_CHARGE.getRotation()),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getRotation()),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_LEFT_NODE_CENTER.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_LEFT_NODE_CENTER.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)));
}
