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
import frc.robot.config.Config;
import frc.robot.localization.Halfmarks;
import frc.robot.localization.Landmarks;
import java.util.List;

public class Paths {
  private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(1.5, 2.5);

  private Paths() {}

  public static final PathPlannerTrajectory BALANCE_AUTO =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(3.0, 3.0),
              Rotation2d.fromDegrees(45),
              Rotation2d.fromDegrees(-90)));

  public static final PathPlannerTrajectory RED_2_RIGHT_CONE_TO_BALANCE =
      PathPlanner.loadPath("Red2RightConeToBalance", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory RED_2_LEFT_CONE_TO_BALANCE =
      PathPlanner.loadPath("Red2LeftConeToBalance", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory RED_3_RIGHT_CONE_AUTO =
      PathPlanner.loadPath("Red3RightConeAuto", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory RED_3_LEFT_CONE_AUTO =
      PathPlanner.loadPath("Red3LeftConeAuto", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory RED_RIGHT_BALANCE_AUTO =
      PathPlanner.loadPath("RedRightBalanceAuto", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory TEST_PATH =
      PathPlanner.loadPath("TestPath", PATH_CONSTRAINTS);

  public static final PathPlannerTrajectory RIGHT_GRID_RIGHT_TO_FAR_RIGHT_STAGING_MARK =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_RIGHT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_RIGHT.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_RIGHT_CHARGE.getX(), Halfmarks.RED_GRID_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_RIGHT_CHARGE.getX(), Halfmarks.RED_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(90),
              Rotation2d.fromDegrees(90)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)));

  public static final PathPlannerTrajectory FAR_RIGHT_STAGING_MARK_TO_RED_GRID_RIGHT_LEFT =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_RIGHT.getY() - Config.ROBOT_CENTER_TO_FRONT),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_RIGHT_CHARGE.getX(), Halfmarks.RED_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_RIGHT_CHARGE.getX(), Halfmarks.RED_GRID_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getY() - Config.ROBOT_CENTER_TO_FRONT),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)));

  public static final PathPlannerTrajectory RIGHT_GRID_LEFT_TO_MIDDLE_RIGHT_STAGING_MARK =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getX() - Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_GRID_RIGHT_NODE_LEFT.getY() - Config.ROBOT_CENTER_TO_FRONT),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_RIGHT_CHARGE.getX(), Halfmarks.RED_GRID_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_RIGHT_CHARGE.getX(), Halfmarks.RED_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(90),
              Rotation2d.fromDegrees(90)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_RIGHT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_RIGHT.getY() - Config.ROBOT_CENTER_TO_FRONT),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)));

  // public static final PathPlannerTrajectory MIDDLE_RIGHT_STAGING_MARK_TO_RIGHT_GRID_LEFT =
  //     PathPlanner.generatePath(
  //         PATH_CONSTRAINTS,
  //         new PathPoint(
  //             new Translation2d(
  //                 Landmarks.RED_STAGING_MARK_RIGHT.getX() + Config.ROBOT_CENTER_TO_FRONT,
  //                 Landmarks.RED_STAGING_MARK_RIGHT.getY()),
  //             Rotation2d.fromDegrees(0),
  //             Rotation2d.fromDegrees(270)),
  //         new PathPoint(
  //             new Translation2d(Units.inchesToMeters(442), Units.inchesToMeters(109)),
  //             Rotation2d.fromDegrees(0),
  //             Rotation2d.fromDegrees(0)),
  //         new PathPoint(
  //             new Translation2d(Units.inchesToMeters(421), Units.inchesToMeters(132)),
  //             Rotation2d.fromDegrees(0),
  //             Rotation2d.fromDegrees(180)),
  //         new PathPoint(
  //             new Translation2d(
  //                 Landmarks.RED_GRID_RIGHT_NODE_LEFT.getX() - Config.ROBOT_CENTER_TO_FRONT,
  //                 Landmarks.RED_GRID_RIGHT_NODE_LEFT.getY()),
  //             Rotation2d.fromDegrees(0),
  //             Rotation2d.fromDegrees(0)));

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
                  Halfmarks.RED_GRID_LEFT_CHARGE.getX(), Halfmarks.RED_GRID_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(Halfmarks.RED_LEFT_CHARGE.getX(), Halfmarks.RED_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(90),
              Rotation2d.fromDegrees(90)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)));

  public static final PathPlannerTrajectory FAR_LEFT_STAGING_MARK_TO_LEFT_GRID_CENTER =
      PathPlanner.generatePath(
          PATH_CONSTRAINTS,
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_GRID_LEFT_CHARGE.getX(), Halfmarks.RED_GRID_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
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

  public static final PathPlannerTrajectory LEFT_GRID_RIGHT_TO_LEFT_STAGING_MARK =
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
                  Halfmarks.RED_GRID_LEFT_CHARGE.getX(), Halfmarks.RED_GRID_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0)),
          new PathPoint(
              new Translation2d(Halfmarks.RED_LEFT_CHARGE.getX(), Halfmarks.RED_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(90),
              Rotation2d.fromDegrees(90)),
          new PathPoint(
              new Translation2d(
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getX(),
                  Halfmarks.RED_STAGING_MARKS_LEFT_CHARGE.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)),
          new PathPoint(
              new Translation2d(
                  Landmarks.RED_STAGING_MARK_LEFT.getX() + Config.ROBOT_CENTER_TO_FRONT,
                  Landmarks.RED_STAGING_MARK_FAR_LEFT.getY()),
              Rotation2d.fromDegrees(180),
              Rotation2d.fromDegrees(180)));
}
