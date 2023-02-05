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
import java.util.ArrayList;

public class Paths {
  private Paths() {}

  public static final PathPlannerTrajectory BALANCE_AUTO =
      PathPlanner.generatePath(
          new PathConstraints(4, 3),
          new PathPoint(
              new Translation2d(1.0, 1.0),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(
                  0)), // position, heading(direction of travel), holonomic rotation
          new PathPoint(
              new Translation2d(3.0, 3.0),
              Rotation2d.fromDegrees(45),
              Rotation2d.fromDegrees(
                  -90) // position, heading(direction of travel), holonomic rotation
              ));
  private static ArrayList<PathPoint> driveForwardList =
      new ArrayList<PathPoint>() {
        {
          add(
              new PathPoint(
                  new Translation2d(0.0, 4.0),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(0)));
        }
      };
  public static final PathPlannerTrajectory DRIVE_FORWARD =
      PathPlanner.generatePath(new PathConstraints(4, 3), driveForwardList);
}
