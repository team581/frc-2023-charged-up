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
}
