// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class Paths {
  private Paths() {}

  public static final PathPlannerTrajectory BLUE_LONG_SIDE_1_CONE =
      PathPlanner.loadPath("BlueLongSide1Cone", new PathConstraints(1.0, 0.75));
  public static final PathPlannerTrajectory BLUE_SHORT_SIDE_1_CONE =
      PathPlanner.loadPath("BlueShortSide1Cone", new PathConstraints(1, 0.75));
  public static final PathPlannerTrajectory BLUE_LONG_SIDE_1_5_CONE_BALANCE =
      PathPlanner.loadPath("BlueLongSide1.5ConeBalance", new PathConstraints(1.5, 0.75));
  public static final PathPlannerTrajectory BLUE_MID_1_5_CONE_BALANCE =
      PathPlanner.loadPath("BlueMid1.5ConeBalance", new PathConstraints(2.5, 0.75));
  public static final PathPlannerTrajectory BLUE_MID_1_CONE_BALANCE =
      PathPlanner.loadPath("BlueMid1ConeBalance", new PathConstraints(1.5, 2.5));
  public static final PathPlannerTrajectory BLUE_SHORT_SIDE_2_5_CONE_BALANCE =
      PathPlanner.loadPath("BlueShortSide2.5ConeBalance", new PathConstraints(4.0, 3.0));
  public static final PathPlannerTrajectory BLUE_SHORT_SIDE_2_CONE_BALANCE =
      PathPlanner.loadPath("BlueShortSide2ConeBalance", new PathConstraints(1.75, 2.0));

  public static final PathPlannerTrajectory RED_LONG_SIDE_1_CONE =
      PathPlanner.loadPath("RedLongSide1Cone", new PathConstraints(1, 0.75));
  public static final PathPlannerTrajectory RED_SHORT_SIDE_1_CONE =
      PathPlanner.loadPath("RedShortSide1Cone", new PathConstraints(1.0, 0.75));
  public static final PathPlannerTrajectory RED_LONG_SIDE_1_5_CONE_BALANCE =
      PathPlanner.loadPath("RedLongSide1.5ConeBalance", new PathConstraints(1.5, 0.75));
  public static final PathPlannerTrajectory RED_MID_1_5_CONE_BALANCE =
      PathPlanner.loadPath("RedMid1.5ConeBalance", new PathConstraints(4, 3));
  public static final PathPlannerTrajectory RED_MID_1_CONE_BALANCE =
      PathPlanner.loadPath("RedMid1ConeBalance", new PathConstraints(1.5, 2.5));
  public static final PathPlannerTrajectory RED_SHORT_SIDE_2_5_CONE_BALANCE =
      PathPlanner.loadPath("RedShortSide2.5ConeBalance", new PathConstraints(4.0, 3.0));
  public static final PathPlannerTrajectory RED_SHORT_SIDE_2_CONE_BALANCE =
      PathPlanner.loadPath("RedShortSide2ConeBalance", new PathConstraints(1.65, 1.5));
}
