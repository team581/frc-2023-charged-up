// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.List;

public class Paths {
  private Paths() {}

  public static final List<PathPlannerTrajectory> BLUE_LONG_SIDE_1_CONE =
      PathPlanner.loadPathGroup("BlueLongSide1Cone", new PathConstraints(1.0, 0.75));
  public static final List<PathPlannerTrajectory> BLUE_SHORT_SIDE_1_CONE =
      PathPlanner.loadPathGroup("BlueShortSide1Cone", new PathConstraints(1, 0.75));
  public static final List<PathPlannerTrajectory> BLUE_LONG_SIDE_1_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("BlueLongSide1.5ConeBalance", new PathConstraints(1.5, 0.75));
  public static final List<PathPlannerTrajectory> BLUE_MID_1_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("BlueMid1.5ConeBalance", new PathConstraints(2.5, 0.75));
  public static final List<PathPlannerTrajectory> BLUE_MID_1_CONE_BALANCE =
      PathPlanner.loadPathGroup("BlueMid1ConeBalance", new PathConstraints(1.5, 2.5));
  public static final List<PathPlannerTrajectory> BLUE_SHORT_SIDE_2_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("BlueShortSide2.5ConeBalance", new PathConstraints(4.0, 3.0));
  public static final List<PathPlannerTrajectory> BLUE_SHORT_SIDE_2_CONE_BALANCE =
      PathPlanner.loadPathGroup("BlueShortSide2ConeBalance", new PathConstraints(1.75, 2.0));

  public static final List<PathPlannerTrajectory> RED_LONG_SIDE_1_CONE =
      PathPlanner.loadPathGroup("RedLongSide1Cone", new PathConstraints(1, 0.75));
  public static final List<PathPlannerTrajectory> RED_SHORT_SIDE_1_CONE =
      PathPlanner.loadPathGroup("RedShortSide1Cone", new PathConstraints(1.0, 0.75));
  public static final List<PathPlannerTrajectory> RED_LONG_SIDE_1_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("RedLongSide1.5ConeBalance", new PathConstraints(1.5, 0.75));
  public static final List<PathPlannerTrajectory> RED_MID_1_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("RedMid1.5ConeBalance", new PathConstraints(4, 3));
  public static final List<PathPlannerTrajectory> RED_MID_1_CONE_BALANCE =
      PathPlanner.loadPathGroup("RedMid1ConeBalance", new PathConstraints(1.5, 2.5));
  public static final List<PathPlannerTrajectory> RED_SHORT_SIDE_2_5_CONE_BALANCE =
      PathPlanner.loadPathGroup("RedShortSide2.5ConeBalance", new PathConstraints(4.0, 3.0));
  public static final List<PathPlannerTrajectory> RED_SHORT_SIDE_2_CONE_BALANCE =
      PathPlanner.loadPathGroup("RedShortSide2ConeBalance", new PathConstraints(1, 1));

  public static final List<PathPlannerTrajectory> BUG =
      PathPlanner.loadPathGroup("bug", new PathConstraints(1, 1));
}
