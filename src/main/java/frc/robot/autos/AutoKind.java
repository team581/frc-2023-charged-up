// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_LONG_SIDE_1_5("BlueLongSide1.5", 2.5, 0.75, true),
  BLUE_LONG_SIDE_1("BlueLongSide1", 1, 0.75, false),
  BLUE_MID_1_5_BALANCE("BlueMid1.5Balance", 1.5, 0.75, true),
  BLUE_MID_1_BALANCE("BlueMid1Balance", 1.5, 2.5, true),
  BLUE_SHORT_SIDE_1("BlueShortSide1", 1, 2, false),
  BLUE_SHORT_SIDE_2_5_BALANCE("BlueShortSide2.5Balance", 2, 1, true),
  BLUE_SHORT_SIDE_2("BlueShortSide2Balance", 1.75, 2, true),

  RED_LONG_SIDE_1_5("RedLongSide1.5", 2.65, 2.2, true),
  RED_LONG_SIDE_1("RedLongSide1", 1, 0.75, false),
  // RED_LONG_SIDE_2_5_BALANCE("RedLongSide2.5Balance", 1, 1, true),
  RED_MID_1_5_BALANCE("RedMid1.5Balance", 3, 3, true),
  RED_MID_1_BALANCE("RedMid1Balance", 1.5, 2.5, true),
  RED_SHORT_SIDE_1("RedShortSide1", 2, 1, false),
  RED_SHORT_SIDE_1_BALANCE("RedShortSide1Balance", 2, 1, true),
  // RED_SHORT_SIDE_2_5_BALANCE("RedShortSide2.5Balance", 2, 2, true),
  RED_SHORT_SIDE_2("RedShortSide2", 2, 3, true),
  TEST("Test", 1.5, 1.5, true);

  public final String pathName;
  public final PathConstraints constraints;
  public final boolean autoBalance;

  private AutoKind(
      String pathName, double maxVelocity, double maxAcceleration, boolean autoBalance) {
    this.pathName = pathName;
    this.constraints = new PathConstraints(maxVelocity, maxAcceleration);
    this.autoBalance = autoBalance;
  }
}
