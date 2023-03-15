// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_LONG_SIDE_2("BlueLongSide2", 3, 4, false),
  BLUE_MID_1_5_BALANCE("BlueMid1.5Balance", 3, 3, true),
  BLUE_SHORT_SIDE_2("BlueShortSide2", 3, 4, false),

  RED_LONG_SIDE_2("RedLongSide2", 3, 4, false),
  RED_MID_1_5_BALANCE("RedMid1.5Balance", 3, 3, true),
  RED_SHORT_SIDE_2("RedShortSide2", 3, 4, false);

  // BLUE_LONG_SIDE_1("BlueLongSide1", 1, 1, false),
  // BLUE_MID_1_BALANCE("BlueMid1Balance", 1.5, 2.5, true),
  // BLUE_SHORT_SIDE_1("BlueShortSide1", 2, 2, false),

  // RED_LONG_SIDE_1("RedLongSide1", 1.5, 1, false),
  // RED_MID_1_BALANCE("RedMid1Balance", 1.5, 2.5, true),
  // RED_SHORT_SIDE_1("RedShortSide1", 2, 1, false),

  // EXTRA_RED_SHORT_SIDE_1_BALANCE("ExtraRedShortSide1Balance", 2, 1, true),

  // TEST("Test", 1.5, 1.5, true);

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
