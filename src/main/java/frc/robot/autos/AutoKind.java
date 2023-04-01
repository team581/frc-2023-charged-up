// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),
  TEST("Test", 2, 3, false),

  BLUE_LONG_SIDE_2("BlueLongSide2", 4, 3, false),
  BLUE_MID_1_5_BALANCE("BlueMid1.5Balance", 2, 3, true),
  BLUE_SHORT_SIDE_2("BlueShortSide2", 3, 3, false),
  BLUE_SHORT_SIDE_2_BALANCE("BlueShortSide2Balance", 4, 4, true),
  BLUE_SHORT_SIDE_3("BlueShortSide3", 5, 4, false),

  RED_LONG_SIDE_2("RedLongSide2", 4, 3, false),
  RED_MID_1_5_BALANCE("RedMid1.5Balance", 2, 3, true),
  RED_SHORT_SIDE_2("RedShortSide2", 3, 3, false),
  RED_SHORT_SIDE_2_BALANCE("RedShortSide2Balance", 4, 4, true),
  RED_SHORT_SIDE_3("RedShortSide3", 5, 4, false);

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
