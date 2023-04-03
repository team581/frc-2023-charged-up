// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),
  TEST(AutoKind.TEST, AutoKind.TEST),

  LONG_SIDE_2(AutoKind.RED_LONG_SIDE_2, AutoKind.BLUE_LONG_SIDE_2),
  LONG_SIDE_2_BALANCE(AutoKind.RED_LONG_SIDE_2_BALANCE, AutoKind.BLUE_LONG_SIDE_2_BALANCE),

  MID_1_5_BALANCE(AutoKind.RED_MID_1_5_BALANCE, AutoKind.BLUE_MID_1_5_BALANCE),

  SHORT_SIDE_2(AutoKind.RED_SHORT_SIDE_2, AutoKind.BLUE_SHORT_SIDE_2),
  SHORT_SIDE_2_BALANCE(AutoKind.RED_SHORT_SIDE_2_BALANCE, AutoKind.BLUE_SHORT_SIDE_2_BALANCE),
  SHORT_SIDE_3(AutoKind.RED_SHORT_SIDE_3, AutoKind.BLUE_SHORT_SIDE_3);

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
