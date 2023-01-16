// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

public class Config {
  public static final double CONTROLLER_PORT = 0;

  public static final RobotConfigKind CONFIG_KIND =
      System.getenv("serialnum") == "xxxxx" ? RobotConfigKind.TYKE : RobotConfigKind.SPIKE;

  public static final double ELEVATOR_GEARING =
      CONFIG_KIND == RobotConfigKind.SPIKE ? 999 : 60.0 * 2.0;
}
