// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModuleConstants;

public class Config {
  private Config() {}

  public static final int CONTROLLER_PORT = 0;

  public static final RobotConfigKind CONFIG_KIND = RobotConfigKind.TYKE;

  private static final boolean IS_SPIKE = CONFIG_KIND == RobotConfigKind.SPIKE;

  public static final int PDP_ID = 1;
  public static final ModuleType PDP_TYPE = IS_SPIKE ? ModuleType.kRev : ModuleType.kCTRE;

  public static final int PIGEON_ID = 1;

  public static final double SWERVE_STEER_GEARING_REDUCTION = IS_SPIKE ? 150.0 / 7.0 : 12.8;
  public static final double SWERVE_DRIVE_GEARING_REDUCTION = IS_SPIKE ? 6.75 : 10.0;

  public static final int SWERVE_FL_DRIVE_MOTOR_ID = 8;
  public static final int SWERVE_FL_STEER_MOTOR_ID = 9;
  public static final int SWERVE_FL_CANCODER_ID = 13;
  public static final SwerveModuleConstants SWERVE_FL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(-62.84), SwerveCorner.FRONT_LEFT, false, false);

  public static final int SWERVE_FR_DRIVE_MOTOR_ID = 6;
  public static final int SWERVE_FR_STEER_MOTOR_ID = 7;
  public static final int SWERVE_FR_CANCODER_ID = 12;
  public static final SwerveModuleConstants SWERVE_FR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(-147.8), SwerveCorner.FRONT_RIGHT, false, false);

  public static final int SWERVE_BL_DRIVE_MOTOR_ID = 4;
  public static final int SWERVE_BL_STEER_MOTOR_ID = 5;
  public static final int SWERVE_BL_CANCODER_ID = 11;
  public static final SwerveModuleConstants SWERVE_BL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(78.75), SwerveCorner.BACK_LEFT, false, false);

  public static final int SWERVE_BR_DRIVE_MOTOR_ID = 2;
  public static final int SWERVE_BR_STEER_MOTOR_ID = 3;
  public static final int SWERVE_BR_CANCODER_ID = 10;
  public static final SwerveModuleConstants SWERVE_BR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(104.58), SwerveCorner.BACK_RIGHT, false, false);

  public static final int ELEVATOR_MOTOR_ID = 14;
  public static final double ELEVATOR_GEARING = 60.0;

  public static final int WRIST_MOTOR_ID = 16;
  public static final double WRIST_GEARING = 48.0;
}
