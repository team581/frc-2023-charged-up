// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModuleConstants;

public class Config {
  private static final String TYKE_SERIAL_NUMBER = "031617f6";

  public static final String SERIAL_NUMBER = System.getenv("serialnum");
  public static final boolean IS_SPIKE =
      SERIAL_NUMBER == null || !SERIAL_NUMBER.equalsIgnoreCase(TYKE_SERIAL_NUMBER);
  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;

  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final String CANIVORE_ID = "581CANivore";

  public static final int PDP_ID = IS_SPIKE ? 1 : 0;
  public static final ModuleType PDP_TYPE = IS_SPIKE ? ModuleType.kRev : ModuleType.kCTRE;

  public static final int PIGEON_ID = 1;

  public static final double SWERVE_STEER_GEARING_REDUCTION = IS_SPIKE ? 150.0 / 7.0 : 12.8;
  public static final double SWERVE_DRIVE_GEARING_REDUCTION = IS_SPIKE ? 6.75 : 8.14;

  public static final Translation2d SWERVE_FRONT_LEFT_LOCATION =
      IS_SPIKE ? new Translation2d(0.263525, 0.263525) : new Translation2d(0.381, 0.381);
  public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION =
      IS_SPIKE ? new Translation2d(0.263525, -0.263525) : new Translation2d(0.381, -0.381);
  public static final Translation2d SWERVE_BACK_LEFT_LOCATION =
      IS_SPIKE ? new Translation2d(-0.263525, 0.263525) : new Translation2d(-0.381, 0.381);
  public static final Translation2d SWERVE_BACK_RIGHT_LOCATION =
      IS_SPIKE ? new Translation2d(-0.263525, -0.263525) : new Translation2d(-0.381, -0.381);

  public static final int SWERVE_FL_DRIVE_MOTOR_ID = 8;
  public static final int SWERVE_FL_STEER_MOTOR_ID = 9;
  public static final int SWERVE_FL_CANCODER_ID = 13;
  public static final SwerveModuleConstants SWERVE_FL_CONSTANTS =
      IS_SPIKE
          ? new SwerveModuleConstants(
              Rotation2d.fromDegrees(135.61), SwerveCorner.FRONT_LEFT, true, true)
          : new SwerveModuleConstants(
              Rotation2d.fromDegrees(117.19), SwerveCorner.FRONT_LEFT, false, false);
  // -62.84
  public static final int SWERVE_FR_DRIVE_MOTOR_ID = 6;
  public static final int SWERVE_FR_STEER_MOTOR_ID = 7;
  public static final int SWERVE_FR_CANCODER_ID = 12;
  public static final SwerveModuleConstants SWERVE_FR_CONSTANTS =
      IS_SPIKE
          ? new SwerveModuleConstants(
              Rotation2d.fromDegrees(67.94), SwerveCorner.FRONT_RIGHT, true, true)
          : new SwerveModuleConstants(
              Rotation2d.fromDegrees(32.2), SwerveCorner.FRONT_RIGHT, false, false);
  // -147.8
  public static final int SWERVE_BL_DRIVE_MOTOR_ID = 4;
  public static final int SWERVE_BL_STEER_MOTOR_ID = 5;
  public static final int SWERVE_BL_CANCODER_ID = 11;
  public static final SwerveModuleConstants SWERVE_BL_CONSTANTS =
      IS_SPIKE
          ? new SwerveModuleConstants(
              Rotation2d.fromDegrees(307.0), SwerveCorner.BACK_LEFT, true, true)
          : new SwerveModuleConstants(
              Rotation2d.fromDegrees(-101.25), SwerveCorner.BACK_LEFT, false, false);
  // 78.75
  public static final int SWERVE_BR_DRIVE_MOTOR_ID = 2;
  public static final int SWERVE_BR_STEER_MOTOR_ID = 3;
  public static final int SWERVE_BR_CANCODER_ID = 10;
  public static final SwerveModuleConstants SWERVE_BR_CONSTANTS =
      IS_SPIKE
          ? new SwerveModuleConstants(
              Rotation2d.fromDegrees(68.82), SwerveCorner.BACK_RIGHT, true, true)
          : new SwerveModuleConstants(
              Rotation2d.fromDegrees(-75.42), SwerveCorner.BACK_RIGHT, false, false);
  // 104.58
  public static final int ELEVATOR_MOTOR_ID = 14;

  public static final double ELEVATOR_GEARING = IS_SPIKE ? 7.2 : 16.0;
  public static final double ELEVATOR_MIN_HEIGHT = IS_SPIKE ? 0 : 0;
  public static final double ELEVATOR_MAX_HEIGHT = IS_SPIKE ? 25.3 : 12;
  public static final double ELEVATOR_KF = IS_SPIKE ? 0 : 0;
  public static final double ELEVATOR_KP = IS_SPIKE ? 0.7 : 0.8;
  public static final double ELEVATOR_KI = IS_SPIKE ? 0 : 0;
  public static final double ELEVATOR_KD = IS_SPIKE ? 0.1 : 0;
  public static final double ELEVATOR_ARB_F = IS_SPIKE ? 0.08 : 0;
  public static final int ELEVATOR_CRUISE_VELOCITY = IS_SPIKE ? 25000 : 15000;
  public static final int ELEVATOR_ACCELERATION = IS_SPIKE ? 30000 : 27500;
  public static final boolean ELEVATOR_INVERTED = IS_SPIKE ? false : true;

  public static final int LIGHTS_CANDLE_ID = 15;
  public static final int LIGHTS_LED_COUNT = 0;

  public static final int WRIST_MOTOR_ID = 16;
  public static final double WRIST_GEARING = IS_SPIKE ? 25.0 * 2 : 48.0 * 2;
  public static final int WRIST_KF = IS_SPIKE ? 0 : 0;
  public static final double WRIST_KP = IS_SPIKE ? 0.25 : 0.1;
  public static final int WRIST_KI = IS_SPIKE ? 0 : 0;
  public static final int WRIST_KD = IS_SPIKE ? 0 : 0;
  public static final int WRIST_MOTION_CRUISE_VELOCITY = IS_SPIKE ? 10000 : 20000;
  public static final int WRIST_MOTION_ACCELERATION = IS_SPIKE ? 65000 : 50000;
  public static final double WRIST_HOMED_CURRENT = IS_SPIKE ? 15 : 15;
  public static final Rotation2d WRIST_HOMED_ANGLE =
      IS_SPIKE ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(0.0);
  public static final double WRIST_HOMING_VOLTAGE = IS_SPIKE ? -0.15 : -0.15;

  public static final int INTAKE_MOTOR_ID = 17;
  public static final boolean INVERTED_INTAKE = IS_SPIKE ? false : true;

  public static final int FORKS_MOTOR_ID = 18;
  public static final double FORKS_GEARING = IS_SPIKE ? 25 : 999;

  public static final double SWERVE_STEER_KV = IS_SPIKE ? 0.0 : 0.0;
  public static final double SWERVE_STEER_KP = IS_SPIKE ? 5.0 : 3.0;
  public static final double SWERVE_STEER_KI = IS_SPIKE ? 0.0 : 0.0;
  public static final double SWERVE_STEER_KD = IS_SPIKE ? 0.0 : 0.0;
  public static final double SWERVE_STEER_KS = IS_SPIKE ? 0.0 : 0.0;

  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_FORWARD_VOLTAGE = IS_SPIKE ? 12 : 0;
  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_REVERSE_VOLTAGE = IS_SPIKE ? -12 : 0;
  public static final double SWERVE_DRIVE_CURRENT_LIMIT = IS_SPIKE ? 35 : 0;
  public static final boolean SWERVE_DRIVE_LIMITS_ENABLE = IS_SPIKE ? true : true;

  public static final double SWERVE_DRIVE_KP = IS_SPIKE ? 0.1 : 0.1;
  public static final double SWERVE_DRIVE_KI = IS_SPIKE ? 0.0 : 0.0;
  public static final double SWERVE_DRIVE_KD = IS_SPIKE ? 0.0 : 0.0;
  public static final double SWERVE_DRIVE_KV = IS_SPIKE ? 0.12 : 0.117;
  public static final double SWERVE_DRIVE_KS = IS_SPIKE ? 0.0 : 0.0;

  public static final double STEER_MOTOR_LIMITS = IS_SPIKE ? 35 : 0.0;
  public static final boolean SWERVE_MOTOR_LIMITS_ENABLED = IS_SPIKE ? true : true;
  public static final PIDConstants SWERVE_TRANSLATION_PID = new PIDConstants(3, 0, 0);
  public static final PIDConstants SWERVE_ROTATION_PID = new PIDConstants(3, 0, 0);
  public static final boolean SWERVE_USE_FOC = IS_SPIKE ? false : false;

  public static final double SUPERSTRUCTURE_COLLISION_HEIGHT = IS_SPIKE ? 0.75 : 26;
  public static final Rotation2d SUPERSTRUCTURE_WRIST_RANGE =
      IS_SPIKE ? Rotation2d.fromDegrees(50) : Rotation2d.fromDegrees(13);

  public static final double ROBOT_CENTER_TO_FRONT = IS_SPIKE ? 0.0 : Units.inchesToMeters(17.5);

  private Config() {}
}
