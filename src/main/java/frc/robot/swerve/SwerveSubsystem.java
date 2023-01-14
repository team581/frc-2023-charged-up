// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.imu.ImuSubsystem;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem {
  private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
  private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
  private static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
  private static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
  public static final double MAX_VELOCITY = 4.5;
  public static final double MAX_ANGULAR_VELOCITY = 20;

  private final ImuSubsystem imu;
  private final SwerveModule frontRight;
  private final SwerveModule frontLeft;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  public SwerveSubsystem(
      ImuSubsystem imu,
      SwerveModule frontRight,
      SwerveModule frontLeft,
      SwerveModule backRight,
      SwerveModule backLeft) {
    this.imu = imu;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backRight = backRight;
    this.backLeft = backLeft;
  }

  public void periodic() {
    // ChassisSpeeds speeds = getChassisSpeeds;
    // ChassisSpeeds speeds = setChassisSpeeds(null, false);
  }

  // public ChassisSpeeds getChassisSpeeds() {
  //   final var frontLeftState = frontLeft.getState();
  //   final var frontRightState = frontRight.getState();
  //   final var backLeftState = backLeft.getState();
  //   final var backRightState = backRight.getState();
  //   return KINEMATICS.toChassisSpeeds(frontLeftState, frontRightState, backLeftState,
  // backRightState);
  // }

  // public swerveModuleState[] getModuleStaes() {
  //   return new swerveModuleState[] {
  //     frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
  //   }
  // }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean openLoop) {
    final var moduleStates = KINEMATICS.toSwerveModuleStates(speeds);
    Logger.getInstance().recordOutput("FrontLeft", moduleStates[0]);
    Logger.getInstance().recordOutput("FrontRight", moduleStates[1]);
    Logger.getInstance().recordOutput("BackLeft", moduleStates[2]);
    Logger.getInstance().recordOutput("BackRight", moduleStates[3]);
    // frontLeft.setDesiredState(moduleStates[0], openLoop);
    // frontRight.setDesiredState(moduleStates[1], openLoop);
    // backLeft.setDesiredState(moduleStates[2], openLoop);
    // backRight.setDesiredState(moduleStaes[3], openLoop);
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative) {
    Logger.getInstance().recordOutput("Sideways percentage", sidewaysPercentage);
    Logger.getInstance().recordOutput("Forward percentage", forwardPercentage);
    Logger.getInstance().recordOutput("Theta percentage", thetaPercentage);

    ChassisSpeeds myChassisSpeeds = new ChassisSpeeds(sidewaysPercentage, forwardPercentage, thetaPercentage);

    setChassisSpeeds(myChassisSpeeds, fieldRelative);
  }

  // public double getAngle() {
  //   return imu.getRobotHeading().getDegrees();
  // }
}
