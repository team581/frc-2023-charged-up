// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends LifecycleSubsystem {
  private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
  private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
  private static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
  private static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);
  public static final SwerveDriveKinematics KINEMATICS =
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

  @Override
  public void disabledPeriodic() {
    // TODO: This causes constants loop overruns when it runs during disabled
    frontRight.resetWheelAngle();
    frontLeft.resetWheelAngle();
    backRight.resetWheelAngle();
    backLeft.resetWheelAngle();
  }

  @Override
  public void robotPeriodic() {
    ChassisSpeeds speeds = getChassisSpeeds();

    this.frontLeft.logValues();
    this.frontRight.logValues();
    this.backLeft.logValues();
    this.backRight.logValues();

    Logger.getInstance().recordOutput("Swerve/RotationalVelocity", speeds.omegaRadiansPerSecond);
    Logger.getInstance().recordOutput("Swerve/ForwardVelocity", speeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/HorizontalVelocity", speeds.vyMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/ModuleStates", getModuleStates());
  }

  public ChassisSpeeds getChassisSpeeds() {
    final var frontLeftState = frontLeft.getState();
    final var frontRightState = frontRight.getState();
    final var backLeftState = backLeft.getState();
    final var backRightState = backRight.getState();
    return KINEMATICS.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean openLoop) {
    final var moduleStates = KINEMATICS.toSwerveModuleStates(speeds);
    Logger.getInstance().recordOutput("Swerve/GoalModuleStates", moduleStates);
    frontLeft.setDesiredState(moduleStates[0], openLoop);
    frontRight.setDesiredState(moduleStates[1], openLoop);
    backLeft.setDesiredState(moduleStates[2], openLoop);
    backRight.setDesiredState(moduleStates[3], openLoop);
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative,
      boolean openLoop) {
    Logger.getInstance().recordOutput("Swerve/SidewaysPercentage", sidewaysPercentage);
    Logger.getInstance().recordOutput("Swerve/ForwardPercentage", forwardPercentage);
    Logger.getInstance().recordOutput("Swerve/ThetaPercentage", thetaPercentage);

    Translation2d robotTranslation =
        new Translation2d(forwardPercentage, -1 * sidewaysPercentage).times(MAX_VELOCITY);
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotTranslation.getX(),
            robotTranslation.getY(),
            -1 * thetaPercentage * MAX_ANGULAR_VELOCITY,
            fieldRelative ? imu.getRobotHeading() : new Rotation2d());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
    setChassisSpeeds(KINEMATICS.toChassisSpeeds(moduleStates), openLoop);
  }

  public double getAngle() {
    return imu.getRobotHeading().getDegrees();
  }
}
