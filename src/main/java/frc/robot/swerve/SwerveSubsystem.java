// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends LifecycleSubsystem {
  private static final Translation2d FRONT_LEFT_LOCATION = Config.SWERVE_FRONT_LEFT_LOCATION;
  private static final Translation2d FRONT_RIGHT_LOCATION = Config.SWERVE_FRONT_RIGHT_LOCATION;
  private static final Translation2d BACK_LEFT_LOCATION = Config.SWERVE_BACK_LEFT_LOCATION;
  private static final Translation2d BACK_RIGHT_LOCATION = Config.SWERVE_BACK_RIGHT_LOCATION;
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
  public static final double MAX_VELOCITY_INCHES_PER_SECOND = 127;
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      MAX_VELOCITY_INCHES_PER_SECOND / 39.37;
  public static final double MAX_ANGULAR_VELOCITY = 30;

  private final ImuSubsystem imu;
  private final SwerveModule frontRight;
  private final SwerveModule frontLeft;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;
  private boolean doneResetting = false;

  private final PIDController xController =
      new PIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD);
  private final PIDController yController =
      new PIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD);
  private final PIDController thetaController =
      new PIDController(
          Config.SWERVE_ROTATION_PID.kP,
          Config.SWERVE_ROTATION_PID.kI,
          Config.SWERVE_ROTATION_PID.kD);

  private final ProfiledPIDController xProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD,
          new TrapezoidProfile.Constraints(0.25, 0.5));
  private final ProfiledPIDController yProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD,
          new TrapezoidProfile.Constraints(0.25, 0.5));
  private final ProfiledPIDController thetaProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_ROTATION_PID.kP,
          Config.SWERVE_ROTATION_PID.kI,
          Config.SWERVE_ROTATION_PID.kD,
          new TrapezoidProfile.Constraints(Math.PI * 2.0, Math.PI * 0.75));

  public SwerveSubsystem(
      ImuSubsystem imu,
      SwerveModule frontRight,
      SwerveModule frontLeft,
      SwerveModule backRight,
      SwerveModule backLeft) {
    super(SubsystemPriority.SWERVE);

    this.imu = imu;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backRight = backRight;
    this.backLeft = backLeft;

    thetaProfiledController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void disabledPeriodic() {
    frontRight.resetWheelAngle();
    frontLeft.resetWheelAngle();
    backRight.resetWheelAngle();
    backLeft.resetWheelAngle();
  }

  @Override
  public void robotPeriodic() {
    this.frontLeft.logValues();
    this.frontRight.logValues();
    this.backLeft.logValues();
    this.backRight.logValues();

    Logger.getInstance().recordOutput("Swerve/ModuleStates", getModuleStates());
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/X", chassisSpeeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/Y", chassisSpeeds.vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Swerve/ChassisSpeeds/Omega", chassisSpeeds.omegaRadiansPerSecond);
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
    setModuleStates(moduleStates, openLoop);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates, boolean openLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);
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
        new Translation2d(forwardPercentage, sidewaysPercentage)
            .times(MAX_VELOCITY_METERS_PER_SECOND);
    Rotation2d fieldRelativeHeading = imu.getRobotHeading();

    if (DriverStation.getAlliance() == Alliance.Red) {
      fieldRelativeHeading = fieldRelativeHeading.plus(Rotation2d.fromDegrees(180));
    }

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotTranslation.getX(),
            robotTranslation.getY(),
            thetaPercentage * MAX_ANGULAR_VELOCITY,
            fieldRelative ? fieldRelativeHeading : new Rotation2d());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setChassisSpeeds(KINEMATICS.toChassisSpeeds(moduleStates), openLoop);

    Logger.getInstance().recordOutput("Swerve/getX", robotTranslation.getX());
    Logger.getInstance().recordOutput("Swerve/getY", robotTranslation.getY());
  }

  public Command getDriveTeleopCommand(DriveController controller) {
    return Commands.run(
        () -> {
          if (!DriverStation.isTeleopEnabled()) {
            return;
          }

          boolean openLoop = false;

          driveTeleop(
              -controller.getSidewaysPercentage(),
              controller.getForwardPercentage(),
              controller.getThetaPercentage(),
              true,
              openLoop);
        },
        this);
  }

  public Command getFollowTrajectoryCommand(
      PathPlannerTrajectory traj, LocalizationSubsystem localization) {
    return new PPSwerveControllerCommand(
        traj,
        localization::getPose,
        SwerveSubsystem.KINEMATICS,
        // x controller
        xController,
        // y controller
        yController,
        // theta controller
        thetaController,
        states -> setModuleStates(states, false),
        false,
        this);
  }

  public Command goToPoseCommand(Pose2d goal, LocalizationSubsystem localization) {
    return resetPoseCommand(localization)
        .andThen(
            run(
                () -> {
                  ChassisSpeeds chassisSpeeds = getSpeedsForGoal(goal, localization);

                  setChassisSpeeds(chassisSpeeds, false);
                }))
        .until(() -> localization.atPose(goal));
  }

  public Command goToPoseContinuousCommand(
      Supplier<Pose2d> goal, LocalizationSubsystem localization) {
    return resetPoseCommand(localization)
        .andThen(
            run(
                () -> {
                  ChassisSpeeds chassisSpeeds = getSpeedsForGoal(goal.get(), localization);

                  setChassisSpeeds(chassisSpeeds, false);
                }))
        .until(() -> localization.atPose(goal.get()));
  }

  private Command resetPoseCommand(LocalizationSubsystem localization) {
    return Commands.runOnce(
        () -> {
          ChassisSpeeds speeds = getChassisSpeeds();
          xProfiledController.reset(localization.getPose().getX(), speeds.vxMetersPerSecond);
          xProfiledController.reset(localization.getPose().getY(), speeds.vyMetersPerSecond);
          xProfiledController.reset(
              localization.getPose().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
        });
  }

  private ChassisSpeeds getSpeedsForGoal(Pose2d goal, LocalizationSubsystem localization) {
    Logger.getInstance().recordOutput("Autoscore/TargetPose", goal);
    Pose2d pose = localization.getPose();

    double xVelocity = xController.calculate(pose.getX(), goal.getX());
    double yVelocity = yController.calculate(pose.getY(), goal.getY());
    double thetaVelocity =
        thetaController.calculate(
            pose.getRotation().getRadians(), goal.getRotation().getRadians());

    xVelocity = -xVelocity;
    yVelocity = -yVelocity;

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, thetaVelocity, pose.getRotation());
    return chassisSpeeds;
  }
}
