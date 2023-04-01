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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends LifecycleSubsystem {
  private static final Translation2d FRONT_LEFT_LOCATION = Config.SWERVE_FRONT_LEFT_LOCATION;
  private static final Translation2d FRONT_RIGHT_LOCATION = Config.SWERVE_FRONT_RIGHT_LOCATION;
  private static final Translation2d BACK_LEFT_LOCATION = Config.SWERVE_BACK_LEFT_LOCATION;
  private static final Translation2d BACK_RIGHT_LOCATION = Config.SWERVE_BACK_RIGHT_LOCATION;
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
  public static final double MAX_VELOCITY =
      ((6080.0 / 60.0) / Config.SWERVE_DRIVE_GEARING_REDUCTION) * (Config.WHEEL_DIAMETER * Math.PI);
  public static final double MAX_ANGULAR_VELOCITY = 20;

  private final ImuSubsystem imu;
  private final SwerveModule frontRight;
  private final SwerveModule frontLeft;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;
  private boolean doneResetting = false;

  private boolean snapToAngle = false;
  private boolean xSwerveEnabled = false;

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
  private final PIDController snapThetaController =
      new PIDController(
          Config.SWERVE_ROTATION_SNAP_PID.kP,
          Config.SWERVE_ROTATION_SNAP_PID.kI,
          Config.SWERVE_ROTATION_SNAP_PID.kD);

  private final ProfiledPIDController xProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD,
          new TrapezoidProfile.Constraints(2.0, 1.5));
  private final ProfiledPIDController yProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_TRANSLATION_PID.kP,
          Config.SWERVE_TRANSLATION_PID.kI,
          Config.SWERVE_TRANSLATION_PID.kD,
          new TrapezoidProfile.Constraints(2.0, 1.5));
  private final ProfiledPIDController thetaProfiledController =
      new ProfiledPIDController(
          Config.SWERVE_ROTATION_PID.kP,
          Config.SWERVE_ROTATION_PID.kI,
          Config.SWERVE_ROTATION_PID.kD,
          new TrapezoidProfile.Constraints(Math.PI * 2.0, Math.PI * 0.75));
  private Rotation2d goalAngle = new Rotation2d();

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

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    snapThetaController.enableContinuousInput(-Math.PI, Math.PI);
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
    frontLeft.log();
    frontRight.log();
    backLeft.log();
    backRight.log();

    Logger.getInstance().recordOutput("Swerve/ModuleStates", getModuleStates());

    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/X", chassisSpeeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/Y", chassisSpeeds.vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Swerve/ChassisSpeeds/Omega", chassisSpeeds.omegaRadiansPerSecond);

    Translation2d t =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double velocity = t.getDistance(new Translation2d());
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/theta", t.getAngle().getRadians());
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/velocity", velocity);
    Logger.getInstance()
        .recordOutput(
            "Swerve/ChassisSpeeds/spinRatio", chassisSpeeds.omegaRadiansPerSecond / velocity);

    Logger.getInstance().recordOutput("Swerve/SnapToAngle/Goal", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Swerve/SnapToAngle/Enabled", snapToAngle);
  }

  @Override
  public void enabledPeriodic() {
    if (xSwerveEnabled) {
      xSwerve();
    }
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

  public void setSnapToAngle(Rotation2d angle) {
    goalAngle = angle;
    snapToAngle = true;
  }

  public void disableSnapToAngle() {
    snapToAngle = false;
  }

  public void setXSwerve(boolean xSwerve) {
    this.xSwerveEnabled = xSwerve;
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean openLoop) {
    if (xSwerveEnabled) {
      return;
    }

    if (snapToAngle) {
      speeds.omegaRadiansPerSecond =
          snapThetaController.calculate(imu.getRobotHeading().getRadians(), goalAngle.getRadians());
    }
    Logger.getInstance().recordOutput("Swerve/CommandedSpeeds/X", speeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/CommandedSpeeds/Y", speeds.vyMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/CommandedSpeeds/Omega", speeds.omegaRadiansPerSecond);

    // Twist computation.
    double lookAheadSeconds = 0.1;
    Pose2d target_pose =
        new Pose2d(
            lookAheadSeconds * speeds.vxMetersPerSecond,
            lookAheadSeconds * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(lookAheadSeconds * speeds.omegaRadiansPerSecond));
    Twist2d twist = (new Pose2d()).log(target_pose);
    speeds.vxMetersPerSecond = twist.dx / lookAheadSeconds;
    speeds.vyMetersPerSecond = twist.dy / lookAheadSeconds;
    speeds.omegaRadiansPerSecond = twist.dtheta / lookAheadSeconds; // omega should stay the same.
    // Kinematics to convert target chassis speeds to module states.
    final var moduleStates = KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates, openLoop, false);
  }

  public void setModuleStates(
      SwerveModuleState[] moduleStates, boolean openLoop, boolean skipJitterOptimization) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
    Logger.getInstance().recordOutput("Swerve/GoalModuleStates", moduleStates);
    frontLeft.setDesiredState(moduleStates[0], openLoop, skipJitterOptimization);
    frontRight.setDesiredState(moduleStates[1], openLoop, skipJitterOptimization);
    backLeft.setDesiredState(moduleStates[2], openLoop, skipJitterOptimization);
    backRight.setDesiredState(moduleStates[3], openLoop, skipJitterOptimization);
  }

  public void xSwerve() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
        },
        true,
        true);
  }

  public Command getXSwerveCommand() {
    return run(() -> setXSwerve(true));
  }

  public Command disableXSwerveCommand() {
    return runOnce(() -> setXSwerve(false));
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative,
      boolean openLoop) {
    Translation2d robotTranslation =
        new Translation2d(forwardPercentage, sidewaysPercentage).times(MAX_VELOCITY);
    Rotation2d fieldRelativeHeading = imu.getRobotHeading();

    if (FmsSubsystem.isRedAlliance()) {
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
  }

  public Command getDriveTeleopCommand(DriveController controller) {
    return Commands.run(
            () -> {
              if (!DriverStation.isTeleopEnabled()) {
                return;
              }

              boolean openLoop = true;

              driveTeleop(
                  -controller.getSidewaysPercentage(),
                  controller.getForwardPercentage(),
                  controller.getThetaPercentage(),
                  true,
                  openLoop);
            },
            this)
        .withName("SwerveDriveTeleop");
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
            states -> setModuleStates(states, false, false),
            false,
            this)
        .withName("SwerveFollowTrajectory");
  }

  // Create a command that accepts a Pose2d and drives to it using a PPHolonomicDriveController
  // The command should exit once it's at the pose
  public Command goToPoseCommand(Pose2d goal, LocalizationSubsystem localization) {
    return run(() -> {
          Logger.getInstance().recordOutput("AutoAlign/TargetPose", goal);
          Pose2d pose = localization.getPose();
          double xVelocity = xProfiledController.calculate(pose.getX(), goal.getX());
          double yVelocity = yProfiledController.calculate(pose.getY(), goal.getY());
          double thetaVelocity =
              thetaProfiledController.calculate(
                  pose.getRotation().getRadians(), goal.getRotation().getRadians());

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xVelocity, yVelocity, thetaVelocity, pose.getRotation());

          setChassisSpeeds(chassisSpeeds, false);
        })
        .until(
            () -> {
              // 3 degree rotation and 0.1 meter distance
              Pose2d pose = localization.getPose();
              double distanceRelative = goal.getTranslation().getDistance(pose.getTranslation());
              Rotation2d rotationDifference = goal.getRotation().minus(pose.getRotation());
              if (distanceRelative < 0.1 && Math.abs(rotationDifference.getDegrees()) < 3) {
                return true;
              } else {
                return false;
              }
            })
        .withName("SwerveGoToPose");
  }
}
