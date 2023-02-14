// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private static final double MAX_APRILTAG_DISTANCE = Units.feetToMeters(15);

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private Pose2d startPose;
  private boolean visionWorking = false;

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu) {
    this.swerve = swerve;
    this.imu = imu;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.0334, 0.1391, Units.degreesToRadians(30)));

    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS, imu.getRobotHeading(), swerve.getModulePositions());

    startPose =
        new Pose2d(
            new Translation2d(Units.inchesToMeters(582.0), Units.inchesToMeters(15.0)),
            imu.getRobotHeading());
  }

  @Override
  public void teleopInit() {
    resetPose(startPose, imu.getRobotHeading());
  }

  @Override
  public void teleopInit() {}

  @Override
  public void robotPeriodic() {
    update();

    Logger.getInstance().recordOutput("Localization/CombinedPose", getPose());
    Logger.getInstance().recordOutput("Localization/OdometryPose", odometry.getPoseMeters());
  }

  private void update() {
    poseEstimator.update(imu.getRobotHeading(), swerve.getModulePositions());
    odometry.update(imu.getRobotHeading(), swerve.getModulePositions());

    double[] emptyArray = {};
    double[] rawPose =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(emptyArray);

    boolean hasTargets =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;

    if (rawPose.length > 0 && hasTargets) {
      LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

      boolean isValid = true;

      for (int i = 0; i < llresults.targetingResults.targets_Retro.length; i++) {
        LimelightHelpers.LimelightTarget_Retro item = llresults.targetingResults.targets_Retro[i];
        Pose2d apriltagPose = item.getTargetPose_RobotSpace2D();
        if (Math.abs(apriltagPose.getX()) > MAX_APRILTAG_DISTANCE
            || Math.abs(apriltagPose.getY()) > MAX_APRILTAG_DISTANCE) {
          isValid = false;
          break;
        }
      }

      Pose2d visionPose = new Pose2d(rawPose[0], rawPose[1], imu.getRobotHeading());

      if (rawPose[0] == 0.0 && rawPose[1] == 0.0) {
        isValid = false;
      }

      if (isValid) {
        poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - 0.02);
        Logger.getInstance().recordOutput("Localization/VisionPose", visionPose);
        visionWorking = true;
      }
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle);
    poseEstimator.resetPosition(gyroAngle, swerve.getModulePositions(), pose);
    odometry.resetPosition(gyroAngle, swerve.getModulePositions(), pose);
  }

  public boolean isVisionWorking() {
    return visionWorking;
  }
}
