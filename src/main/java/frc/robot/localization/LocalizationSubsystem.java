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
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LimelightHelpers.LimelightResults;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private static final double MAX_APRILTAG_DISTANCE = Units.feetToMeters(15);
  private static final int RESET_ODOMETRY_FROM_VISION_SAMPLE_COUNT = 5;

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private boolean visionWorking = false;

  private final Pose2d startPose;

  private final CircularBuffer xVisionPoseBuffer =
      new CircularBuffer(RESET_ODOMETRY_FROM_VISION_SAMPLE_COUNT);
  private final CircularBuffer yVisionPoseBuffer =
      new CircularBuffer(RESET_ODOMETRY_FROM_VISION_SAMPLE_COUNT);

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.LOCALIZATION);

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
    // odometry.resetPosition(imu.getRobotHeading(), swerve.getModulePositions(), startPose);
  }

  @Override
  public void robotPeriodic() {
    update();

    Logger.getInstance().recordOutput("Localization/CombinedPose", getPose());
    Logger.getInstance().recordOutput("Localization/OdometryPose", odometry.getPoseMeters());
  }

  private void update() {
    poseEstimator.update(imu.getRobotHeading(), swerve.getModulePositions());
    odometry.update(imu.getRobotHeading(), swerve.getModulePositions());

    LimelightResults results = LimelightHelpers.getLatestResults("");
    Pose2d currentVisionPose = results.targetingResults.getBotPose2d_wpiBlue();
    // Pose2d angleAdjustedVisionPose = new Pose2d(currentVisionPose.getTranslation(),
    // imu.getRobotHeading());
    Pose2d angleAdjustedVisionPose = currentVisionPose;

    double visionTimestamp = Timer.getFPGATimestamp() - 0.02;
    double visionTimestampB =
        results.targetingResults.timestamp_RIOFPGA_capture
            - ((results.targetingResults.latency_capture
                    + results.targetingResults.latency_jsonParse
                    + results.targetingResults.latency_pipeline)
                / 1000);

    boolean visionIsValid = false;
    double averageDistanceToIndividualFiducialTags = 0;
    double fiducialTagCount = 0;

    if (results.targetingResults.valid
        && currentVisionPose.getX() != 0.0
        && currentVisionPose.getY() != 0.0) {
      for (int i = 0; i < results.targetingResults.targets_Fiducials.length; ++i) {
        Pose2d fiducialPose =
            results.targetingResults.targets_Fiducials[i].getRobotPose_FieldSpace2D();
        double fiducialDistanceAway = Math.sqrt(Math.pow(fiducialPose.getX(), 2) + Math.pow(fiducialPose.getY(), 2));
        averageDistanceToIndividualFiducialTags += fiducialDistanceAway;
        fiducialTagCount++;
      }
      averageDistanceToIndividualFiducialTags =
          averageDistanceToIndividualFiducialTags / fiducialTagCount;
      visionIsValid = true;
    }

    double trustMultiplier = Math.pow(averageDistanceToIndividualFiducialTags, 3);
    Logger.getInstance().recordOutput("Localization/FiducialLength", fiducialTagCount);
    Logger.getInstance()
        .recordOutput(
            "Localization/AverageFiducialDistanceAway", averageDistanceToIndividualFiducialTags);
    Logger.getInstance().recordOutput("Localization/StaticLatency", visionTimestamp);
    Logger.getInstance().recordOutput("Localization/DynamicLatency", visionTimestampB);

    if (visionIsValid) {
      xVisionPoseBuffer.addFirst(angleAdjustedVisionPose.getX());
      yVisionPoseBuffer.addFirst(angleAdjustedVisionPose.getY());
      poseEstimator.addVisionMeasurement(
          angleAdjustedVisionPose,
          visionTimestampB,
          VecBuilder.fill(
              0.05 * trustMultiplier,
              0.05 * trustMultiplier,
              Units.degreesToRadians(5 * trustMultiplier)));
      Logger.getInstance().recordOutput("Localization/VisionPose", angleAdjustedVisionPose);
      visionWorking = true;

      if (checkVisionPoseConsistent()) {
        odometry.resetPosition(
            imu.getRobotHeading(), swerve.getModulePositions(), angleAdjustedVisionPose);
        // resetPose(angleAdjustedVisionPose, imu.getRobotHeading());
      }
    }
  }

  private boolean checkVisionPoseConsistent() {
    double firstX = xVisionPoseBuffer.get(0);
    double firstY = yVisionPoseBuffer.get(0);
    boolean valid = true;
    for (int i = 1; i < xVisionPoseBuffer.size(); i++) {
      if (Math.abs(firstX - xVisionPoseBuffer.get(i)) > 0.025
          || Math.abs(firstY - yVisionPoseBuffer.get(i)) > 0.025) {
        valid = false;
      }
    }

    return valid;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    imu.setAngle(pose.getRotation());
    poseEstimator.resetPosition(pose.getRotation(), swerve.getModulePositions(), pose);
    odometry.resetPosition(pose.getRotation(), swerve.getModulePositions(), pose);
  }

  public void resetGyro(Rotation2d gyroAngle) {
    Pose2d pose = new Pose2d(getPose().getTranslation(), gyroAngle);
    resetPose(pose);
  }

  public boolean isVisionWorking() {
    return visionWorking;
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () ->
            resetGyro(
                Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 180 : 0)));
  }
}
