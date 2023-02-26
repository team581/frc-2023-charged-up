// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;
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

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private boolean visionWorking = false;

  private final InterpolatingTreeMap<Double, Double> visionStdLookup =
      new InterpolatingTreeMap<Double, Double>();

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

    // visionStdLookup.put(0.5, 0.1);
    // visionStdLookup.put(1.0, 0.5);
    // visionStdLookup.put(2.0, 1.0);
    visionStdLookup.put(0.752358, 0.005);
    visionStdLookup.put(1.016358, 0.0135);
    visionStdLookup.put(1.296358, 0.016);
    visionStdLookup.put(1.574358, 0.038);
    visionStdLookup.put(1.913358, 0.0515);
    visionStdLookup.put(2.184358, 0.0925);
    visionStdLookup.put(2.493358, 0.0695);
    visionStdLookup.put(2.758358, 0.046);
    visionStdLookup.put(3.223358, 0.1245);
    visionStdLookup.put(4.093358, 0.0815);
    visionStdLookup.put(4.726358, 0.193);
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

    boolean visionIsValid = false; // Indicates if vision is valid in this loop.

    LimelightResults results = LimelightHelpers.getLatestResults("");
    Pose2d currentVisionPose = results.targetingResults.getBotPose2d_wpiBlue();
    Pose2d angleAdjustedVisionPose =
        new Pose2d(currentVisionPose.getTranslation(), imu.getRobotHeading());

    double visionTimestamp =
        Timer.getFPGATimestamp()
            - ((results.targetingResults.latency_capture
                    + results.targetingResults.latency_jsonParse
                    + results.targetingResults.latency_pipeline)
                / 1000);

    double averageDistanceToIndividualFiducialTags = 0;
    double fiducialTagCount = 0;

    // Calculate average distance of each tag seen.
    if (results.targetingResults.valid
        && currentVisionPose.getX() != 0.0
        && currentVisionPose.getY() != 0.0) {
      for (int i = 0; i < results.targetingResults.targets_Fiducials.length; ++i) {
        Pose2d fiducialPose =
            results.targetingResults.targets_Fiducials[i].getTargetPose_RobotSpace2D();
        double fiducialDistanceAway =
            Math.sqrt(Math.pow(fiducialPose.getX(), 2) + Math.pow(fiducialPose.getY(), 2));
        averageDistanceToIndividualFiducialTags += fiducialDistanceAway;
        fiducialTagCount++;
      }
      averageDistanceToIndividualFiducialTags =
          averageDistanceToIndividualFiducialTags / fiducialTagCount;
      visionIsValid = true;
    }

    // Update pose estimator if vision is valid.
    if (visionIsValid) {
      // Adjust vision measurement standard deviation by average distance from tags.
      double stdForVision =
          visionStdLookup.get(averageDistanceToIndividualFiducialTags) / fiducialTagCount;
      poseEstimator.addVisionMeasurement(
          angleAdjustedVisionPose,
          visionTimestamp,
          VecBuilder.fill(stdForVision, stdForVision, Units.degreesToRadians(5)));
      Logger.getInstance().recordOutput("Localization/VisionPose", angleAdjustedVisionPose);
      visionWorking = true;
    }
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

  public boolean atPose(Pose2d goal) {
    // 3 degree rotation and 0.1 meter distance
    Pose2d pose = getPose();
    double distanceRelative = goal.getTranslation().getDistance(pose.getTranslation());

    Rotation2d rotationDifference = goal.getRotation().minus(pose.getRotation());
    if (distanceRelative < 0.1 && Math.abs(rotationDifference.getDegrees()) < 3) {
      return true;
    } else {
      return false;
    }
  }
}
