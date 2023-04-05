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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.fms.FmsSubsystem;
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

  private Pose2d previousPose = new Pose2d();

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

    visionStdLookup.put(0.1, 0.8);
    visionStdLookup.put(0.5, 1.0);
    visionStdLookup.put(1.0, 2.0);
    visionStdLookup.put(2.0, 2.5);
  }

  @Override
  public void robotPeriodic() {
    update();

    Logger.getInstance().recordOutput("Localization/CombinedPose", getPose());
    Logger.getInstance().recordOutput("Localization/OdometryPose", odometry.getPoseMeters());
  }

  private void update() {
    try {
      poseEstimator.update(imu.getRobotHeading(), swerve.getModulePositions());
    } catch (Exception e) {
      System.err.println("Pose estimator threw while adding odometry measurement:");
      e.printStackTrace();
    }
    try {
      odometry.update(imu.getRobotHeading(), swerve.getModulePositions());
    } catch (Exception e) {
      System.err.println("Odometry threw while adding measurement:");
      e.printStackTrace();
    }

    boolean visionIsValid = false; // Indicates if vision is valid in this loop.

    if (Config.VISION_MODE == VisionMode.OFF) {
      visionWorking = false;
    } else if (Config.VISION_MODE == VisionMode.ENABLED_UNUSED) {
      Pose2d ntCurrentVisionPose = LimelightHelpers.getBotPose2d_wpiBlue("");
      Pose2d angleAdjustedVisionPose =
          new Pose2d(ntCurrentVisionPose.getTranslation(), imu.getRobotHeading());
      Logger.getInstance().recordOutput("Localization/VisionPose", angleAdjustedVisionPose);
      visionWorking = false;
    } else if (LimelightHelpers.getTV("") == 1) {
      Pose2d ntCurrentVisionPose = LimelightHelpers.getBotPose2d_wpiBlue("");
      if (previousPose.getX() != ntCurrentVisionPose.getX()
          && previousPose.getY() != ntCurrentVisionPose.getY()) {
        previousPose = ntCurrentVisionPose;

        LimelightResults results = LimelightHelpers.getLatestResults("");
        Pose2d angleAdjustedVisionPose =
            new Pose2d(ntCurrentVisionPose.getTranslation(), imu.getRobotHeading());

        double visionTimestamp =
            Timer.getFPGATimestamp()
                - ((results.targetingResults.latency_capture
                        + results.targetingResults.latency_jsonParse
                        + results.targetingResults.latency_pipeline)
                    / 1000);

        double averageDistanceToIndividualFiducialTags = 0;
        double fiducialTagCount = 0;

        // Calculate average distance of each tag seen.
        if (results.targetingResults.valid == 1
            && ntCurrentVisionPose.getX() != 0.0
            && ntCurrentVisionPose.getY() != 0.0) {
          for (int i = 0; i < results.targetingResults.targets_Fiducials.length; ++i) {
            Pose2d fiducialPose =
                results.targetingResults.targets_Fiducials[i].getTargetPose_RobotSpace2D();
            double fiducialDistanceAway =
                Math.sqrt(Math.pow(fiducialPose.getX(), 2) + Math.pow(fiducialPose.getY(), 2));
            averageDistanceToIndividualFiducialTags += fiducialDistanceAway;
            fiducialTagCount++;
          }
          visionIsValid = true;
        }

        // Update pose estimator if vision is valid.
        if (visionIsValid) {
          // Adjust vision measurement standard deviation by average distance from tags.
          double stdForVision =
              visionStdLookup.get(averageDistanceToIndividualFiducialTags) / fiducialTagCount;
          try {
            poseEstimator.addVisionMeasurement(
                angleAdjustedVisionPose,
                visionTimestamp,
                VecBuilder.fill(stdForVision, stdForVision, Units.degreesToRadians(360)));

          } catch (Exception e) {
            System.err.println("Pose estimator threw while adding vision measurement:");
            e.printStackTrace();
          }
          Logger.getInstance().recordOutput("Localization/VisionPose", angleAdjustedVisionPose);
          visionWorking = true;
        }
      }
    }

    if (!visionIsValid) {
      Logger.getInstance().recordOutput("Localization/VisionPose", new Pose2d());
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
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 180 : 0)));
  }

  public Command getZeroAwayCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180)));
  }

  public boolean atPose(Pose2d goal) {

    Pose2d pose = getPose();
    double distanceRelative = goal.getTranslation().getDistance(pose.getTranslation());

    Logger.getInstance().recordOutput("Localization/AtPoseGoal", goal);

    Rotation2d rotationDifference = goal.getRotation().minus(pose.getRotation());
    if (distanceRelative < 0.2 && Math.abs(rotationDifference.getDegrees()) < 5) {
      Logger.getInstance().recordOutput("Localization/AtPose", true);
      return true;
    } else {
      Logger.getInstance().recordOutput("Localization/AtPose", false);
      return false;
    }
  }
}
