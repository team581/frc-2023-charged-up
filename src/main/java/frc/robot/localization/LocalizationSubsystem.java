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
  // TODO: log vision pose
  // TODO: Use plain odometry class, log pose
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private Pose2d startPose;

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu) {
    this.swerve = swerve;
    this.imu = imu;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions(),
            new Pose2d(),
            // TODO: tune standard deviations
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS, imu.getRobotHeading(), swerve.getModulePositions());

    startPose = new Pose2d(new Translation2d(582, 15), imu.getRobotHeading());
  }

  @Override
  public void teleopInit() {
    resetPose(startPose, imu.getRobotHeading());
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

    double[] emptyArray = {};
    double[] rawPose =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(emptyArray);

    if (rawPose.length > 0) {
      Pose2d visionPose =
          new Pose2d(
              Units.metersToInches(rawPose[0]),
              Units.metersToInches(rawPose[1]),
              Rotation2d.fromDegrees(rawPose[4]));
      poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
      Logger.getInstance().recordOutput("Localization/VisionPose", visionPose);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle);
    poseEstimator.resetPosition(imu.getRobotHeading(), swerve.getModulePositions(), pose);
    odometry.resetPosition(imu.getRobotHeading(), swerve.getModulePositions(), pose);
  }
}
