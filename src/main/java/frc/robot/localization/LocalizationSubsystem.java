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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.LifecycleSubsystem;
import frc.robot.util.geometry.InchesPose2d;
import frc.robot.util.geometry.InchesTranslation2d;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private InchesPose2d startPose;

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

    startPose = new InchesPose2d(new InchesTranslation2d(582.0, 15.0), imu.getRobotHeading());
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
      InchesPose2d visionPose =
          new InchesPose2d(new Pose2d(rawPose[0], rawPose[1], Rotation2d.fromDegrees(rawPose[4])));
      poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
      Logger.getInstance().recordOutput("Localization/VisionPose", visionPose);
    }
  }

  public InchesPose2d getPose() {
    return new InchesPose2d(poseEstimator.getEstimatedPosition());
  }

  public void resetPose(InchesPose2d pose, Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle);
    poseEstimator.resetPosition(imu.getRobotHeading(), swerve.getModulePositions(), pose);
    odometry.resetPosition(imu.getRobotHeading(), swerve.getModulePositions(), pose);
  }
}