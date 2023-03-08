// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autobalance extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private boolean enabled = false;
  private static final double DRIVE_VELOCITY = -0.55;
  private static final double ANGLE_THRESHOLD = 10;
  private final LinearFilter autoBalanceFilter = LinearFilter.movingAverage(13);
  private Rotation2d averageRoll = new Rotation2d();

  public Autobalance(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.AUTOBALANCE);
    this.swerve = swerve;
    this.imu = imu;
  }

  public void setEnabled(boolean mode) {
    enabled = mode;
    if (!mode) {
      swerve.disableSnapToAngle();
    }
  }

  @Override
  public void robotPeriodic() {
    averageRoll = Rotation2d.fromDegrees(autoBalanceFilter.calculate(ANGLE_THRESHOLD));
  }

  @Override
  public void enabledPeriodic() {
    if (enabled) {
      Rotation2d goalAngle = Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180);

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(getDriveVelocity(), 0, 0);
      swerve.setSnapToAngle(goalAngle);
      swerve.setChassisSpeeds(chassisSpeeds, false);
    }
  }

  private double getDriveVelocity() {
    if (imu.getRoll().getDegrees() > ANGLE_THRESHOLD) {
      return DRIVE_VELOCITY * -1;
    } else if (imu.getRoll().getDegrees() < -ANGLE_THRESHOLD) {
      return DRIVE_VELOCITY;
    } else {
      return DRIVE_VELOCITY * 0;
    }
  }

  private boolean atGoal() {
    return averageRoll.getDegrees() < ANGLE_THRESHOLD
        && averageRoll.getDegrees() > -ANGLE_THRESHOLD;
  }

  public Command getCommand() {
    return Commands.run(() -> setEnabled(true), swerve)
        .until(() -> atGoal())
        .withTimeout(15.0)
        .andThen(runOnce(() -> setEnabled(false)))
        .andThen(runOnce(() -> swerve.getXSwerveCommand()))
        .handleInterrupt(() -> setEnabled(false));
  }
}
