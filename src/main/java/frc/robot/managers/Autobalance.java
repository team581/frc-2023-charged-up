// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autobalance extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private boolean enabled = false;
  private double driveVelocity = 0.35;
  private double angleThreshold = 10;
  private final LinearFilter autoBalanceFilter = LinearFilter.movingAverage(13);
  private Rotation2d averageRoll = new Rotation2d();
  private PIDController yawController = new PIDController(0.05, 0, 0);

  public Autobalance(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.AUTOBALANCE);
    this.swerve = swerve;
    this.imu = imu;
  }

  public void setEnabled(boolean mode) {
    enabled = mode;
  }

  @Override
  public void robotPeriodic() {
    averageRoll = Rotation2d.fromDegrees(autoBalanceFilter.calculate(angleThreshold));
  }

  @Override
  public void enabledPeriodic() {
    if (enabled) {
      ChassisSpeeds chassisSpeeds =
          new ChassisSpeeds(
              getDriveVelocity(),
              0,
              yawController.calculate(imu.getRobotHeading().getDegrees(), 0));
      swerve.setChassisSpeeds(chassisSpeeds, false);
    }
  }

  private double getDriveVelocity() {
    if (imu.getRoll().getDegrees() > angleThreshold) {
      return driveVelocity * -1;
    } else if (imu.getRoll().getDegrees() < -angleThreshold) {
      return driveVelocity;
    } else {
      return driveVelocity * 0;
    }
  }

  private boolean atGoal() {
    return averageRoll.getDegrees() < angleThreshold && averageRoll.getDegrees() > -angleThreshold;
  }

  public Command getCommand() {
    return Commands.run(() -> setEnabled(true), swerve).until(() -> atGoal());
  }
}
