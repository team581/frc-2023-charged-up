// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.LifecycleSubsystem;

public class Autobalance extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final PIDController pidController = new PIDController(0.05, 0, 0);
  private boolean enabled = false;

  public Autobalance(SwerveSubsystem swerve, ImuSubsystem imu) {
    this.swerve = swerve;
    this.imu = imu;
  }

  public void setEnabled(boolean mode) {
    enabled = mode;
  }

  @Override
  public void enabledPeriodic() {
    if (enabled) {
      double velocity = pidController.calculate(imu.getPitch().getDegrees());

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, velocity, 0);

      swerve.setChassisSpeeds(chassisSpeeds, false);
    }
  }

  public boolean atGoal() {
    return Math.abs(imu.getPitch().getDegrees()) < 2.0;
  }

  public Command getCommand() {
    return Commands.runOnce(() -> setEnabled(true), swerve)
        .andThen(Commands.waitUntil(() -> atGoal()));
  }
}
