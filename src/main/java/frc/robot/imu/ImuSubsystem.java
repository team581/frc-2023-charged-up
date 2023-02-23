// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends LifecycleSubsystem {
  private final Pigeon2 imu;

  public ImuSubsystem(Pigeon2 imu) {
    this.imu = imu;
  }

  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Gyro sensor", this.getRobotHeading().getDegrees());
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public void zero() {
    // TODO: This should use setAngle()
    this.imu.setYaw(0);
  }

  public void setAngle(Rotation2d zeroAngle) {
    this.imu.setYaw(zeroAngle.getDegrees());
  }

  public Command getZeroCommand() {
    return Commands.runOnce(() -> zero());
  }
}
