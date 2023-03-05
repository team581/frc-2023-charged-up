// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends LifecycleSubsystem {
  private final Pigeon2 imu;

  public ImuSubsystem(Pigeon2 imu) {
    super(SubsystemPriority.IMU);

    this.imu = imu;
  }

  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Imu/RobotHeading", this.getRobotHeading().getDegrees());
    Logger.getInstance().recordOutput("Imu/RobotHeadingRadians", this.getRobotHeading().getRadians());
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(imu.getRoll());
  }

  public void zero() {
    // TODO: This should use setAngle()
    this.imu.setYaw(0);
    this.imu.configMountPoseRoll(this.imu.getRoll());
  }

  public void setAngle(Rotation2d zeroAngle) {
    this.imu.setYaw(zeroAngle.getDegrees());
  }

  public boolean atAngle(Rotation2d angle) {
    return Math.abs(getRobotHeading().minus(angle).getDegrees()) < 3;
  }

  public Command getZeroCommand() {
    return Commands.runOnce(() -> zero());
  }
}
