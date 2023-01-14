// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ImuSubsystem {
  private final Pigeon2 imu;

  public ImuSubsystem(Pigeon2 imu) {
    this.imu = imu;
  }

  public void periodic() {
    SmartDashboard.putNumber("Gyro sensor", this.getRobotHeading().getDegrees());
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public void zero() {
    this.imu.setYaw(0);
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }
}
