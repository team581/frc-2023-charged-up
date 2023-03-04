// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.Supplier;

public class AutoRotate extends LifecycleSubsystem {
  public static Rotation2d getGridAngle() {
    return FmsSubsystem.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(0);
  }

  public static Rotation2d getShelfAngle() {
    return FmsSubsystem.isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
  }

  private final SwerveSubsystem swerve;
  private Rotation2d angle;
  private boolean enabled;
  private final ImuSubsystem imu;

  public AutoRotate(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.AUTOROTATE);
    this.swerve = swerve;
    this.imu = imu;
  }

  public void setAngle(Rotation2d angle) {
    this.angle = angle;
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
    swerve.disableSnapToAngle();
  }

  @Override
  public void enabledPeriodic() {
    if (enabled) {
      swerve.setSnapToAngle(angle);
    }
  }

  public Command getCommand(Supplier<Rotation2d> angle) {
    return run(() -> setAngle(angle.get()))
        .until(() -> imu.atAngle(angle.get()))
        .handleInterrupt(() -> disable());
  }

  public Command getDisableCommand() {
    return runOnce(() -> disable());
  }
}
