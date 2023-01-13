// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.HomingState;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final double GEARING = 999;
  private static final double HOMED_CURRENT = 999;
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);
  private final TalonFX motor;
  private Rotation2d goalAngle = new Rotation2d();
  private HomingState homingState = HomingState.NOT_HOMED;

  public WristSubsystem(TalonFX motor) {
    this.motor = motor;

    motor.config_kF(0, 0);
    motor.config_kP(0, 0);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(motor.getSelectedSensorPosition() / 2048.0 / GEARING);
  }

  public void setAngle(Rotation2d angle) {
    goalAngle = angle;
  }

  public boolean atAngle(Rotation2d angle) {
    var currentAngle = getAngle();
    return Math.abs(currentAngle.minus(angle).getDegrees()) < TOLERANCE.getDegrees();
  }

  public void startHoming() {
    homingState = HomingState.HOMING;
  }

  public boolean isHomed() {
    return homingState == HomingState.HOMED;
  }

  @Override
  public void enabledPeriodic() {
    if (homingState == HomingState.HOMING) {
      if (motor.getStatorCurrent() >= HOMED_CURRENT) {
        motor.set(TalonFXControlMode.PercentOutput, 0);
        motor.setSelectedSensorPosition(0);
        homingState = HomingState.HOMED;
      } else {
        motor.set(TalonFXControlMode.PercentOutput, 0.1);
      }
    } else {
      motor.set(TalonFXControlMode.Position, goalAngle.getRotations() * 2048.0 * GEARING);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
  }
}
