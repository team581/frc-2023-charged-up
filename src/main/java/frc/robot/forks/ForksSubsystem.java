// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.forks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class ForksSubsystem extends LifecycleSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2); // placeholder
  private static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2);
  private final TalonFX motor;
  private ForksMode mode = ForksMode.STOWED;
  private HomingState homingState = HomingState.NOT_HOMED;

  public ForksSubsystem(TalonFX motor) {
    this.motor = motor;
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
    motor.config_kF(0, 0);
    motor.config_kP(0, 0.1);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
  }

  public void setMode(ForksMode mode) {
    this.mode = mode;
  }

  @Override
  public void enabledPeriodic() {
    if (homingState == HomingState.HOMED) {
      double goalPositionInSensorUnits = mode.angle.getRotations() * 2048 * Config.FORKS_GEARING;
      motor.set(ControlMode.Position, goalPositionInSensorUnits);
    } else if (homingState == HomingState.HOMING) {
      if (motor.getStatorCurrent() > 10) {
        homingState = HomingState.HOMED;
      } else {
        motor.set(ControlMode.PercentOutput, -0.1);
      }
    } else if (homingState == HomingState.NOT_HOMED) {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
        motor.getSelectedSensorPosition() / 2048.0 / Config.FORKS_GEARING);
  }

  public boolean atGoal(ForksMode mode) {
    return Math.abs(getAngle().minus(mode.angle).getDegrees()) < TOLERANCE.getDegrees();
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Forks/Current", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Forks/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Forks/Mode", mode.toString());
  }
}
