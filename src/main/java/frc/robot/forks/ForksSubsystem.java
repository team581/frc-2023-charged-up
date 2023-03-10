// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.forks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleStage;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ForksSubsystem extends LifecycleSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2); // placeholder
  private static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 40, 40, 0.2);
  // make a SAFE_CURRENT_LIMIT, which is 10, 15, 0.2
  // in testinit, use safe current limit
  // in teleop init or auto init, use regular current limit
  private static final SupplyCurrentLimitConfiguration SAFE_CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 10, 15, 0.2);

  private final TalonFX motor;
  private ForksMode mode = ForksMode.STOPPED;
  private HomingState homingState = HomingState.NOT_HOMED;

  public ForksSubsystem(TalonFX motor) {
    super(SubsystemPriority.FORKS);

    this.motor = motor;
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
    motor.configForwardSoftLimitThreshold(0);
  }

  public void setMode(ForksMode mode) {
    this.mode = mode;
  }

  @Override
  public void testInit() {
    motor.configForwardSoftLimitEnable(false);

    motor.configSupplyCurrentLimit(SAFE_CURRENT_LIMIT);
  }

  @Override
  public void teleopInit() {
    motor.configForwardSoftLimitEnable(true);

    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
  }

  @Override
  public void autonomousInit() {
    motor.configForwardSoftLimitEnable(true);

    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
  }

  @Override
  public void enabledPeriodic() {
    if (mode == ForksMode.UP) {
      if (LifecycleSubsystemManager.getStage() == LifecycleStage.TEST) {
        motor.set(ControlMode.PercentOutput, 0.2);
      } else {
        motor.set(ControlMode.PercentOutput, 1);
      }
    } else if (mode == ForksMode.DOWN) {
      if (LifecycleSubsystemManager.getStage() == LifecycleStage.TEST) {
        motor.set(ControlMode.PercentOutput, -0.2);
      } else {
        motor.set(ControlMode.PercentOutput, -1);
      }
    } else {
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
    Logger.getInstance().recordOutput("Forks/HomingState", homingState.toString());
  }

  public Command getCommand(ForksMode newGoal) {
    return Commands.runOnce(() -> setMode(newGoal), this);
  }
}
