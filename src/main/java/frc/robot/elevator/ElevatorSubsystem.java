// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.config.Config;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private double goalPositionInInches = 0;
  private double sensorUnitsPerElevatorInch = (Config.ELEVATOR_GEARING * 2048) / (1.75 * Math.PI);
  private boolean isHoming = false;
  private double homingCurrent = 1.5;
  private boolean goToGoal = false;
  private static final double TOLERANCE = 0.5;

  public ElevatorSubsystem(TalonFX motor) {
    this.motor = motor;
    this.motor.setInverted(Config.ELEVATOR_INVERTED);

    // Set pid for slot 0
    this.motor.config_kF(0, Config.ELEVATOR_KF);
    this.motor.config_kP(0, Config.ELEVATOR_KP);
    this.motor.config_kI(0, Config.ELEVATOR_KI);
    this.motor.config_kD(0, Config.ELEVATOR_KD);
    // Set motion magic
    this.motor.configMotionCruiseVelocity(Config.ELEVATOR_CRUISE_VELOCITY);
    this.motor.configMotionAcceleration(Config.ELEVATOR_ACCELERATION);
    // Set current limiting
    this.motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 40, 1));
    // TODO: Verify behavior with this current limit enabled
    this.motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
  }

  public void startHoming() {
    this.isHoming = true;
  }

  public boolean isHoming() {
    return isHoming;
  }

  public double getHeight() {
    // Read talon sensor, convert to inches
    double sensorUnits = motor.getSelectedSensorPosition();
    double position = sensorUnits / sensorUnitsPerElevatorInch;
    return position;
  }

  public boolean atHeight(double height) {
    // Edit atHeight tolerance
    if (Math.abs(getHeight() - height) < TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  public void setGoalPosition(double goal) {
    // Save goal position
    this.goalPositionInInches =
        MathUtil.clamp(goal, Config.ELEVATOR_MIN_HEIGHT, Config.ELEVATOR_MAX_HEIGHT);
    goToGoal = true;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Elevator/Position", getHeight());
    Logger.getInstance().recordOutput("Elevator/Current", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Elevator/GoalPosition", goalPositionInInches);
    Logger.getInstance().recordOutput("Elevator/Homing", isHoming);
  }

  @Override
  public void enabledInit() {
    goToGoal = false;
  }

  @Override
  public void enabledPeriodic() {
    // Add homing sequence
    // Convert goal in inches to sensor units, and set motor
    if (isHoming) {
      motor.set(ControlMode.PercentOutput, -0.1);
      double current = motor.getSupplyCurrent();
      if (current > homingCurrent) {
        motor.setSelectedSensorPosition(0);
        this.isHoming = false;
        goalPositionInInches = 0;
      }
    } else if (goToGoal) {
      double goalPositionInSensorUnits = goalPositionInInches * sensorUnitsPerElevatorInch;
      motor.set(
          ControlMode.MotionMagic,
          goalPositionInSensorUnits,
          DemandType.ArbitraryFeedForward,
          Config.ELEVATOR_ARB_F);
      // motor.set(TalonFXControlMode.MotionMagic, goalPositionInSensorUnits);
    } else {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }
}
