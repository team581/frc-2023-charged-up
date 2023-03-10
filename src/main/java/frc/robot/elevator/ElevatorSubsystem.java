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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private double goalPositionInMeters = 0;
  private double sensorUnitsPerElevatorInch = (Config.ELEVATOR_GEARING * 2048) / (1.75 * Math.PI);
  private boolean isHoming = false;
  private double homingCurrent = 1.5;
  private boolean goToGoal = false;
  private static final double TOLERANCE = Units.inchesToMeters(0.5);

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);
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
    double position = Units.inchesToMeters(sensorUnits / sensorUnitsPerElevatorInch);
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
    this.goalPositionInMeters =
        MathUtil.clamp(goal, Config.ELEVATOR_MIN_HEIGHT, Config.ELEVATOR_MAX_HEIGHT);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Elevator/Position", Units.metersToInches(getHeight()));
    Logger.getInstance().recordOutput("Elevator/Current", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Elevator/GoalPosition", goalPositionInMeters);
    Logger.getInstance().recordOutput("Elevator/Homing", isHoming);
    Logger.getInstance().recordOutput("Elevator/test", Positions.CONE_NODE_MID.height);
  }

  @Override
  public void enabledPeriodic() {
    if (isHoming) {
      motor.set(ControlMode.PercentOutput, -0.1);
      double current = motor.getSupplyCurrent();
      if (current > homingCurrent) {
        motor.setSelectedSensorPosition(0);
        this.isHoming = false;
        goToGoal = true;
        setGoalPosition(Positions.STOWED.height);
      }
    } else if (goToGoal) {
      double goalPositionInSensorUnits =
          Units.metersToInches(goalPositionInMeters) * sensorUnitsPerElevatorInch;
      motor.set(
          ControlMode.MotionMagic,
          goalPositionInSensorUnits,
          DemandType.ArbitraryFeedForward,
          Config.ELEVATOR_ARB_F);
    } else {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming()).andThen(Commands.waitUntil(() -> isHoming() == false));
  }
}
