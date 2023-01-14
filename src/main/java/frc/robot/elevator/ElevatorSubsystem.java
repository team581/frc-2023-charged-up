// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import frc.robot.config.Config;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private double goalPositionInInches = 0;
  private double sensorUnitsPerElevatorInch = 0;

  public ElevatorSubsystem(TalonFX motor) {
    this.motor = motor;

    // Set pid for slot 0
    // Set motion magic
  }

  public double getPosition() {
   // Read talon sensor, convert to inches
   double sensorUnits = motor.getSelectedSensorPosition();
   double position = sensorUnits / sensorUnitsPerElevatorInch;
   return position;
  }

  public void setGoalPosition(double goal) {
    // Save goal position
    this.goalPositionInInches = goal;
  }

   @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Elevator/Position", getPosition());
  }

  @Override
  public void enabledPeriodic() {
    // Convert goal in inches to sensor units, and set motor
    double goalPositionInSensorUnits = goalPositionInInches * sensorUnitsPerElevatorInch;
    motor.set(ControlMode.Position, goalPositionInSensorUnits);
  }
}
