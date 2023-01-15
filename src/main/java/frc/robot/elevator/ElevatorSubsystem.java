// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private double goalPositionInInches = 0;
  private double sensorUnitsPerElevatorInch = (2048 * 1.75 * Math.PI) / 20;
  private boolean isHoming = false;
  private double homingCurrent = 5;

  public ElevatorSubsystem(TalonFX motor) {
    this.motor = motor;

    // Set pid for slot 0
    // Set motion magic
  }

  public void startHoming() {
    this.isHoming = true;
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
    Logger.getInstance().recordOutput("Elevator/Current", motor.getSupplyCurrent());
  }

  @Override
  public void enabledPeriodic() {
    // Add homing sequence
    // Convert goal in inches to sensor units, and set motor

    if (isHoming) {
      motor.set(ControlMode.PercentOutput, -0.01);
      double current = motor.getSupplyCurrent();
      if (current > homingCurrent) {
        motor.setSelectedSensorPosition(0);
        this.isHoming = false;
        goalPositionInInches = 0;
      }
    } else {
      double goalPositionInSensorUnits = goalPositionInInches * sensorUnitsPerElevatorInch;
      motor.set(ControlMode.Position, goalPositionInSensorUnits);
    }
  }
}
