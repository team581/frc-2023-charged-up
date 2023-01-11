// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.config.Config;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final StatusSignalValue<Double> distance;
  private double goalDistance = 0;

  public ElevatorSubsystem(TalonFX motor) {
    this.motor = motor;
    this.distance = motor.getPosition();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Config.ELEVATOR_GEARING;
    motor.getConfigurator().apply(feedbackConfigs);
  }

  public double getDistance() {
    // TODO: Convert from rotations to inches
    return distance.getValue();
  }

  public void setDistance(double distance) {
    goalDistance = distance;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Elevator/Distance", getDistance());
  }

  @Override
  public void enabledPeriodic() {
    // TODO: Convert from inches to rotations
    var request = new PositionVoltage(goalDistance).withSlot(0);
    motor.setControl(request);
  }
}
