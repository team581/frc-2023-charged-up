// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private IntakeMode mode = IntakeMode.STOPPED_INTAKING;

  private final TalonFX motor;

  public IntakeSubsystem(TalonFX motor) {
    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
  }

  @Override
  public void enabledPeriodic() {
    motor.set(TalonFXControlMode.PercentOutput, mode.percentage);
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
  }
}
