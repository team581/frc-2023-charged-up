// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.util.LifecycleSubsystem;

public class IntakeSubsystem extends LifecycleSubsystem {

  private final TalonFX motor;

  public IntakeSubsystem(TalonFX motor) {
    this.motor = motor;
  }
}
