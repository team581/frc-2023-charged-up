// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.ctre.phoenixpro.hardware.TalonFX;
import frc.robot.util.LifecycleSubsystem;

public class ArmSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  public ArmSubsystem(TalonFX motor) {
    this.motor = motor;

  }
}
