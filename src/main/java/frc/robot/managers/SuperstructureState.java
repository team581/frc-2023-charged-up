// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import frc.robot.intake.IntakeMode;

public class SuperstructureState {
  public final SuperstructurePosition position;
  public final IntakeMode intakeMode;
  public final boolean intakeNow;

  public SuperstructureState(
      SuperstructurePosition position, IntakeMode intakeMode, boolean intakeNow) {
    this.position = position;
    this.intakeMode = intakeMode;
    this.intakeNow = intakeNow;
  }

  public SuperstructureState(SuperstructurePosition position, IntakeMode intakeMode) {
    this(position, intakeMode, false);
  }
}
