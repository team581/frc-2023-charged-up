// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public enum IntakeMode {
  INTAKING_CUBE(999),
  INTAKING_CONE(999),
  OUTTAKING_CUBE(999),
  OUTTAKING_CONE(999),
  STOPPED_INTAKING(0);

  public final double percentage;

  private IntakeMode(double percentage) {
    this.percentage = percentage;
  }
}
