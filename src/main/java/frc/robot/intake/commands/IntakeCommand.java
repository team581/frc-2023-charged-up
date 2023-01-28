// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {
  private final IntakeSubsystem intakeSubsystem;
  private final IntakeMode mode;

  public IntakeCommand(IntakeSubsystem intake, IntakeMode mode) {
    intakeSubsystem = intake;
    this.mode = mode;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setMode(mode);
  }
}
