// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorSubsystem;

public class ElevatorHomingCommand extends CommandBase {
  private final ElevatorSubsystem elevator;

  /** Creates a new ElevatorHomingCommand. */
  public ElevatorHomingCommand(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.startHoming();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (!elevator.isHoming()) {
    return true;
   } else {
    return false;
   }
  }
}
