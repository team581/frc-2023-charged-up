// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristSubsystem;

public class WristHomingCommand extends CommandBase {
  private final WristSubsystem wrist;

  /** Creates a new WristHomingCommand. */
  public WristHomingCommand(WristSubsystem wrist) {
    this.wrist = wrist;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.startHoming();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!wrist.isHoming()) {
      return true;
    } else {
      return false;
    }
  }
}
