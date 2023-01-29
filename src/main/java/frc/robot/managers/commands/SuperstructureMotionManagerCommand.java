// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.managers.SuperstructureMotionManager;
import frc.robot.managers.SuperstructurePosition;

public class SuperstructureMotionManagerCommand extends CommandBase {
  private final SuperstructureMotionManager manager;
  private final SuperstructurePosition position;

  public SuperstructureMotionManagerCommand(
      SuperstructureMotionManager manager, SuperstructurePosition position) {
    this.manager = manager;
    this.position = position;
  }

  @Override
  public void initialize() {
    manager.set(position.height, position.angle);
  }

  @Override
  public boolean isFinished() {
    if (manager.atGoal(position)) {
      return true;
    } else {
      return false;
    }
  }
}
