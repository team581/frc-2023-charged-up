// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperstructureManager extends LifecycleSubsystem {
  private final SuperstructureMotionManager motionManager;
  private final IntakeSubsystem intake;
  private SuperstructureState goal;

  public SuperstructureManager(SuperstructureMotionManager motionManager, IntakeSubsystem intake) {
    this.motionManager = motionManager;
    this.intake = intake;
  }

  public void set(SuperstructureState state) {
    goal = state;
  }

  // Create an atGoal() method which accepts a SuperstructureState and returns a boolean of whether
  // the superstructure is at the provided state

  @Override
  public void enabledPeriodic() {
    motionManager.set(goal.position);
    if (goal.intakeNow || motionManager.atGoal(goal.position)) {
      intake.setMode(goal.intakeMode);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance()
        .recordOutput("SuperstructureManager/Goal/IntakeMode", goal.intakeMode.toString());
    Logger.getInstance()
        .recordOutput("SuperstructureManager/Goal/ElevatorHeight", goal.position.height);
    Logger.getInstance()
        .recordOutput("SuperstructureManager/Goal/WristAngle", goal.position.angle.getDegrees());
    Logger.getInstance().recordOutput("SuperstructureManager/Goal/IntakeNow", goal.intakeNow);
  }
}
