// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ScoringLocation;
import frc.robot.States;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperstructureManager extends LifecycleSubsystem {
  private final SuperstructureMotionManager motionManager;
  private final IntakeSubsystem intake;
  private SuperstructureState goal = States.STOWED;
  private HeldGamePiece mode = HeldGamePiece.CUBE;
  private ScoringState scoringState = ScoringState.IDLE;

  public SuperstructureManager(SuperstructureMotionManager motionManager, IntakeSubsystem intake) {
    this.motionManager = motionManager;
    this.intake = intake;
  }

  public ScoringState getScoringState() {
    return scoringState;
  }

  public void set(SuperstructureState state) {
    goal = state;
    motionManager.set(goal.position);
    if (state == States.STOWED) {
      scoringState = ScoringState.IDLE;
    }
  }

  public boolean atGoal(SuperstructureState goal) {
    if (motionManager.atGoal(goal.position) && intake.atGoal(goal.intakeMode)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void enabledPeriodic() {
    if (goal.intakeNow || motionManager.atGoal(goal.position)) {
      intake.setMode(goal.intakeMode);
    }
    if (scoringState == ScoringState.ALIGNING && motionManager.atGoal(goal.position)) {
      scoringState = ScoringState.SCORING;
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

  public HeldGamePiece getMode() {
    return mode;
  }

  public Command getCommand(SuperstructureState state) {
    return Commands.runOnce(
            () -> this.set(state), motionManager.wrist, motionManager.elevator, intake)
        .andThen(Commands.waitUntil(() -> atGoal(state)));
  }

  public Command getScoreCommand() {
    return getManualScoreCommand(ScoringLocation.LOW).andThen(getCommand(States.STOWED));
  }

  public Command getManualScoreCommand(ScoringLocation scoringLocation) {
    Command scoreCommand;
    if (scoringLocation == ScoringLocation.LOW) {
      scoreCommand =
          Commands.either(
              getCommand(States.CUBE_NODE_LOW),
              getCommand(States.CONE_NODE_LOW),
              () -> intake.getGamePiece() == HeldGamePiece.CUBE);
    } else if (scoringLocation == ScoringLocation.MID) {
      scoreCommand =
          Commands.either(
              getCommand(States.CUBE_NODE_MID),
              getCommand(States.CONE_NODE_MID),
              () -> intake.getGamePiece() == HeldGamePiece.CUBE);
    } else {
      scoreCommand =
          Commands.either(
              getCommand(States.CUBE_NODE_HIGH),
              getCommand(States.CONE_NODE_HIGH),
              () -> intake.getGamePiece() == HeldGamePiece.CUBE);
    }
    return Commands.runOnce(() -> scoringState = ScoringState.ALIGNING).alongWith(scoreCommand);
  }

  public Command getIntakeCommand() {
    return Commands.either(
            getCommand(States.INTAKING_CUBE),
            getCommand(States.INTAKING_CONE),
            () -> mode == HeldGamePiece.CUBE)
        .andThen(getCommand(States.STOWED));
  }

  public Command setIntakeModeCommand(HeldGamePiece gamePiece) {
    return Commands.runOnce(() -> setIntakeMode(gamePiece));
  }

  public void setIntakeMode(HeldGamePiece gamePiece) {
    mode = gamePiece;
  }
}
