// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ManualScoringLocation;
import frc.robot.States;
import frc.robot.autoscore.AutoScoreLocation;
import frc.robot.autoscore.GridKind;
import frc.robot.autoscore.NodeKind;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.Landmarks;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.LifecycleSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SuperstructureManager extends LifecycleSubsystem {
  private final SuperstructureMotionManager motionManager;
  private final IntakeSubsystem intake;
  private SuperstructureState goal = States.STOWED;
  private HeldGamePiece mode = HeldGamePiece.CUBE;
  private ScoringState scoringState = ScoringState.IDLE;
  private LocalizationSubsystem localization;
  private boolean autoScoreEnabled = false;

  public SuperstructureManager(
      SuperstructureMotionManager motionManager,
      IntakeSubsystem intake,
      LocalizationSubsystem localization) {
    this.motionManager = motionManager;
    this.intake = intake;
    this.localization = localization;
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
    return getManualScoreCommand(ManualScoringLocation.LOW).andThen(getCommand(States.STOWED));
  }

  public Command getManualScoreCommand(ManualScoringLocation scoringLocation) {
    Command scoreCommand;

    if (scoringLocation == ManualScoringLocation.LOW) {
      scoreCommand =
          Commands.either(
              getCommand(States.CUBE_NODE_LOW),
              getCommand(States.CONE_NODE_LOW),
              () -> intake.getGamePiece() == HeldGamePiece.CUBE);
    } else if (scoringLocation == ManualScoringLocation.MID) {
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

  public AutoScoreLocation getAutoScoreLocation(NodeKind node) {
    List<Pose2d> grids =
        DriverStation.getAlliance() == Alliance.Red ? Landmarks.RED_GRIDS : Landmarks.BLUE_GRIDS;
    Pose2d nearestGrid = localization.getPose().nearest(grids);
    if (nearestGrid == Landmarks.RED_GRID_LEFT || nearestGrid == Landmarks.BLUE_GRID_LEFT) {
      return new AutoScoreLocation(GridKind.LEFT, node, nearestGrid);
    } else if (nearestGrid == Landmarks.RED_GRID_CENTER
        || nearestGrid == Landmarks.BLUE_GRID_CENTER) {
      return new AutoScoreLocation(GridKind.CENTER, node, nearestGrid);
    } else {
      return new AutoScoreLocation(GridKind.RIGHT, node, nearestGrid);
    }
  }

  public boolean isAutoScoreEnabled() {
    return autoScoreEnabled;
  }

  public void setAutoScoreEnabled(boolean enabled) {
    autoScoreEnabled = enabled;
  }
}
