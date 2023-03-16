// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NodeHeight;
import frc.robot.Positions;
import frc.robot.States;
import frc.robot.autoscore.AutoScoreLocation;
import frc.robot.autoscore.GridKind;
import frc.robot.autoscore.NodeKind;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.Landmarks;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureManager extends LifecycleSubsystem {
  private final SuperstructureMotionManager motionManager;
  private final IntakeSubsystem intake;
  private SuperstructureState goal = States.STOWED;
  private HeldGamePiece mode = HeldGamePiece.CUBE;
  private ScoringState scoringState = ScoringState.IDLE;
  private LocalizationSubsystem localization;
  private boolean autoScoreEnabled = false;
  private IntakeMode manualIntakeMode;
  private SuperstructureState goalBeforeDunk;

  public SuperstructureManager(
      SuperstructureMotionManager motionManager,
      IntakeSubsystem intake,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.SUPERSTRUCTURE_MANAGER);

    this.motionManager = motionManager;
    this.intake = intake;
    this.localization = localization;
  }

  public ScoringState getScoringState() {
    return scoringState;
  }

  public void set(SuperstructureState state) {
    goal = state;
    manualIntakeMode = null;
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

  public boolean atPosition(SuperstructurePosition goal) {
    if (motionManager.atGoal(goal)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void enabledPeriodic() {
    motionManager.set(goal.position);

    if (manualIntakeMode != null) {
      intake.setMode(manualIntakeMode);
    } else {
      if (goal.intakeNow || motionManager.atGoal(goal.position)) {
        intake.setMode(goal.intakeMode);
      }
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

  public Command getCommand(Supplier<SuperstructureState> state) {
    return Commands.run(
            () -> this.set(state.get()), motionManager.wrist, motionManager.elevator, intake)
        .until(() -> atGoal(state.get()))
        .withName("SuperstructureCommand");
  }

  public Command getCommand(SuperstructureState state) {
    return getCommand(() -> state);
  }

  public Command getScoreCommand(NodeHeight scoringLocation, double delay) {
    return Commands.either(
        finishManualScoreCommand(),
        getManualScoreCommand(scoringLocation)
            .andThen(Commands.waitSeconds(delay))
            .andThen(finishManualScoreCommand())
            .andThen(getCommand(States.STOWED)),
        () ->
            goal.position.height >= Positions.CONE_NODE_MID.height
                || goal.position.height >= Positions.CUBE_NODE_MID.height);
  }

  public Command getManualScoreCommand(NodeHeight scoringLocation) {
    SuperstructureState cubeState;
    SuperstructureState coneState;

    if (scoringLocation == NodeHeight.LOW) {
      cubeState = States.CUBE_NODE_LOW;
      coneState = States.CONE_NODE_LOW;
    } else if (scoringLocation == NodeHeight.MID) {
      cubeState = States.CUBE_NODE_MID;
      coneState = States.CONE_NODE_MID;
    } else {
      cubeState = States.CUBE_NODE_HIGH;
      coneState = States.CONE_NODE_HIGH;
    }

    return Commands.runOnce(() -> scoringState = ScoringState.ALIGNING)
        .andThen(
            getCommand(
                () ->
                    mode == HeldGamePiece.CUBE
                        ? new SuperstructureState(cubeState.position, IntakeMode.STOPPED, true)
                        : new SuperstructureState(coneState.position, IntakeMode.STOPPED, true)))
        .withName("SuperstructureManualScore");
  }

  public Command getFloorIntakeIdleCommand() {
    return Commands.either(
            getFloorIntakeSpinningCommand()
                .unless(() -> intake.getGamePiece() == HeldGamePiece.CUBE),
            getCommand(States.INTAKING_CONE_FLOOR_IDLE)
                .unless(() -> intake.getGamePiece() == HeldGamePiece.CONE),
            () -> mode == HeldGamePiece.CUBE)
        .withName("SuperstructureFloorIntakeIdle");
  }

  public Command getFloorIntakeSpinningCommand() {
    return getCommand(
            () ->
                mode == HeldGamePiece.CUBE
                    ? States.INTAKING_CUBE_FLOOR_SPINNING
                    : States.INTAKING_CONE_FLOOR_SPINNING)
        .andThen(getCommand(States.STOWED))
        .withName("SuperstructureFloorIntakeSpinning");
  }

  public Command getShelfIntakeCommand() {
    return getCommand(
            () ->
                mode == HeldGamePiece.CUBE
                    ? States.INTAKING_CUBE_SHELF
                    : States.INTAKING_CONE_SHELF)
        .andThen(getCommand(States.STOWED))
        .withName("SuperstructureShelfIntake");
  }

  public AutoScoreLocation getAutoScoreLocation(NodeKind node) {
    List<Pose2d> grids = FmsSubsystem.isRedAlliance() ? Landmarks.RED_GRIDS : Landmarks.BLUE_GRIDS;
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

  public Command setIntakeModeCommand(HeldGamePiece gamePiece) {
    return Commands.runOnce(() -> setIntakeMode(gamePiece));
  }

  public void setIntakeMode(HeldGamePiece newMode) {
    if (mode != newMode) {
      intake.setGamePiece(HeldGamePiece.NOTHING);
    }

    mode = newMode;
  }

  public void setManualIntakeMode(IntakeMode manualIntakeMode) {
    this.manualIntakeMode = manualIntakeMode;
  }

  public Command setManualIntakeCommand(IntakeMode manualIntakeMode) {
    return Commands.runOnce(() -> setManualIntakeMode(manualIntakeMode));
  }

  // TODO: Ignore this command when the superstructure is STOWED
  public Command finishManualScoreCommand() {
    return Commands.waitUntil(() -> atPosition(goal.position))
        // Dunk motion when we are scoring a cone
        .andThen(
            () -> {
              goalBeforeDunk = goal;
            })
        .andThen(
            getCommand(
                    () ->
                        new SuperstructureState(
                            new SuperstructurePosition(
                                goalBeforeDunk.position.height + 0.5,
                                Rotation2d.fromDegrees(
                                    goalBeforeDunk.position.angle.getDegrees() + 15),
                                -1),
                            IntakeMode.OUTTAKE_CONE))
                .unless(
                    () ->
                        mode == HeldGamePiece.CUBE
                            || goal.position.height < Positions.CONE_NODE_MID.height))
        .andThen(
            Commands.either(
                Commands.runOnce(() -> setManualIntakeMode(IntakeMode.OUTTAKE_CUBE)),
                Commands.runOnce(() -> setManualIntakeMode(IntakeMode.OUTTAKE_CONE)),
                () -> mode == HeldGamePiece.CUBE))
        .andThen(Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.NOTHING))
        .andThen(Commands.runOnce(() -> scoringState = ScoringState.FINISHED_SCORING))
        .withName("SuperstructureFinishManualScore");
  }

  public Command getHomeCommand() {
    return Commands.runOnce(() -> set(States.STOWED))
        .alongWith(
            intake
                .getCommand(IntakeMode.STOPPED)
                .andThen(Commands.runOnce(() -> motionManager.wrist.resetHoming()))
                .andThen(motionManager.elevator.getHomeCommand())
                .andThen(motionManager.wrist.getHomeCommand()));
  }
}
