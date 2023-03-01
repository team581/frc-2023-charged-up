// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ManualScoringLocation;
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
    motionManager.set(goal.position);
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

  public Command getCommand(SuperstructureState state) {
    return Commands.runOnce(
            () -> this.set(state), motionManager.wrist, motionManager.elevator, intake)
        .andThen(Commands.waitUntil(() -> atGoal(state)));
  }

  public Command getScoreCommand(ManualScoringLocation scoringLocation) {
    SuperstructureState cubeState;
    SuperstructureState coneState;

    if (scoringLocation == ManualScoringLocation.LOW) {
      cubeState = States.CUBE_NODE_LOW;
      coneState = States.CONE_NODE_LOW;
    } else if (scoringLocation == ManualScoringLocation.MID) {
      cubeState = States.CUBE_NODE_MID;
      coneState = States.CONE_NODE_MID;
    } else {
      cubeState = States.CUBE_NODE_HIGH;
      coneState = States.CONE_NODE_HIGH;
    }

    return Commands.either(
        finishManualScoreCommand(),
        Commands.either(
                getCommand(cubeState),
                getCommand(coneState),
                () -> intake.getGamePiece() == HeldGamePiece.CUBE)
            .andThen(getCommand(States.STOWED)),
        () ->
            goal.position.height >= Positions.CUBE_NODE_MID.height
                || goal.position.height >= Positions.CONE_NODE_MID.height);
  }

  public Command getManualScoreCommand(ManualScoringLocation scoringLocation) {
    SuperstructureState cubeState;
    SuperstructureState coneState;

    if (scoringLocation == ManualScoringLocation.LOW) {
      cubeState = States.CUBE_NODE_LOW;
      coneState = States.CONE_NODE_LOW;
    } else if (scoringLocation == ManualScoringLocation.MID) {
      cubeState = States.CUBE_NODE_MID;
      coneState = States.CONE_NODE_MID;
    } else {
      cubeState = States.CUBE_NODE_HIGH;
      coneState = States.CONE_NODE_HIGH;
    }

    return Commands.runOnce(() -> scoringState = ScoringState.ALIGNING)
        .andThen(
            Commands.either(
                getCommand(new SuperstructureState(cubeState.position, IntakeMode.STOPPED, true)),
                getCommand(new SuperstructureState(coneState.position, IntakeMode.STOPPED, true)),
                () -> intake.getGamePiece() == HeldGamePiece.CUBE));
  }

  public Command getFloorIntakeIdleCommand() {
    return Commands.either(
        getFloorIntakeSpinningCommand(),
        getCommand(States.INTAKING_CONE_FLOOR_IDLE)
            .unless(() -> intake.getGamePiece() == HeldGamePiece.CONE),
        () -> mode == HeldGamePiece.CUBE);
  }

  public Command getFloorIntakeSpinningCommand() {
    return Commands.either(
            getCommand(States.INTAKING_CUBE_FLOOR_SPINNING),
            getCommand(States.INTAKING_CONE_FLOOR_SPINNING),
            () -> mode == HeldGamePiece.CUBE)
        .andThen(getCommand(States.STOWED));
  }

  public Command getShelfIntakeCommand() {
    return Commands.either(
            getCommand(States.INTAKING_CUBE_SHELF),
            getCommand(States.INTAKING_CONE_SHELF),
            () -> mode == HeldGamePiece.CUBE)
        .andThen(getCommand(States.STOWED));
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

  public void setIntakeMode(HeldGamePiece gamePiece) {
    mode = gamePiece;
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
        .andThen(
            () ->
                motionManager.set(
                    new SuperstructurePosition(
                        goal.position.height + 0.5,
                        Rotation2d.fromDegrees(goal.position.angle.getDegrees() + 15),
                        -1)))
        .andThen(
            Commands.either(
                Commands.runOnce(() -> setManualIntakeMode(IntakeMode.OUTTAKE_CUBE)),
                Commands.runOnce(() -> setManualIntakeMode(IntakeMode.OUTTAKE_CONE)),
                () -> intake.getGamePiece() == HeldGamePiece.CUBE));
  }
}
