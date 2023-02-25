// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.autoscore.AutoScoreLocation;
import frc.robot.autoscore.GridKind;
import frc.robot.autoscore.NodeKind;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.Landmarks;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Autoscore extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SuperstructureManager superstructure;
  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final DriveController controller;
  private boolean autoScoreEnabled = false;
  private AutoScoreLocation autoScoreLocation;

  public Autoscore(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      SuperstructureManager superstructure,
      DriveController controller,
      IntakeSubsystem intake) {
    super(SubsystemPriority.AUTOSCORE);
    this.localization = localization;
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.controller = controller;
    this.intake = intake;
  }

  private AutoScoreLocation getAutoScoreLocation() {
    NodeKind node = controller.getAutoScoreNodeKind();
    List<Pose2d> grids =
        DriverStation.getAlliance() == Alliance.Red ? Landmarks.RED_GRIDS : Landmarks.BLUE_GRIDS;
    Pose2d nearestGrid = localization.getPose().nearest(grids);
    Pose2d scorePoint =
        new Pose2d(
            nearestGrid.getX(),
            nearestGrid.getY() - getScoreLocationOffset(node),
            nearestGrid.getRotation());

    if (nearestGrid == Landmarks.RED_GRID_LEFT || nearestGrid == Landmarks.BLUE_GRID_LEFT) {
      return new AutoScoreLocation(GridKind.LEFT, node, scorePoint);
    } else if (nearestGrid == Landmarks.RED_GRID_CENTER
        || nearestGrid == Landmarks.BLUE_GRID_CENTER) {
      return new AutoScoreLocation(GridKind.CENTER, node, scorePoint);
    } else {
      return new AutoScoreLocation(GridKind.RIGHT, node, scorePoint);
    }
  }

  private double getScoreLocationOffset(NodeKind node) {
    double multiplier = DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0;
    double offset = Config.ROBOT_CENTER_TO_FRONT;

    if (node == NodeKind.CENTER_MID_CUBE
        || node == NodeKind.CENTER_HIGH_CUBE
        || intake.getGamePiece() == HeldGamePiece.CUBE
        || node == NodeKind.LEFT_HIGH_CONE
        || node == NodeKind.RIGHT_HIGH_CONE) {
      // no offset needed for cubes or high cones
    } else if (node == NodeKind.RIGHT_MID_CONE
        || node == NodeKind.LEFT_MID_CONE
        || intake.getGamePiece() == HeldGamePiece.CONE) {
      offset = 0.127;
    }

    return multiplier * offset;
  }

  public boolean isAutoScoreEnabled() {
    return autoScoreEnabled;
  }

  public void setAutoScoreEnabled(boolean enabled) {
    autoScoreEnabled = enabled;
  }

  public Command getCommand() {
    return setEnabledCommand(true)
        .andThen(
            runOnce(
                () -> {
                  autoScoreLocation = getAutoScoreLocation();
                }))
        .andThen(swerve.goToPoseCommand(autoScoreLocation.pose, localization))
        .andThen(superstructure.getScoreCommand(autoScoreLocation.nodeHeight))
        .handleInterrupt(() -> superstructure.set(States.STOWED));
  }

  public Command getAutoAlignCommand() {
    return setEnabledCommand(true)
        .andThen(
            runOnce(
                () -> {
                  autoScoreLocation = getAutoScoreLocation();
                }))
        .andThen(swerve.goToPoseCommand(autoScoreLocation.pose, localization))
        .handleInterrupt(() -> superstructure.set(States.STOWED));
  }

  public Command setEnabledCommand(boolean enabled) {
    return runOnce(() -> setAutoScoreEnabled(enabled));
  }

  @Override
  public void robotPeriodic() {
    if (autoScoreLocation != null) {
      Logger.getInstance().recordOutput("AutoScore/GoalLocation/Pose", autoScoreLocation.pose);
      Logger.getInstance()
          .recordOutput("AutoScore/GoalLocation/Node", autoScoreLocation.node.toString());
      Logger.getInstance()
          .recordOutput(
              "AutoScore/GoalLocation/NodeHeight", autoScoreLocation.nodeHeight.toString());
      Logger.getInstance()
          .recordOutput("Autoscore/GoalLocation/Grid", autoScoreLocation.grid.toString());
    }
  }
}
