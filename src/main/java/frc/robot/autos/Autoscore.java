// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
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
  private AutoScoreLocation autoScoreLocation =
      new AutoScoreLocation(GridKind.CENTER, NodeKind.LEFT_HIGH_CONE, new Pose2d());

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
            nearestGrid.getX() - getScoreLocationForwardOffset(),
            nearestGrid.getY() - getScoreLocationSidewaysOffset(),
            nearestGrid.getRotation().plus(Rotation2d.fromDegrees(180)));

    if (nearestGrid == Landmarks.RED_GRID_LEFT || nearestGrid == Landmarks.BLUE_GRID_LEFT) {
      return new AutoScoreLocation(GridKind.LEFT, node, scorePoint);
    } else if (nearestGrid == Landmarks.RED_GRID_CENTER
        || nearestGrid == Landmarks.BLUE_GRID_CENTER) {
      return new AutoScoreLocation(GridKind.CENTER, node, scorePoint);
    } else {
      return new AutoScoreLocation(GridKind.RIGHT, node, scorePoint);
    }
  }

  private double getScoreLocationForwardOffset() {
    NodeKind node = autoScoreLocation.node;
    double multiplier = DriverStation.getAlliance() == Alliance.Red ? 1.0 : -1.0;
    double offset = Config.ROBOT_CENTER_TO_FRONT;

    if (node == NodeKind.LEFT_MID_CONE || node == NodeKind.RIGHT_MID_CONE) {
      // Cone mid
      offset += 0.127;
    } else if (superstructure.getMode() == HeldGamePiece.CONE
        && (node == NodeKind.LEFT_HYBRID
            || node == NodeKind.CENTER_HYBRID
            || node == NodeKind.RIGHT_HYBRID)) {
      // Cone low
      offset += 0.127;
    }
    // No offset for cone high or any cubes

    return multiplier * offset;
  }

  private double getScoreLocationSidewaysOffset() {
    NodeKind node = autoScoreLocation.node;
    double multiplier = DriverStation.getAlliance() == Alliance.Red ? 1.0 : -1.0;
    double offset = 0;

    if (node == NodeKind.LEFT_HYBRID || node == NodeKind.LEFT_MID_CONE || node == NodeKind.LEFT_HIGH_CONE) {
      offset = Units.inchesToMeters(22);
    } else if (node == NodeKind.RIGHT_HYBRID || node == NodeKind.RIGHT_MID_CONE || node == NodeKind.RIGHT_HIGH_CONE) {
      offset = Units.inchesToMeters(-22);
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
    return new ProxyCommand(
        () ->
            setEnabledCommand(true)
                .andThen(runOnce(() -> autoScoreLocation = getAutoScoreLocation()))
                .andThen(swerve.goToPoseCommand(autoScoreLocation.pose, localization))
                .andThen(superstructure.getScoreCommand(autoScoreLocation.nodeHeight))
                .handleInterrupt(() -> superstructure.set(States.STOWED)));
  }

  public Command getAutoAlignCommand() {
    return new ProxyCommand(
        () ->
            setEnabledCommand(true)
                .andThen(
                    runOnce(
                        () -> {
                          autoScoreLocation = getAutoScoreLocation();
                        }))
                .andThen(swerve.goToPoseCommand(autoScoreLocation.pose, localization))
                .handleInterrupt(() -> superstructure.set(States.STOWED)));
  }

  public Command setEnabledCommand(boolean enabled) {
    return runOnce(() -> setAutoScoreEnabled(enabled));
  }

  @Override
  public void robotPeriodic() {
    // Remove this, it's temporary and just for logging
    autoScoreLocation = getAutoScoreLocation();

    Logger.getInstance().recordOutput("Autoscore/GoalLocation/Pose", autoScoreLocation.pose);
    Logger.getInstance()
        .recordOutput("Autoscore/GoalLocation/Node", autoScoreLocation.node.toString());
    Logger.getInstance()
        .recordOutput("Autoscore/GoalLocation/NodeHeight", autoScoreLocation.nodeHeight.toString());
    Logger.getInstance()
        .recordOutput("Autoscore/GoalLocation/Grid", autoScoreLocation.grid.toString());
    Logger.getInstance()
        .recordOutput("Autoscore/LocationOffset", getScoreLocationForwardOffset());
  }
}
