// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public Autos(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    this.localization = localization;
    this.swerve = swerve;

    autoChooser.addDefaultOption("Do nothing", Commands.none());
    autoChooser.addOption("Balance Auto", followTrajectoryCommand(Paths.BALANCE_AUTO, true));
  }

  public Command getAutoCommand() {
    Command command = autoChooser.get();

    if (command != null) {
      return command;
    }

    return Commands.none();
  }

  private Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                // gyroAngle should not be null
                localization.resetPose(traj.getInitialHolonomicPose(), null);
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            localization::getPose,
            SwerveSubsystem.KINEMATICS,
            // x controller
            new PIDController(5, 0, 0),
            // y controller
            new PIDController(5, 0, 0),
            // theta controller
            new PIDController(0.5, 0, 0),
            states -> swerve.setModuleStates(states, false),
            true,
            swerve));
  }
}
