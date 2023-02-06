// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.geometry.InchesChassisSpeeds;
import frc.robot.util.geometry.InchesPose2d;
import frc.robot.util.geometry.InchesSwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final ImuSubsystem imu;

  public Autos(LocalizationSubsystem localization, SwerveSubsystem swerve, ImuSubsystem imu) {
    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("Balance Auto", getBalanceAuto());
    autoChooser.addOption("Drive Forward", getDriveForward());
    autoChooser.addOption("Two Cube Auto Score", getTwoAutoScore());

    PPSwerveControllerCommand.setLoggingCallbacks(
        (PathPlannerTrajectory activeTrajectory) -> {
          Logger.getInstance().recordOutput("Autos/CurrentTrajectory", activeTrajectory);
        },
        (Pose2d targetPose) -> {
          Logger.getInstance().recordOutput("Autos/TargetPose", targetPose);
        },
        (ChassisSpeeds rawSetpointSpeeds) -> {
          InchesChassisSpeeds chassisSpeeds = new InchesChassisSpeeds(rawSetpointSpeeds);

          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/X", chassisSpeeds.vxInchesPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Y", chassisSpeeds.vyInchesPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Omega", chassisSpeeds.omegaDegreesPerSecond);
        },
        (Translation2d translationError, Rotation2d rotationError) -> {
          Logger.getInstance()
              .recordOutput(
                  "Autos/TranslationError", new Pose2d(translationError, new Rotation2d()));
          Logger.getInstance().recordOutput("Autos/RotationError", rotationError.getDegrees());
        });
  }

  private Command getDriveForward() {
    return followTrajectoryCommand(Paths.DRIVE_FORWARD, true).withName("DriveForwardAutoCommand");
  }

  private Command getBalanceAuto() {
    return followTrajectoryCommand(Paths.BALANCE_AUTO, true).withName("BalanceAutoCommand");
  }

  private Command getTwoAutoScore() {
    return followTrajectoryCommand(Paths.TWO_AUTO_SCORE, true).withName("Two auto score");
  }

  private CommandBase getDoNothingAuto() {
    return Commands.none().withName("DoNothingAutoCommand");
  }

  public Command getAutoCommand() {
    Command command = autoChooser.get();

    if (command != null) {
      return command;
    }

    return getDoNothingAuto();
  }

  private Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                // gyroAngle should not be null
                localization.resetPose(
                    new InchesPose2d(traj.getInitialHolonomicPose()), imu.getRobotHeading());
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
            states -> swerve.setModuleStates(InchesSwerveModuleState.fromStates(states), false),
            false,
            swerve));
  }
}