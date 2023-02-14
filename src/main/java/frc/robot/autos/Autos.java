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
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.ElevatorHomingCommand;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.wrist.WristSubsystem;
import frc.robot.wrist.commands.WristHomingCommand;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final ImuSubsystem imu;
  private SuperstructureManager superstructure;
  // private States states;
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;

  public Autos(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      ImuSubsystem imu,
      SuperstructureManager superstructure,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      IntakeSubsystem intake) {
    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;
    this.superstructure = superstructure;
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;

    // this.states = states;

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("Balance Auto", getBalanceAuto());
    autoChooser.addOption("Drive Forward", getDriveForward());
    autoChooser.addOption("BackRightForwardAutoCommand", backRightForwardAutoCommand());
    autoChooser.addOption("ScoreCubeAndBackConesCommand", scoreAndBackConesCommand());

    PPSwerveControllerCommand.setLoggingCallbacks(
        (PathPlannerTrajectory activeTrajectory) -> {
          Logger.getInstance().recordOutput("Autos/CurrentTrajectory", activeTrajectory);
        },
        (Pose2d targetPose) -> {
          Logger.getInstance().recordOutput("Autos/TargetPose", targetPose);
        },
        (ChassisSpeeds setpointSpeeds) -> {
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/X", setpointSpeeds.vxMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Y", setpointSpeeds.vyMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Omega", setpointSpeeds.omegaRadiansPerSecond);
        },
        (Translation2d translationError, Rotation2d rotationError) -> {
          Logger.getInstance()
              .recordOutput(
                  "Autos/TranslationError", new Pose2d(translationError, new Rotation2d()));
          Logger.getInstance().recordOutput("Autos/RotationError", rotationError.getDegrees());
        });
  }

  private Command getDriveForward() {
    return followTrajectoryCommand(Paths.DRIVE_BACKWARDS, true).withName("DriveForwardAutoCommand");
  }

  private Command getBalanceAuto() {
    return followTrajectoryCommand(Paths.BALANCE_AUTO, true).withName("BalanceAutoCommand");
  }

  private Command backRightForwardAutoCommand() {
    return followTrajectoryCommand(Paths.BACK_RIGHT_FORWARD, true)
        .withName("BackRightForwardAutoCommand");
  }

  private Command scoreAndBackConesCommand() {
    return superstructure
        .setIntakeModeCommand(HeldGamePiece.CONE)
        .andThen(() -> intake.setPreloadForAutos(HeldGamePiece.CONE))
        .andThen(superstructure.getScoreCommand())
        .andThen(
            followTrajectoryCommand(Paths.RIGHT_NODE_TO_OPPOSITE_PIECE, true)
                .alongWith(Commands.waitSeconds(4).andThen((superstructure.getIntakeCommand()))))
        .andThen(followTrajectoryCommand(Paths.RIGHT_PRELOAD_TO_RED_GRID_RIGHT_CENTER, false))
        .andThen(superstructure.getScoreCommand());
  }

  private CommandBase getDoNothingAuto() {
    return Commands.none().withName("DoNothingAutoCommand");
  }

  public Command getAutoCommand() {
    Command homingCommand =
        new ElevatorHomingCommand(elevator)
            .andThen(new WristHomingCommand(wrist))
            .alongWith(new IntakeCommand(intake, IntakeMode.STOPPED));
    Command command = autoChooser.get();

    if (command != null) {
      return homingCommand.andThen(command);
    }

    return homingCommand.andThen(getDoNothingAuto());
  }

  private Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                // gyroAngle should not be null
                localization.resetPose(traj.getInitialHolonomicPose(), imu.getRobotHeading());
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            localization::getOdometryPose,
            SwerveSubsystem.KINEMATICS,
            // x controller
            new PIDController(5, 0, 0),
            // y controller
            new PIDController(5, 0, 0),
            // theta controller
            new PIDController(1, 0, 0),
            states -> swerve.setModuleStates(states, false),
            false,
            swerve));
  }
}
