// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("Balance Auto", getBalanceAuto());
    autoChooser.addOption("RedRightThreeConeAuto", RedRightThreeConeAuto());

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

  private Command getBalanceAuto() {
    return followTrajectoryCommand(Paths.BALANCE_AUTO, true).withName("BalanceAutoCommand");
  }

  private Command RedRightThreeConeAuto() {
    return
        Commands.sequence(
        superstructure.setIntakeModeCommand(HeldGamePiece.CONE),
        Commands.runOnce(() -> intake.setPreloadForAutos(HeldGamePiece.CONE)),
        superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.RIGHT_NODE_TO_OPPOSITE_STAGING_MARK, true)
            .alongWith(Commands.waitSeconds(4))
            .andThen((superstructure.getIntakeCommand().withTimeout(3))),
        followTrajectoryCommand(Paths.RIGHT_STAGING_MARK_TO_RED_GRID_RIGHT_CENTER, false),
        superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.RIGHT_GRID_CENTER_TO_MIDDLE_RIGHT_STAGING_MARK, false)
            .alongWith(Commands.waitSeconds(4.75))
            .andThen(superstructure.getIntakeCommand().withTimeout(3)),
        followTrajectoryCommand(Paths.MIDDLE_RIGHT_STAGING_MARK_TO_RIGHT_GRID_LEFT, false),
        superstructure.getScoreCommand());
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
    return Commands.runOnce(
            () -> {
              if (isFirstPath) {
                localization.resetPose(traj.getInitialHolonomicPose(), imu.getRobotHeading());
              }
            })
        .andThen(swerve.getFollowTrajectoryCommand(traj, localization));
  }
}
