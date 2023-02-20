// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.States;
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
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final ImuSubsystem imu;
  private SuperstructureManager superstructure;
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;
  private final SwerveAutoBuilder autoBuilder;

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
    Map<String, Command> eventMap =
        Map.ofEntries(
            Map.entry(
                "intakeCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(superstructure.getIntakeCommand())),
            Map.entry(
                "intakeCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(superstructure.getIntakeCommand())),
            Map.entry("score", superstructure.getScoreCommand()),
            Map.entry("stowSuperstructure", superstructure.getCommand(States.STOWED)));

    autoBuilder =
        new SwerveAutoBuilder(
            localization::getPose,
            (pose) -> localization.resetPose(pose, pose.getRotation()),
            SwerveSubsystem.KINEMATICS,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(1.1, 0.0, 0.0),
            (states) -> swerve.setModuleStates(states, false),
            eventMap,
            false,
            swerve);

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("Balance Auto", getBalanceAuto());
    autoChooser.addOption("RedRightThreeConeAuto", RedRightThreeConeAuto());
    autoChooser.addOption("RedLeftThreeConeAuto", RedLeftThreeConeAuto());
    autoChooser.addOption("GUIPath", DrivingGUIPath());
    autoChooser.addOption("GUIPATH2", DrivingGUIPath2());
    autoChooser.addOption("GUIFullAuto", getDrivingAuto());

    PathPlannerServer.startServer(5811);

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
    return Commands.sequence(
        // superstructure.setIntakeModeCommand(HeldGamePiece.CONE),
        // Commands.runOnce(() -> intake.setPreloadForAutos(HeldGamePiece.CONE)),
        // superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.RIGHT_GRID_RIGHT_TO_FAR_RIGHT_STAGING_MARK, true)
            .alongWith(Commands.waitSeconds(3.5))
            .andThen((superstructure.getIntakeCommand().withTimeout(3))),
        followTrajectoryCommand(Paths.FAR_RIGHT_STAGING_MARK_TO_RED_GRID_RIGHT_LEFT, false),
        superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.RIGHT_GRID_LEFT_TO_MIDDLE_RIGHT_STAGING_MARK, false)
            .alongWith(Commands.waitSeconds(4.75))
            .andThen(superstructure.getIntakeCommand().withTimeout(3)));
  }

  private Command DrivingGUIPath() {
    return Commands.sequence(followTrajectoryCommand(Paths.GUI_TEST, true));
  }

  private Command getDrivingAuto() {
    return autoBuilder.fullAuto(Paths.GUI_FULL_AUTO);
  }

  private Command DrivingGUIPath2() {
    return Commands.sequence(followTrajectoryCommand(Paths.GUI_TEST_2, true));
  }

  private Command RedLeftThreeConeAuto() {
    return Commands.sequence(
        superstructure.setIntakeModeCommand(HeldGamePiece.CONE),
        Commands.runOnce(() -> intake.setPreloadForAutos(HeldGamePiece.CONE)),
        superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.LEFT_GRID_LEFT_TO_FAR_LEFT_STAGING_MARK, true)
            .alongWith(Commands.waitSeconds(4))
            .andThen((superstructure.getIntakeCommand().withTimeout(3))),
        followTrajectoryCommand(Paths.FAR_LEFT_STAGING_MARK_TO_LEFT_GRID_CENTER, false),
        superstructure.getScoreCommand(),
        followTrajectoryCommand(Paths.LEFT_GRID_RIGHT_TO_LEFT_STAGING_MARK, false)
            .alongWith(Commands.waitSeconds(4.75))
            .andThen(superstructure.getIntakeCommand().withTimeout(3)));
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
