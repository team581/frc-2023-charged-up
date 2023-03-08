// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ManualScoringLocation;
import frc.robot.States;
import frc.robot.config.Config;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.Autobalance;
import frc.robot.managers.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.wrist.WristSubsystem;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static Command wrapAutoEvent(String commandName, Command command) {
    if (!Config.IS_DEVELOPMENT) {
      return command;
    }

    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static Map<String, Command> wrapAutoEventMap(Map<String, Command> eventMap) {
    if (!Config.IS_DEVELOPMENT) {
      return eventMap;
    }

    Map<String, Command> wrappedMap = new HashMap<>();
    for (Map.Entry<String, Command> entry : eventMap.entrySet()) {
      wrappedMap.put(entry.getKey(), wrapAutoEvent(entry.getKey(), entry.getValue()));
    }
    return wrappedMap;
  }

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final ImuSubsystem imu;
  private final SuperstructureManager superstructure;
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final IntakeSubsystem intake;
  private final SwerveAutoBuilder autoBuilder;
  private final Autobalance autoBalance;

  public Autos(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      ImuSubsystem imu,
      SuperstructureManager superstructure,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      IntakeSubsystem intake,
      Autobalance autoBalance) {
    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;
    this.superstructure = superstructure;
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;
    this.autoBalance = autoBalance;
    Map<String, Command> eventMap =
        Map.ofEntries(
            Map.entry(
                "coneWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CONE)),
            Map.entry(
                "cubeWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CUBE)),
            Map.entry(
                "preloadCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.CUBE)))),
            Map.entry(
                "preloadCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.CONE)))
                    .andThen(superstructure.setManualIntakeCommand(IntakeMode.INTAKE_CONE))
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(superstructure.setManualIntakeCommand(null))),
            Map.entry(
                "scoreLow",
                superstructure
                    .getScoreCommand(ManualScoringLocation.LOW)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "scoreMid",
                superstructure
                    .getScoreCommand(
                        Config.IS_SPIKE ? ManualScoringLocation.MID : ManualScoringLocation.LOW)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "scoreHigh",
                superstructure
                    .getScoreCommand(
                        Config.IS_SPIKE ? ManualScoringLocation.HIGH : ManualScoringLocation.LOW)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry("home", superstructure.getHomeCommand()),
            Map.entry(
                "intakeCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(superstructure.getFloorIntakeSpinningCommand())),
            Map.entry(
                "intakeCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(superstructure.getFloorIntakeSpinningCommand())),
            Map.entry("stow", superstructure.getCommand(States.STOWED)),
            Map.entry("autoBalance", autoBalance.getCommand().withName("AutoAutoBalance")));

    eventMap = wrapAutoEventMap(eventMap);

    autoBuilder =
        new SwerveAutoBuilder(
            localization::getPose,
            localization::resetPose,
            SwerveSubsystem.KINEMATICS,
            Config.SWERVE_TRANSLATION_PID,
            Config.SWERVE_ROTATION_PID,
            (states) -> swerve.setModuleStates(states, false, false),
            eventMap,
            false,
            swerve);

    if (Config.IS_DEVELOPMENT) {
      CommandScheduler.getInstance()
          .onCommandInitialize(
              command -> System.out.println("[COMMANDS] Starting command " + command.getName()));
      CommandScheduler.getInstance()
          .onCommandInterrupt(
              command -> System.out.println("[COMMANDS] Cancelled command " + command.getName()));
      CommandScheduler.getInstance()
          .onCommandFinish(
              command -> System.out.println("[COMMANDS] Finished command " + command.getName()));
    }

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("Blue long side 1", getBlueLongSideAuto());
    autoChooser.addOption("Blue short side 1", getBlueShortSideAuto());
    autoChooser.addOption("Blue long sie 1.5 balance", getBlueLongSide1_5Balance());
    autoChooser.addOption("Blue mid 1.5 balance", getBlueMid1_5Balance());
    autoChooser.addOption("Blue mid 1 balance", getBlueMid1Balance());
    autoChooser.addOption("Blue short side 2.5 balance", getBlueShortSide2_5Balance());
    autoChooser.addOption("Blue short side 2 balance", getBlueShortSide2Balance());

    autoChooser.addOption("Red long side 1.5 balance", getRedLongSide1_5BalanceAuto());
    autoChooser.addOption("Red long side 1", getRedLongSide1Auto());
    autoChooser.addOption("Red mid 1.5 balance", getRedMid1_5BalanceAuto());
    autoChooser.addOption("Red mid 1 balance", getRedMid1BalanceAuto());
    autoChooser.addOption("Red short side 1", getRedShortSide1());
    autoChooser.addOption("Red long side 2.5 balance", getRedLongSide2_5Balance());
    autoChooser.addOption("Red short side 2 balance", getRedShortSide2Balance());
    autoChooser.addOption("Red short side 2.5 balance", getRedShortSide2_5Balance());

    if (Config.IS_DEVELOPMENT) {
      PathPlannerServer.startServer(5811);
    }

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

  private Command getBlueLongSideAuto() {
    return autoBuilder.fullAuto(Paths.BLUE_LONG_SIDE_1_CONE);
  }

  private Command getBlueShortSideAuto() {
    return autoBuilder.fullAuto(Paths.BLUE_SHORT_SIDE_1_CONE);
  }

  private Command getBlueLongSide1_5Balance() {
    return autoBuilder
        .fullAuto(Paths.BLUE_LONG_SIDE_1_5_CONE_BALANCE)
        .andThen(autoBalance.getCommand());
  }

  private Command getBlueMid1_5Balance() {
    return autoBuilder.fullAuto(Paths.BLUE_MID_1_5_CONE_BALANCE).andThen(autoBalance.getCommand());
  }

  private Command getBlueMid1Balance() {
    return autoBuilder.fullAuto(Paths.BLUE_MID_1_CONE_BALANCE).andThen(autoBalance.getCommand());
  }

  private Command getBlueShortSide2_5Balance() {
    return autoBuilder
        .fullAuto(Paths.BLUE_SHORT_SIDE_2_5_CONE_BALANCE)
        .andThen(autoBalance.getCommand());
  }

  public Command getBlueShortSide2Balance() {
    return autoBuilder
        .fullAuto(Paths.BLUE_SHORT_SIDE_2_CONE_BALANCE)
        .andThen(autoBalance.getCommand())
        .withName("AutoBlueShortSide2ConeBalance");
  }

  private Command getRedLongSide1_5BalanceAuto() {
    return autoBuilder
        .fullAuto(Paths.RED_LONG_SIDE_1_5_CONE_BALANCE)
        .andThen(autoBalance.getCommand());
  }

  private Command getRedLongSide1Auto() {
    return autoBuilder.fullAuto(Paths.RED_LONG_SIDE_1_CONE);
  }

  private Command getRedMid1_5BalanceAuto() {
    return autoBuilder.fullAuto(Paths.RED_MID_1_5_CONE_BALANCE).andThen(autoBalance.getCommand());
  }

  private Command getRedMid1BalanceAuto() {
    return autoBuilder.fullAuto(Paths.RED_MID_1_CONE_BALANCE).andThen(autoBalance.getCommand());
  }

  private Command getRedShortSide1() {
    return autoBuilder.fullAuto(Paths.RED_SHORT_SIDE_1_CONE);
  }

  private Command getRedLongSide2_5Balance() {
    return autoBuilder
        .fullAuto(Paths.RED_LONG_SIDE_2_5_CONE_BALANCE)
        .andThen(autoBalance.getCommand());
  }

  private Command getRedShortSide2Balance() {
    return autoBuilder
        .fullAuto(Paths.RED_SHORT_SIDE_2_CONE_BALANCE)
        .andThen(autoBalance.getCommand())
        .withName("AutoRedShortSide2ConeBalance");
  }

  private Command getRedShortSide2_5Balance() {
    return autoBuilder
        .fullAuto(Paths.RED_SHORT_SIDE_2_5_CONE_BALANCE)
        .andThen(autoBalance.getCommand());
  }

  private CommandBase getDoNothingAuto() {
    return Commands.none().withName("DoNothingAuto");
  }

  public Command getAutoCommand() {
    Command command = autoChooser.get();

    if (command != null) {
      return command;
    }

    return getDoNothingAuto();
  }

  private Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return Commands.runOnce(
            () -> {
              if (isFirstPath) {
                localization.resetPose(traj.getInitialHolonomicPose());
              }
            })
        .andThen(swerve.getFollowTrajectoryCommand(traj, localization));
  }
}
