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
import frc.robot.ManualScoringLocation;
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
                "coneWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CONE)),
            Map.entry(
                "cubeWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CUBE)),
            Map.entry(
                "preloadCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(
                        Commands.runOnce(() -> intake.setPreloadForAutos(HeldGamePiece.CUBE)))),
            Map.entry(
                "preloadCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(
                        Commands.runOnce(() -> intake.setPreloadForAutos(HeldGamePiece.CONE)))),
            Map.entry(
                "scoreLow",
                superstructure
                    .getManualScoreCommand(ManualScoringLocation.LOW)
                    .andThen(superstructure.finishManualScoreCommand())),
            Map.entry(
                "scoreMid",
                superstructure
                    .getManualScoreCommand(ManualScoringLocation.MID)
                    .andThen(superstructure.finishManualScoreCommand())),
            Map.entry(
                "scoreHigh",
                superstructure
                    .getManualScoreCommand(ManualScoringLocation.HIGH)
                    .andThen(superstructure.finishManualScoreCommand())),
            Map.entry(
                "home",
                new ElevatorHomingCommand(elevator)
                    .andThen(new WristHomingCommand(wrist))
                    .alongWith(new IntakeCommand(intake, IntakeMode.STOPPED))),
            Map.entry(
                "intakeCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(superstructure.getFloorIntakeCommand())),
            Map.entry(
                "intakeCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(superstructure.getFloorIntakeCommand())),
            Map.entry("stow", superstructure.getCommand(States.STOWED)));

    autoBuilder =
        new SwerveAutoBuilder(
            localization::getPose,
            (pose) -> localization.resetPose(pose, pose.getRotation()),
            SwerveSubsystem.KINEMATICS,
            new PIDConstants(3, 0.0, 0.0),
            new PIDConstants(3, 0.0, 0.0),
            (states) -> swerve.setModuleStates(states, false),
            eventMap,
            false,
            swerve);

    autoChooser.addDefaultOption("Do nothing", getDoNothingAuto());
    autoChooser.addOption("red2LeftConeToBalance", red2LeftConeToBalance());
    autoChooser.addOption("red2RightConeToBalance", red2RightConeToBalance());
    autoChooser.addOption("red3RightConeAuto", red3RightConeAuto());
    autoChooser.addOption("red3LeftConeAuto", red3LeftConeAuto());
    autoChooser.addOption("redRightBalanceAuto", redRightBalanceAuto());
    autoChooser.addOption("testPath", testPath());

    // TODO: Don't run this at comps
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

  private Command red2RightConeToBalance() {
    return autoBuilder.fullAuto(Paths.RED_2_RIGHT_CONE_TO_BALANCE).andThen(() -> swerve.setChassisSpeeds(new ChassisSpeeds(), false));
  }

  private Command red2LeftConeToBalance() {
    return autoBuilder.fullAuto(Paths.RED_2_LEFT_CONE_TO_BALANCE);
  }

  private Command red3RightConeAuto() {
    return autoBuilder.fullAuto(Paths.RED_3_RIGHT_CONE_AUTO);
  }

  private Command red3LeftConeAuto() {
    return autoBuilder.fullAuto(Paths.RED_3_LEFT_CONE_AUTO);
  }

  private Command redRightBalanceAuto () {
    return autoBuilder.fullAuto(Paths.RED_RIGHT_BALANCE_AUTO);
  }

  private Command testPath() {
    return autoBuilder.fullAuto(Paths.TEST_PATH);
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
    return Commands.runOnce(
            () -> {
              if (isFirstPath) {
                localization.resetPose(traj.getInitialHolonomicPose(), imu.getRobotHeading());
              }
            })
        .andThen(swerve.getFollowTrajectoryCommand(traj, localization));
  }
}
