// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.autoscore.AutoScoreLocation;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.forks.ForksMode;
import frc.robot.forks.ForksSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.JacksonLagUtil;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.AutoRotate;
import frc.robot.managers.Autobalance;
import frc.robot.managers.SuperstructureManager;
import frc.robot.managers.SuperstructureMotionManager;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.wrist.WristSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private final PowerDistribution pdpLogging;

  private final SwerveModule frontLeft =
      new SwerveModule(
          Config.SWERVE_FL_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_FL_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_FL_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANCoder(Config.SWERVE_FL_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule frontRight =
      new SwerveModule(
          Config.SWERVE_FR_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_FR_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_FR_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANCoder(Config.SWERVE_FR_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule backLeft =
      new SwerveModule(
          Config.SWERVE_BL_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_BL_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_BL_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANCoder(Config.SWERVE_BL_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule backRight =
      new SwerveModule(
          Config.SWERVE_BR_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_BR_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new com.ctre.phoenixpro.hardware.TalonFX(
              Config.SWERVE_BR_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANCoder(Config.SWERVE_BR_CANCODER_ID, Config.CANIVORE_ID));

  private final DriveController driveController = new DriveController(Config.DRIVE_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(Config.OPERATOR_CONTROLLER_PORT);

  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(new TalonFX(Config.ELEVATOR_MOTOR_ID, Config.CANIVORE_ID));
  private final WristSubsystem wrist =
      new WristSubsystem(new TalonFX(Config.WRIST_MOTOR_ID, Config.CANIVORE_ID));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(new TalonFX(Config.INTAKE_MOTOR_ID, Config.CANIVORE_ID));
  private final SuperstructureMotionManager superstructureMotionManager =
      new SuperstructureMotionManager(elevator, wrist, driveController);
  private final ImuSubsystem imu =
      new ImuSubsystem(new Pigeon2(Config.PIGEON_ID, Config.CANIVORE_ID));
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(imu, frontRight, frontLeft, backRight, backLeft);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu);
  private final ForksSubsystem forks =
      new ForksSubsystem(new TalonFX(Config.FORKS_MOTOR_ID, Config.CANIVORE_ID));
  private final FmsSubsystem fmsSubsystem = new FmsSubsystem();
  private final SuperstructureManager superstructureManager =
      new SuperstructureManager(superstructureMotionManager, intake, localization);
  private final LightsSubsystem lights =
      new LightsSubsystem(
          new CANdle(Config.LIGHTS_CANDLE_ID, Config.CANIVORE_ID),
          intake,
          superstructureManager,
          localization);

  private final Autobalance autobalance = new Autobalance(swerve, imu);
  private final AutoRotate autoRotate = new AutoRotate(swerve);

  private final Autos autos =
      new Autos(
          localization, swerve, imu, superstructureManager, elevator, wrist, intake, autobalance);

  private Command autoCommand = autos.getAutoCommand();

  public Robot() {
    // Log to a USB stick
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/"));
    // Publish data to NetworkTables
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    // Enables power distribution logging
    pdpLogging = new PowerDistribution(Config.PDP_ID, Config.PDP_TYPE);

    // Record metadata
    Logger.getInstance().recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.getInstance().recordMetadata("RoborioSerialNumber", Config.SERIAL_NUMBER);
    Logger.getInstance().recordMetadata("RobotConfig", Config.IS_SPIKE ? "Spike" : "Tyke");
    Logger.getInstance().recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.getInstance().recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.getInstance().recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.getInstance().recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.getInstance().recordMetadata("GitDirty", "Unknown");
        break;
    }

    Logger.getInstance().start();

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    configureButtonBindings();

    enableLiveWindowInTest(false);

    JacksonLagUtil.causeLag();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.getDriveTeleopCommand(driveController));

    // Intake on floor
    driveController
        .leftTrigger(0.3)
        .onTrue(superstructureManager.getFloorIntakeSpinningCommand())
        .onFalse(superstructureManager.getFloorIntakeIdleCommand());
    // Outtake/score low node/finish manual score
    driveController
        .rightTrigger(0.3)
        .onTrue(superstructureManager.getScoreCommand(NodeHeight.LOW, 0));
    // Zero gyro
    driveController.back().onTrue(localization.getZeroCommand());
    // Set mode to cubes
    driveController.povUp().onTrue(superstructureManager.setIntakeModeCommand(HeldGamePiece.CUBE));
    // Set mode to cones
    driveController
        .povDown()
        // TODO: Support cancelling this command when the button is released early
        .onTrue(superstructureManager.setIntakeModeCommand(HeldGamePiece.CONE));
    // Intake on shelf
    driveController.leftBumper().onTrue(superstructureManager.getShelfIntakeCommand());

    // X swerve
    driveController.x().onTrue(swerve.getXSwerveCommand());

    // Face towards grids
    driveController.b().onTrue(autoRotate.getCommand(() -> AutoRotate.getGridAngle()));
    // Face towards shelf
    driveController.a().onTrue(autoRotate.getCommand(() -> AutoRotate.getShelfAngle()));

    new Trigger(() -> driveController.getThetaPercentage() == 0)
        .onFalse(autoRotate.getDisableCommand());

    new Trigger(
            () ->
                driveController.getSidewaysPercentage() == 0
                    && driveController.getForwardPercentage() == 0
                    && driveController.getThetaPercentage() == 0)
        .onFalse(swerve.disableXSwerveCommand());

    // Manual intake
    operatorController
        .leftTrigger(0.3)
        .onTrue(superstructureManager.setManualIntakeCommand(IntakeMode.MANUAL_INTAKE))
        .onFalse(superstructureManager.setManualIntakeCommand(null));
    // Manual outtake
    operatorController
        .rightTrigger(0.3)
        .onTrue(superstructureManager.setManualIntakeCommand(IntakeMode.MANUAL_OUTTAKE))
        .onFalse(superstructureManager.setManualIntakeCommand(null));
    // Manual score low
    operatorController
        .a()
        .onTrue(superstructureManager.getManualScoreCommand(NodeHeight.LOW))
        .onFalse(superstructureManager.getCommand(States.STOWED));
    // Manual score mid
    operatorController
        .b()
        .onTrue(superstructureManager.getManualScoreCommand(NodeHeight.MID))
        .onFalse(superstructureManager.getCommand(States.STOWED));
    // Manual score high
    operatorController
        .y()
        .onTrue(superstructureManager.getManualScoreCommand(NodeHeight.HIGH))
        .onFalse(superstructureManager.getCommand(States.STOWED));

    // Stow all
    operatorController.x().onTrue(superstructureManager.getCommand(States.STOWED));
    // Home superstructure
    operatorController.back().onTrue(superstructureManager.getHomeCommand());

    // Forks go up
    operatorController
        .povUp()
        .onTrue(forks.getCommand(ForksMode.UP))
        .onFalse(forks.getCommand(ForksMode.STOPPED));
    // Forks go down
    operatorController
        .povDown()
        .onTrue(forks.getCommand(ForksMode.DOWN))
        .onFalse(forks.getCommand(ForksMode.STOPPED));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    AutoScoreLocation autoScoreLocation =
        superstructureManager.getAutoScoreLocation(driveController.getAutoScoreNodeKind());
    Logger.getInstance().recordOutput("AutoScore/GoalLocation/Pose", autoScoreLocation.pose);
    Logger.getInstance()
        .recordOutput("AutoScore/GoalLocation/Node", autoScoreLocation.node.toString());

    Logger.getInstance()
        .recordOutput("Scheduler/Stage", LifecycleSubsystemManager.getStage().toString());
  }

  @Override
  public void autonomousInit() {
    autoCommand = autos.getAutoCommand();
    CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autoCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
