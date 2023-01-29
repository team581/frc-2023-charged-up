// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.managers.SuperstructureMotionManager;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.wrist.WristSubsystem;

import java.lang.ModuleLayer.Controller;

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
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_FL_DRIVE_MOTOR_ID, "581CANivore"),
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_FL_STEER_MOTOR_ID, "581CANivore"),
          new CANCoder(Config.SWERVE_FL_CANCODER_ID, "581CANivore"));
  private final SwerveModule frontRight =
      new SwerveModule(
          Config.SWERVE_FR_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_FR_DRIVE_MOTOR_ID, "581CANivore"),
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_FR_STEER_MOTOR_ID, "581CANivore"),
          new CANCoder(Config.SWERVE_FR_CANCODER_ID, "581CANivore"));
  private final SwerveModule backLeft =
      new SwerveModule(
          Config.SWERVE_BL_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_BL_DRIVE_MOTOR_ID, "581CANivore"),
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_BL_STEER_MOTOR_ID, "581CANivore"),
          new CANCoder(Config.SWERVE_BL_CANCODER_ID, "581CANivore"));
  private final SwerveModule backRight =
      new SwerveModule(
          Config.SWERVE_BR_CONSTANTS,
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_BR_DRIVE_MOTOR_ID, "581CANivore"),
          new com.ctre.phoenixpro.hardware.TalonFX(Config.SWERVE_BR_STEER_MOTOR_ID, "581CANivore"),
          new CANCoder(Config.SWERVE_BR_CANCODER_ID, "581CANivore"));

  // TODO: Re-enable elevator and wrist
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          new com.ctre.phoenix.motorcontrol.can.TalonFX(Config.ELEVATOR_MOTOR_ID, "581CANivore"));
  private final WristSubsystem wrist =
      new WristSubsystem(
          new com.ctre.phoenix.motorcontrol.can.TalonFX(Config.WRIST_MOTOR_ID, "581CANivore"));
  private final SuperstructureMotionManager superstructureMotionManager =
          new SuperstructureMotionManager(elevator, wrist);
  private final ImuSubsystem imu = new ImuSubsystem(new Pigeon2(Config.PIGEON_ID, "581CANivore"));
  private final SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(imu, frontRight, frontLeft, backRight, backLeft);

  private final DriveController driveController = new DriveController(Config.CONTROLLER_PORT);

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
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    boolean buttonA = driveController.getAButton();
    boolean buttonB = driveController.getBButton();
    boolean buttonY = driveController.getYButton();
    boolean buttonX = driveController.getXButton();
    double rightTrigger = driveController.getRightTriggerAxis();
    boolean rightBumper = driveController.getRightBumper();

    if (buttonX) {
      elevator.startHoming();
      wrist.startHoming();
    } else if (buttonA) {
      superstructureMotionManager.set(
          1, Rotation2d.fromDegrees(5)); // Original: height 1 and degrees 5
    } else if (buttonB) {
      superstructureMotionManager.set(
          16, Rotation2d.fromDegrees(20)); // Original: height 12 and degrees 65
    } else if (buttonY) {
      superstructureMotionManager.set(
          32, Rotation2d.fromDegrees(20)); // Original: height 24 and degrees 95
    } else if (rightTrigger > 0.3) {
      superstructureMotionManager.set(
          32, Rotation2d.fromDegrees(100)); // Original: height 1 and degrees 35
    } else if (rightBumper) {
      superstructureMotionManager.set(
          1, Rotation2d.fromDegrees(125)); // Original: height 1 and degrees 125
    }

    boolean openLoop = !driveController.getStartButton();
    swerveSubsystem.driveTeleop(
        -driveController.getSidewaysPercentage(),
        driveController.getForwardPercentage(),
        -driveController.getThetaPercentage(),
        true,
        openLoop);
        //If backButton is pressed then closed loop
    if (driveController.getBackButton()) {
      imu.zero();
    }
      Logger.getInstance().recordOutput("Menu buttons Imu", driveController.getBackButton());
      Logger.getInstance().recordOutput("Menu buttons/ Closed Loop", driveController.getStartButton());
  }

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
