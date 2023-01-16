// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.Config;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSubsystem;
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
  // private final ElevatorSubsystem elevator =
  //     new ElevatorSubsystem(new TalonFX(Config.ELEVATOR_MOTOR_ID, "581CANivore"));
  // private final WristSubsystem wrist =
  //     new WristSubsystem(new TalonFX(Config.WRIST_MOTOR_ID, "581CANivore"));
  private final ImuSubsystem imu = new ImuSubsystem(new Pigeon2(Config.PIGEON_ID, "581CANivore"));
  private final SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(imu, frontRight, frontLeft, backRight, backLeft);
  private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem();
  private final XboxController controller = new XboxController(Config.CONTROLLER_PORT);

  public Robot() {

    // Log to a USB stick
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/"));
    // Publish data to NetworkTables
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    // Enables power distribution logging
    pdpLogging = new PowerDistribution(Config.PDP_ID, Config.PDP_TYPE);

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
    boolean buttonA = controller.getAButton();
    boolean buttonB = controller.getBButton();
    boolean buttonY = controller.getYButton();
    boolean buttonX = controller.getXButton();
    double rightTrigger = controller.getRightTriggerAxis();
    boolean rightBumper = controller.getRightBumper();

    // TODO: Re-enable elevator and wrist
    // if (buttonX) {
    //   elevator.startHoming();
    //   wrist.startHoming();
    // } else if (buttonA) {
    //   elevator.setGoalPosition(2);
    // } else if (buttonB) {
    //   elevator.setGoalPosition(12);
    //   wrist.setAngle(Rotation2d.fromDegrees(45));
    // } else if (buttonY) {
    //   elevator.setGoalPosition(24);
    //   wrist.setAngle(Rotation2d.fromDegrees(90));
    // } else if (rightTrigger > 0.3) {
    //   wrist.setAngle(Rotation2d.fromDegrees(30));
    // } else if (rightBumper) {
    //   wrist.setAngle(Rotation2d.fromDegrees(135));
    // }

    swerveSubsystem.driveTeleop(
        -controller.getLeftX(), controller.getLeftY(), -controller.getRightX(), true);
    if (controller.getStartButton()) {
      imu.zero();
    }
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
