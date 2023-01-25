// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.Config;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.generated.BuildConstants;
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
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(new TalonFX(Config.ELEVATOR_MOTOR_ID, "581CANivore"));
  private final WristSubsystem wrist =
      new WristSubsystem(new TalonFX(Config.WRIST_MOTOR_ID, "581CANivore"));
  private final XboxController controller = new XboxController(Config.CONTROLLER_PORT);

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
    boolean buttonA = controller.getAButton();
    boolean buttonB = controller.getBButton();
    boolean buttonY = controller.getYButton();
    boolean buttonX = controller.getXButton();
    double rightTrigger = controller.getRightTriggerAxis();
    boolean rightBumper = controller.getRightBumper();

    if (buttonX) {
      elevator.startHoming();
      wrist.startHoming();
    } else if (buttonA) {
      elevator.setGoalPosition(2);
    } else if (buttonB) {
      elevator.setGoalPosition(12);
      wrist.setAngle(Rotation2d.fromDegrees(45));
    } else if (buttonY) {
      elevator.setGoalPosition(24);
      wrist.setAngle(Rotation2d.fromDegrees(90));
    } else if (rightTrigger > 0.3) {
      wrist.setAngle(Rotation2d.fromDegrees(30));
    } else if (rightBumper) {
      wrist.setAngle(Rotation2d.fromDegrees(135));
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
