// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.Config;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.commands.IntakeCommand;
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
  private final IntakeSubsystem intake =
      new IntakeSubsystem(new TalonFX(Config.INTAKE_MOTOR_ID, "581CANivore"));
  private final CommandXboxController controller =
      new CommandXboxController(Config.CONTROLLER_PORT);

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
    boolean buttonA = controller.a().getAsBoolean();
    boolean buttonB = controller.b().getAsBoolean();
    boolean buttonY = controller.y().getAsBoolean();
    boolean buttonX = controller.x().getAsBoolean();

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
    }

    controller.rightTrigger().whileTrue(new IntakeCommand(intake, IntakeMode.OUTTAKE));
    controller.leftTrigger().whileTrue(new IntakeCommand(intake, IntakeMode.INTAKE_CUBE));
    controller.leftBumper().whileTrue(new IntakeCommand(intake, IntakeMode.INTAKE_CONE));
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
