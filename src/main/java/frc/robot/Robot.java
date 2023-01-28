// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.managers.SuperstructureMotionManager;
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
  XboxController controller = new XboxController(0);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(new TalonFX(14, "581CANivore"));
  private final WristSubsystem wrist = new WristSubsystem(new TalonFX(16, "581CANivore"));
  private final SuperstructureMotionManager superstructureMotionManager =
      new SuperstructureMotionManager(elevator, wrist);

  public Robot() {
    // Log to a USB stick
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/"));
    // Publish data to NetworkTables
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    // Enables power distribution logging
    new PowerDistribution(1, ModuleType.kCTRE);

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
