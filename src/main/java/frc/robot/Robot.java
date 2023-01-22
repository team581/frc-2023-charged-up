// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleConstants;
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
  Pigeon2 pigeonImu = new Pigeon2(1, "581CANivore");

  ImuSubsystem imu = new ImuSubsystem(pigeonImu);
  SwerveModule frontLeft =
      new SwerveModule(
          new SwerveModuleConstants(
              Rotation2d.fromDegrees(-62.53), SwerveCorner.FRONT_LEFT, false, false),
          new TalonFX(8, "581CANivore"),
          new TalonFX(9, "581CANivore"),
          new CANCoder(13, "581CANivore"));
  SwerveModule frontRight =
      new SwerveModule(
          new SwerveModuleConstants(
              Rotation2d.fromDegrees(-148), SwerveCorner.FRONT_RIGHT, false, false),
          new TalonFX(6, "581CANivore"),
          new TalonFX(7, "581CANivore"),
          new CANCoder(12, "581CANivore"));
  SwerveModule backLeft =
      new SwerveModule(
          new SwerveModuleConstants(
              Rotation2d.fromDegrees(78.95), SwerveCorner.BACK_LEFT, false, false),
          new TalonFX(4, "581CANivore"),
          new TalonFX(5, "581CANivore"),
          new CANCoder(11, "581CANivore"));
  SwerveModule backRight =
      new SwerveModule(
          new SwerveModuleConstants(
              Rotation2d.fromDegrees(104.6), SwerveCorner.BACK_RIGHT, false, false),
          new TalonFX(2, "581CANivore"),
          new TalonFX(3, "581CANivore"),
          new CANCoder(10, "581CANivore"));
  SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(imu, frontRight, frontLeft, backRight, backLeft);
  XboxController controller = new XboxController(0);

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
