// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * Extends {@link SubsystemBase} by adding in lifecycle methods for robotInit, teleopPeriodic, etc.,
 * similar to {@link Robot}.
 */
public class LifecycleSubsystem extends SubsystemBase {
  private enum Mode {
    TELEOP,
    AUTONOMOUS,
    DISABLED,
    ENABLED
  }

  private Mode mode = null;

  public LifecycleSubsystem() {
    robotInit();
  }

  @Override
  public void periodic() {
    super.periodic();
    robotPeriodic();

    if (DriverStation.isEnabled()) {
      enabledPeriodic();
      if (mode != Mode.ENABLED) {
        enabledInit();
        mode = Mode.ENABLED;
      }
    } else {
      disabledPeriodic();
      if (mode != Mode.DISABLED) {
        disabledInit();
        mode = Mode.DISABLED;
      }
    }

    if (DriverStation.isTeleopEnabled()) {
      teleopPeriodic();
      if (mode != Mode.TELEOP) {
        teleopInit();
        mode = Mode.TELEOP;
      }
    } else if (DriverStation.isAutonomousEnabled()) {
      autonomousPeriodic();
      if (mode != Mode.AUTONOMOUS) {
        autonomousInit();
        mode = Mode.AUTONOMOUS;
      }
    }
  }

  /** {@link IterativeRobotBase#robotInit()} */
  public void robotInit() {}

  /** {@link IterativeRobotBase#robotPeriodic()} */
  public void robotPeriodic() {}

  public void enabledInit() {}

  public void enabledPeriodic() {}

  /** {@link IterativeRobotBase#autonomousInit()} */
  public void autonomousInit() {}

  /** {@link IterativeRobotBase#autonomousPeriodic()} */
  public void autonomousPeriodic() {}

  /** {@link IterativeRobotBase#teleopInit()} */
  public void teleopInit() {}

  /** {@link IterativeRobotBase#teleopPeriodic()} */
  public void teleopPeriodic() {}

  /** {@link IterativeRobotBase#disabledInit()} */
  public void disabledInit() {}

  /** {@link IterativeRobotBase#disabledPeriodic()} */
  public void disabledPeriodic() {}
}
