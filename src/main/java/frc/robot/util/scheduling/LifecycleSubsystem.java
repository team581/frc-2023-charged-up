// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Stopwatch;

/**
 * Extends {@link SubsystemBase} by adding in lifecycle methods for robotInit, teleopPeriodic, etc.,
 * similar to {@link Robot}.
 */
public class LifecycleSubsystem extends SubsystemBase {
  private final String loggerName;
  private final Stopwatch stopwatch = Stopwatch.getInstance();

  private LifecycleStage previousStage = null;

  public LifecycleSubsystem() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    loggerName = "Scheduler/LifecycleSubsystem/" + name + ".periodic()";

    robotInit();
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

  @Override
  public void periodic() {
    stopwatch.start(loggerName);
    LifecycleStage stage;

    if (DriverStation.isTeleopEnabled()) {
      stage = LifecycleStage.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      stage = LifecycleStage.AUTONOMOUS;
    } else {
      stage = LifecycleStage.DISABLED;
    }

    boolean isInit = previousStage != stage;

    robotPeriodic();

    if (stage == LifecycleStage.DISABLED) {
      if (isInit) {
        disabledInit();
      }

      disabledPeriodic();
    } else {
      if (isInit) {
        enabledInit();
      }

      enabledPeriodic();
    }

    if (stage == LifecycleStage.TELEOP) {
      if (isInit) {
        teleopInit();
      }

      teleopPeriodic();
    }

    if (stage == LifecycleStage.AUTONOMOUS) {
      if (isInit) {
        autonomousInit();
      }

      autonomousPeriodic();
    }

    stopwatch.stop(loggerName);

    previousStage = stage;
  }
}
