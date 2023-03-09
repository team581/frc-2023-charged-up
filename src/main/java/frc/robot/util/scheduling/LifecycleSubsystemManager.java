// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class LifecycleSubsystemManager {
  public static LifecycleStage getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return LifecycleStage.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return LifecycleStage.AUTONOMOUS;
    } else if (DriverStation.isDisabled()) {
      return LifecycleStage.DISABLED;
    } else {
      return LifecycleStage.TEST;
    }
  }

  private static LifecycleSubsystemManager instance;

  public static LifecycleSubsystemManager getInstance() {
    if (instance == null) {
      instance = new LifecycleSubsystemManager();
    }

    return instance;
  }

  private final List<LifecycleSubsystem> subsystems = new ArrayList<>();
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private LifecycleSubsystemManager() {}

  public void ready() {
    Collections.sort(
        subsystems,
        (Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed()));

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }

  void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }
}
