// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.fms;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class FmsSubsystem extends LifecycleSubsystem {
  public FmsSubsystem() {
    super(SubsystemPriority.FMS);
  }

  public static boolean isRedAlliance() {
    if (DriverStation.getAlliance() == Alliance.Red
        || DriverStation.getAlliance() == Alliance.Invalid) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void robotPeriodic() {}
}
