// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

public enum SubsystemPriority {
  SUPERSTRUCTURE_MANAGER(21),

  SUPERSTRUCTURE_MOTION_MANAGER(20),

  // Run autobalance before swerve
  AUTOBALANCE(11),

  WRIST(10),
  ELEVATOR(10),
  INTAKE(10),
  SWERVE(10),
  IMU(10),

  // Run localization after swerve & IMU
  LOCALIZATION(9),

  // Update the lights after everything else has been updated
  LIGHTS(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
