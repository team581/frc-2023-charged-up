// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class SimulationUtil {
  public static boolean simulateDelay(double duration) {
    return Math.random() < 1.0 / (duration * 50.0);
  }

  private SimulationUtil() {}
}
