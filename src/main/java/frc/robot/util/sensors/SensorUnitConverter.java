// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sensors;

public class SensorUnitConverter {
  public static final CtreUnitConverter cancoder = new CtreUnitConverter(4096);
  public static final CtreUnitConverter talonFX = new CtreUnitConverter(2048);

  private SensorUnitConverter() {}
}
