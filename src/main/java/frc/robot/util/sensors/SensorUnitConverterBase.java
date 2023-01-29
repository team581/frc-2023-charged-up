// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sensors;

public class SensorUnitConverterBase {
  private final double sensorUnitsPerRotation;

  protected SensorUnitConverterBase(double sensorUnitsPerRotation) {
    this.sensorUnitsPerRotation = sensorUnitsPerRotation;
  }

  public double sensorUnitsToRotations(double sensorUnits) {
    return sensorUnits / sensorUnitsPerRotation;
  }

  public double rotationsToSensorUnits(double rotations) {
    return rotations * sensorUnitsPerRotation;
  }
}
