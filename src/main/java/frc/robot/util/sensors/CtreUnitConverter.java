// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sensors;

public class CtreUnitConverter extends SensorUnitConverterBase {
  protected CtreUnitConverter(double sensorUnitsPerRotation) {
    super(sensorUnitsPerRotation);
  }

  public int rotationsPerMinuteToSensorUnitsPer100ms(double rotationsPerMinute) {
    final var sensorUnitsPerMinute = rotationsToSensorUnits(rotationsPerMinute);
    final var sensorUnitsPerSecond = sensorUnitsPerMinute / 60;
    final var sensorUnitsPer100ms = sensorUnitsPerSecond / 10;

    return (int) Math.round(sensorUnitsPer100ms);
  }

  public double sensorUnitsPer100msToRotationsPerMinute(double sensorUnitsPer100ms) {
    final var sensorUnitsPerSecond = sensorUnitsPer100ms * 10;
    final var sensorUnitsPerMinute = sensorUnitsPerSecond * 60;

    return sensorUnitsToRotations(sensorUnitsPerMinute);
  }
}
