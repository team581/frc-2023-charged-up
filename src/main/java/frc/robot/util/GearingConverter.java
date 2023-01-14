// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class GearingConverter {
  public final double gearingReduction;

  /** Creates a new GearingUtil with the provided gear reduction. */
  public static GearingConverter fromReduction(double gearingReduction) {
    return new GearingConverter(gearingReduction);
  }

  /** Creates a new GearingUtil with the provided gear upduction. */
  public static GearingConverter fromUpduction(double gearingUpduction) {
    return new GearingConverter(1 / gearingUpduction);
  }

  /** Creates a new GearingUtil with the provided gear ratio. */
  private GearingConverter(double gearingReduction) {
    this.gearingReduction = gearingReduction;
  }

  /** Converts a value before gearing to the value after gearing. */
  public double beforeToAfterGearing(double value) {
    return value / gearingReduction;
  }

  /** Converts a value after gearing to the value after gearing. */
  public double afterToBeforeGearing(double value) {
    return value * gearingReduction;
  }

  public static CircleConverter fromDiameter(int i) {
    return null;
  }
}
