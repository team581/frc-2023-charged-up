// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CircleConverter {
  public static CircleConverter fromRadius(double radius) {
    return new CircleConverter(radius);
  }

  public static CircleConverter fromDiameter(double diameter) {
    return new CircleConverter(diameter / 2);
  }

  private final double radius;

  private CircleConverter(double radius) {
    this.radius = radius;
  }

  /** Converts a rotation to a distance. */
  public double rotationsToDistance(double rotationsPerMinute) {
    return Units.rotationsToRadians(rotationsPerMinute) * radius;
  }

  /** Converts a rotation to a distance. */
  public double rotationsToDistance(Rotation2d rotation) {
    return rotationsToDistance(Units.radiansToRotations(rotation.getRadians()));
  }

  /** Converts a distance to a rotation. */
  public double distanceToRotations(double distance) {
    return Units.radiansToRotations(distance / radius);
  }
}
