// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class InchesTranslation2d extends Translation2d {
  public InchesTranslation2d(double xInches, double yInches) {
    super(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
  }

  public InchesTranslation2d(Translation2d translation) {
    super(translation.getX(), translation.getY());
  }

  public double getXInches() {
    return Units.metersToInches(getX());
  }

  public double getYInches() {
    return Units.metersToInches(getY());
  }

  public double getDistanceInches(Translation2d other) {
    return Units.metersToInches(getDistance(other));
  }
}
