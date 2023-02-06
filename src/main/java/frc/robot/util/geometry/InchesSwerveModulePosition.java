// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

public class InchesSwerveModulePosition extends SwerveModulePosition {
  public final double distanceInches;

  public InchesSwerveModulePosition(double distanceInches, Rotation2d angle) {
    super(Units.inchesToMeters(distanceInches), angle);
    this.distanceInches = distanceInches;
  }

  public InchesSwerveModulePosition(SwerveModulePosition position) {
    this(Units.metersToInches(position.distanceMeters), position.angle);
  }

  @Override
  public boolean equals(Object obj) {
    return super.equals(obj);
  }

  @Override
  public int hashCode() {
    return super.hashCode();
  }
}
