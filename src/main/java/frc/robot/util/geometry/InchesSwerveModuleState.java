// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class InchesSwerveModuleState extends SwerveModuleState {
  public final double speedInchesPerSecond;

  public InchesSwerveModuleState(double speedInchesPerSecond, Rotation2d angle) {
    super(Units.inchesToMeters(speedInchesPerSecond), angle);
    this.speedInchesPerSecond = speedInchesPerSecond;
  }

  public InchesSwerveModuleState(SwerveModuleState state) {
    this(Units.metersToInches(state.speedMetersPerSecond), state.angle);
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
