// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class InchesChassisSpeeds extends ChassisSpeeds {
  public final double vxInchesPerSecond;
  public final double vyInchesPerSecond;
  public final double omegaDegreesPerSecond;

  public InchesChassisSpeeds(
      double vxInchesPerSecond, double vyInchesPerSecond, double omegaDegreesPerSecond) {
    super(
        Units.inchesToMeters(vxInchesPerSecond),
        Units.inchesToMeters(vyInchesPerSecond),
        Units.radiansToDegrees(omegaDegreesPerSecond));
    this.vxInchesPerSecond = vxInchesPerSecond;
    this.vyInchesPerSecond = vyInchesPerSecond;
    this.omegaDegreesPerSecond = omegaDegreesPerSecond;
  }

  public InchesChassisSpeeds(ChassisSpeeds speeds) {
    this(
        Units.metersToInches(speeds.vxMetersPerSecond),
        Units.metersToInches(speeds.vyMetersPerSecond),
        Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
  }
}
