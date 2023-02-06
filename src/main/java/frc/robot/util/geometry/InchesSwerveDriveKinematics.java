// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class InchesSwerveDriveKinematics extends SwerveDriveKinematics {
  public static void desaturateWheelSpeeds(
      SwerveModuleState[] moduleStates, double attainableMaxSpeedInchesPerSecond) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, Units.inchesToMeters(attainableMaxSpeedInchesPerSecond));
  }

  public InchesSwerveDriveKinematics(Translation2d... moduleLocations) {
    super(moduleLocations);
  }

  @Override
  public InchesChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
    return new InchesChassisSpeeds(super.toChassisSpeeds(moduleStates));
  }

  public InchesSwerveModuleState[] toSwerveModuleStates(InchesChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] rawStates = super.toSwerveModuleStates(chassisSpeeds);
    InchesSwerveModuleState[] states = new InchesSwerveModuleState[rawStates.length];

    for (int i = 0; i < rawStates.length; i++) {
      states[i] = new InchesSwerveModuleState(rawStates[i]);
    }

    return states;
  }
}
