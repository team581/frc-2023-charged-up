package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final Rotation2d angleOffset;
  public final SwerveCorner corner;
  public final boolean angleInversion;
  public final boolean driveInversion;

  public SwerveModuleConstants(Rotation2d angleOffset, SwerveCorner corner, boolean angleInversion, boolean driveInversion) {
    this.angleOffset = angleOffset;
    this.corner = corner;
    this.angleInversion = angleInversion;
    this.driveInversion = driveInversion;
  }
}
