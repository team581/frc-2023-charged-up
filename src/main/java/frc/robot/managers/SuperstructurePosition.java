package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructurePosition {
  public final double height;
  public final Rotation2d angle;

  public SuperstructurePosition(double height, Rotation2d angle) {
    this.height = height;
    this.angle = angle;
  }
}
