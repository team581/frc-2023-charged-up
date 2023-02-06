// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class InchesPose2d extends Pose2d {
  private final InchesTranslation2d inchesTranslation2d;

  public InchesPose2d(InchesTranslation2d translation, Rotation2d rotation) {
    super(translation, rotation);
    inchesTranslation2d = translation;
  }

  public InchesPose2d(double xInches, double yInches, Rotation2d rotation) {
    this(new InchesTranslation2d(xInches, yInches), rotation);
  }

  public InchesPose2d(Pose2d pose) {
    this(new InchesTranslation2d(pose.getTranslation()), pose.getRotation());
  }

  public InchesPose2d(Translation2d translation, Rotation2d rotation) {
    this(new InchesTranslation2d(translation), rotation);
  }

  public InchesTranslation2d getTranslationInches() {
    return inchesTranslation2d;
  }

  public double getXInches() {
    return inchesTranslation2d.getXInches();
  }

  public double getYInches() {
    return inchesTranslation2d.getYInches();
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
