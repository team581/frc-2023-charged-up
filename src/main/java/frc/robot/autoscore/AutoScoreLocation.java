// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoscore;

import edu.wpi.first.math.geometry.Pose2d;

public class AutoScoreLocation {
  public final GridKind grid;
  public final NodeKind node;
  public final Pose2d pose;

  public AutoScoreLocation(GridKind grid, NodeKind node, Pose2d pose) {
    this.grid = grid;
    this.node = node;
    this.pose = pose;
  }
}
