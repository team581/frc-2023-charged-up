// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoscore;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.ManualScoringLocation;

public class AutoScoreLocation {
  public final GridKind grid;
  public final NodeKind node;
  public final ManualScoringLocation nodeHeight;
  public final Pose2d pose;

  public AutoScoreLocation(GridKind grid, NodeKind node, Pose2d pose) {
    this.grid = grid;
    this.node = node;
    this.pose = pose;
    if (node == NodeKind.CENTER_HIGH_CUBE
        || node == NodeKind.LEFT_HIGH_CONE
        || node == NodeKind.RIGHT_HIGH_CONE) {
      nodeHeight = ManualScoringLocation.HIGH;
    } else if (node == NodeKind.CENTER_MID_CUBE
        || node == NodeKind.LEFT_MID_CONE
        || node == NodeKind.RIGHT_MID_CONE) {
      nodeHeight = ManualScoringLocation.MID;
    } else {
      nodeHeight = ManualScoringLocation.LOW;
    }
  }
}
