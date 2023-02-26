// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ManualScoringLocation;
import frc.robot.autoscore.NodeKind;

public class DriveController extends CommandXboxController {
  public DriveController(int port) {
    super(port);
  }

  /** Scale a joystick value. */
  private static double joystickScale(double x) {
    if (Math.abs(x) < 0.075) {
      return 0;
    }

    return Math.signum(x) * Math.pow(x, 2);
  }

  /** The rotation across the robot's x-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getSidewaysPercentage() {
    return joystickScale(getLeftX());
  }

  /** The translation across the robot's y-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getForwardPercentage() {
    return joystickScale(-1 * getLeftY());
  }

  /** The rotation about the robot's z-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getThetaPercentage() {
    return joystickScale(-1 * getRightX());
  }

  public NodeKind getAutoScoreNodeKind() {
    double rightX = getRightX();
    double rightY = -getRightY();

    ManualScoringLocation scoringHeight;

    if (rightY > 0.6) {
      scoringHeight = ManualScoringLocation.HIGH;
    } else if (rightY < -0.6) {
      scoringHeight = ManualScoringLocation.LOW;
    } else {
      scoringHeight = ManualScoringLocation.MID;
    }

    if (rightX < -0.6) {
      if (scoringHeight == ManualScoringLocation.LOW) {
        return NodeKind.LEFT_HYBRID;
      } else if (scoringHeight == ManualScoringLocation.MID) {
        return NodeKind.LEFT_MID_CONE;
      } else if (scoringHeight == ManualScoringLocation.HIGH) {
        return NodeKind.LEFT_HIGH_CONE;
      }
    } else if (rightX > 0.6) {
      if (scoringHeight == ManualScoringLocation.LOW) {
        return NodeKind.RIGHT_HYBRID;
      } else if (scoringHeight == ManualScoringLocation.MID) {
        return NodeKind.RIGHT_MID_CONE;
      } else if (scoringHeight == ManualScoringLocation.HIGH) {
        return NodeKind.RIGHT_HIGH_CONE;
      }
    }

    if (scoringHeight == ManualScoringLocation.LOW) {
      return NodeKind.CENTER_HYBRID;
    } else if (scoringHeight == ManualScoringLocation.HIGH) {
      return NodeKind.CENTER_HIGH_CUBE;
    }

    return NodeKind.CENTER_MID_CUBE;
  }
}
