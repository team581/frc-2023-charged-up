// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveController extends ButtonController {
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(7);

  public DriveController(int port) {
    super(port);
  }

  /** Scale a joystick value. */
  private static double joystickScale(double x) {
    return Math.signum(x) * Math.pow(x, 2);
  }

  /** The rotation across the robot's x-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getSidewaysPercentage() {
    return joystickScale(xLimiter.calculate(getLeftX()));
  }

  /** The translation across the robot's y-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getForwardPercentage() {
    return joystickScale(yLimiter.calculate(getLeftY()));
  }

  /** The rotation about the robot's z-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getThetaPercentage() {
    return joystickScale(thetaLimiter.calculate(getRightX()));
  }
}
