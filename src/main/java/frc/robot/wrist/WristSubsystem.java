// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LifecycleSubsystem;

import javax.swing.text.Position;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final double GEARING = 48 * 2;
  private static final double HOMED_CURRENT = 5;
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
  private final TalonFX motor;
  private Rotation2d goalAngle = new Rotation2d();
  private boolean homing = false;

  public WristSubsystem(TalonFX motor) {
    this.motor = motor;

    motor.config_kF(0, 0);
    motor.config_kP(0,0.05); // Edit PID and add motion magic
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);

  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(motor.getSelectedSensorPosition() / 2048.0 / GEARING);
  }

  public void setAngle(Rotation2d angle) {
    goalAngle = angle;
  }

  public boolean atAngle(Rotation2d angle) {
    var currentAngle = getAngle();
    return Math.abs(currentAngle.minus(angle).getDegrees()) < TOLERANCE.getDegrees();
  }

  public void startHoming() {
    homing = true;
  }

  @Override
  public void enabledPeriodic() {
    if (homing) {
      motor.set(TalonFXControlMode.PercentOutput, -0.25);

      if (motor.getStatorCurrent() > HOMED_CURRENT) {
        motor.set(TalonFXControlMode.PercentOutput, 0);
        motor.setSelectedSensorPosition(0);
        homing = false;
      }
    } else {
      motor.set(ControlMode.Position, goalAngle.getRotations() * 2048 * GEARING);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
  }
}
