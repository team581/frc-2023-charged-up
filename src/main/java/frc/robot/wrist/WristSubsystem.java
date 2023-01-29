// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Config;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final double HOMED_CURRENT = 15;
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
  private final TalonFX motor;
  private Rotation2d goalAngle = new Rotation2d();
  private boolean homing = false;
  private boolean goToGoal = false;

  public WristSubsystem(TalonFX motor) {
    this.motor = motor;

    motor.config_kF(0, 0);
    motor.config_kP(0, 0.1); // Edit PID and add motion magic
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);

    this.motor.configMotionCruiseVelocity(20000);
    this.motor.configMotionAcceleration(50000);

    this.motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 25, 25, 0.5));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
        motor.getSelectedSensorPosition() / 2048.0 / Config.WRIST_GEARING);
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
    goToGoal = false;
  }

  @Override
  public void enabledPeriodic() {
    if (homing) {
      motor.set(TalonFXControlMode.PercentOutput, 0.15);

      if (motor.getStatorCurrent() > HOMED_CURRENT) {
        motor.set(TalonFXControlMode.PercentOutput, 0);
        motor.setSelectedSensorPosition(
            Rotation2d.fromDegrees(133.0).getRotations() * 2048.0 * Config.WRIST_GEARING);
        setAngle(Rotation2d.fromDegrees(80));
        homing = false;
        goToGoal = true;
      }
    } else if (goToGoal) {
      motor.set(ControlMode.MotionMagic, goalAngle.getRotations() * 2048 * Config.WRIST_GEARING);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/Homing", homing);
    Logger.getInstance().recordOutput("Wrist/Current", motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Wrist/GoToGoal", goToGoal);
    Logger.getInstance().recordOutput("Wrist/Voltage", motor.getMotorOutputVoltage());
  }
}
