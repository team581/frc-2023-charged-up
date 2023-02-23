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
import frc.robot.util.scheduling.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
  private final TalonFX motor;
  private Rotation2d goalAngle = new Rotation2d();
  private boolean isHoming = false;
  private boolean goToGoal = false;

  public WristSubsystem(TalonFX motor) {
    this.motor = motor;

    motor.config_kF(0, Config.WRIST_KF);
    motor.config_kP(0, Config.WRIST_KP);
    motor.config_kI(0, Config.WRIST_KI);
    motor.config_kD(0, Config.WRIST_KD);

    this.motor.configMotionCruiseVelocity(Config.WRIST_MOTION_CRUISE_VELOCITY);
    this.motor.configMotionAcceleration(Config.WRIST_MOTION_ACCELERATION);

    this.motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
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
    isHoming = true;
    goToGoal = false;
  }

  public boolean isHoming() {
    return isHoming;
  }

  @Override
  public void enabledPeriodic() {
    if (isHoming) {
      motor.set(TalonFXControlMode.PercentOutput, Config.WRIST_HOMING_VOLTAGE);

      if (motor.getStatorCurrent() > Config.WRIST_HOMED_CURRENT) {
        motor.set(TalonFXControlMode.PercentOutput, 0);
        motor.setSelectedSensorPosition(
            Config.WRIST_HOMED_ANGLE.getRotations() * 2048.0 * Config.WRIST_GEARING);
        setAngle(Rotation2d.fromDegrees(80));
        isHoming = false;
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
    Logger.getInstance().recordOutput("Wrist/Homing", isHoming);
    Logger.getInstance().recordOutput("Wrist/Current", motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Wrist/GoToGoal", goToGoal);
    Logger.getInstance().recordOutput("Wrist/Voltage", motor.getMotorOutputVoltage());
  }
}
