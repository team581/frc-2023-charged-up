// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
  private final TalonFX motor;
  private Rotation2d goalAngle = Positions.STOWED.angle;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private boolean isHoming = false;
  private boolean goToGoal = false;

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;

    motor.setInverted(false);

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
    double rawCurrent = motor.getSupplyCurrent();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    Logger.getInstance().recordOutput("Wrist/FilteredCurrent", filteredCurrent);
    Logger.getInstance().recordOutput("Wrist/RawCurrent", rawCurrent);

    if (isHoming) {
      motor.set(TalonFXControlMode.PercentOutput, Config.WRIST_HOMING_VOLTAGE);

      if (filteredCurrent > Config.WRIST_HOMED_CURRENT) {
        motor.set(TalonFXControlMode.PercentOutput, 0);
        motor.setSelectedSensorPosition(
            Config.WRIST_HOMED_ANGLE.getRotations() * 2048.0 * Config.WRIST_GEARING);
        setAngle(Positions.STOWED.angle);
        isHoming = false;
        goToGoal = true;
      }
    } else if (goToGoal) {
      motor.set(ControlMode.MotionMagic, goalAngle.getRotations() * 2048 * Config.WRIST_GEARING);
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/Homing", isHoming);
    Logger.getInstance().recordOutput("Wrist/GoToGoal", goToGoal);
    Logger.getInstance().recordOutput("Wrist/Voltage", motor.getMotorOutputVoltage());

    if (Config.IS_DEVELOPMENT) {
      Logger.getInstance().recordOutput("Wrist/RawAngle", motor.getSelectedSensorPosition());
      Logger.getInstance().recordOutput("Wrist/ControlMode", motor.getControlMode().toString());
    }
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming()).andThen(Commands.waitUntil(() -> isHoming() == false));
  }
}
