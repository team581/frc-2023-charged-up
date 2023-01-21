// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.CircleConverter;
import frc.robot.util.CtreModuleState;
import frc.robot.util.GearingConverter;
import frc.robot.util.sensors.SensorUnitConverter;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final SimpleMotorFeedforward DRIV_SIMPLE_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0, 0);
  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(Config.SWERVE_DRIVE_GEARING_REDUCTION);
  private static final GearingConverter STEER_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(Config.SWERVE_STEER_GEARING_REDUCTION);
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      CircleConverter.fromDiameter(6);
  private static final double STEER_MOTOR_TICKS_PER_ROTATION =
      STEER_MOTOR_GEARING_CONVERTER.gearingReduction * 2048;

  private final SwerveModuleConstants constants;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder encoder;
  private Rotation2d previousAngle = new Rotation2d();

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    driveMotor.config_kP(0, 0);
    driveMotor.config_kI(0, 0);
    driveMotor.config_kD(0, 0);
    driveMotor.config_kF(0, 0);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    driveMotor.setInverted(this.constants.driveInversion);

    steerMotor.config_kP(0, 0.67);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0.17);
    steerMotor.config_kF(0, 0);
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    steerMotor.setInverted(this.constants.driveInversion);

    resetWheelAngle();
  }

  public void setDesiredState(SwerveModuleState state, boolean OpenLoop) {
    final var steerMotorPosition = getSteerMotorPosition();

    state = CtreModuleState.optimize(state, steerMotorPosition);

    boolean isStopped = false;
    Rotation2d angle = isStopped ? this.previousAngle : state.angle;
    this.previousAngle = angle;
    final var rotationsBeforeGearing =
        STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(
            Units.radiansToRotations(angle.getRadians()));
    final var sensorUnitsBeforeGearing =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(rotationsBeforeGearing);
    steerMotor.set(ControlMode.Position, sensorUnitsBeforeGearing);

    final var feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    final var feetPerMinute = feetPerSecond * 60;
    final var rotationsPerMinute = DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRotations(feetPerMinute);
    final var sensorUnitsPer100ms =
        SensorUnitConverter.talonFX.rotationsPerMinuteToSensorUnitsPer100ms(rotationsPerMinute);
    final var sensorUnitsPer100msBeforeGearing =
        SensorUnitConverter.talonFX.rotationsPerMinuteToSensorUnitsPer100ms(sensorUnitsPer100ms);

    if (OpenLoop) {
      driveMotor.set(
          ControlMode.PercentOutput, state.speedMetersPerSecond / SwerveSubsystem.MAX_VELOCITY);
    } else {
      driveMotor.set(
          ControlMode.Velocity,
          sensorUnitsPer100msBeforeGearing,
          DemandType.ArbitraryFeedForward,
          DRIV_SIMPLE_MOTOR_FEEDFORWARD.calculate(state.speedMetersPerSecond));
    }

    Logger.getInstance()
        .recordOutput(
            this.constants.corner.toString() + "/Open loop voltage", state.speedMetersPerSecond);
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = getDriveMotorVelocity();

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  public void logValues() {
    Logger.getInstance()
        .recordOutput(
            this.constants.corner.toString() + "/Drive motor velocity (ft/sec)",
            this.getDriveMotorVelocity());
    Logger.getInstance()
        .recordOutput(
            this.constants.corner.toString() + "/Steer motor position (deg)",
            this.getSteerMotorPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            this.constants.corner.toString() + "/CANcoder position (deg)",
            this.getCancoderPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            this.constants.corner.toString() + "/Raw CANcoder position (deg)",
            this.getRawCancoderPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            this.toString() + "/Steer motor commanded angle (deg)",
            this.previousAngle.getDegrees());
  }

  private Rotation2d getSteerMotorPosition() {
    double ticksBeforeGearing = steerMotor.getSelectedSensorPosition();
    double ticks = STEER_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(ticksBeforeGearing);
    double rotations = SensorUnitConverter.talonFX.sensorUnitsToRotations(ticks);
    return new Rotation2d(Units.rotationsToRadians(rotations));
  }

  private double getDriveMotorVelocity() {
    final var ticksPer100msBeforeGearing = driveMotor.getSelectedSensorVelocity();
    final var ticksPer100ms =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(ticksPer100msBeforeGearing);
    final var rotationsPerMinute =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRotationsPerMinute(ticksPer100ms);
    final var feetPerMinute = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerMinute);
    final var feetPerSecond = feetPerMinute / 60;
    return feetPerSecond;
  }

  private void resetWheelAngle() {
    final var absolutePosition = getCancoderPosition();
    double rotations = absolutePosition.getDegrees() / 360;
    double encoderTicks = rotations * STEER_MOTOR_TICKS_PER_ROTATION;
    steerMotor.setSelectedSensorPosition(encoderTicks);
  }

  private final Rotation2d getCancoderPosition() {
    return getRawCancoderPosition().minus(constants.angleOffset);
  }

  private Rotation2d getRawCancoderPosition() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
  }
}
