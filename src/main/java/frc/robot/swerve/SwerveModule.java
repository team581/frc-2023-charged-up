// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.CircleConverter;
import frc.robot.util.GearingConverter;

public class SwerveModule {
  private static final double TICKS_PER_ROTATION = 12.8 * 2048;
  private static final double DRIVE_MOTOR_MAX_VOLTAGE = 12;
  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(10);
  private static final GearingConverter STEER_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(12.8);
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      GearingConverter.fromDiameter(6);
  private SwerveModuleConstants constants;
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANcoder encoder;
  private Rotation2d swerveWheelOffset;

  public SwerveModule(
      SwerveModuleConstants constants,
      TalonFX driveMotor,
      TalonFX steerMotor,
      CANcoder encoder,
      Rotation2d swerveWheeleOffset) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;
    this.swerveWheelOffset = swerveWheelOffset;

    driveMotor.config_kP(0, 0);
    driveMotor.config_kI(0, 0);
    driveMotor.config_kD(0, 0);
    driveMotor.config_kF(0, 0);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    driveMotor.setInverted(this.constants.driveInversion);

    steerMotor.config_kP(0, 0);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0);
    steerMotor.config_kF(0, 0);
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    steerMotor.setInverted(this.constants.driveInversion);

    // resetWheelAngle();
  }

  // public void setDesiredStae()

  // private Rotation2d getSteerMotorPosition() {
  //   double ticksBeforeGearing = steerMotor.getSelectedSensorPosition();
  //   double ticks = STEER_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(ticksBeforeGearing);
  //   double rotations = SensorUnitConverter.talonFX.sensorUnitsToRotations(ticks);
  //   return new Rotation2d(Units.roationstoRadians(rotations));
  // }

  // public void getDriveMotorVelocity() {
  //   final var ticksPer100msBeforeGearing = driveMotor.getSelectedSensorVelocity();
  //   final var ticksPer100ms =
  // DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
  //   final var rotationsPerMinute =
  // SensorUnitConverter.talonFX.sensorUnitsPer100msToRotationsPerMinute(ticksPer100ms);
  //   final var feetPerMinute = DRIVE_MOTOR_WHEEL_CONVERTER.roationsToDistance(roationsPerMinute);
  //   final var feetPerSecond = feetPerMinute / 60;
  //   return feetPerSecond;
  // }

  // public void resetWheelAngle() {
  //   final var absolutePosition = cancoderPosition();
  //   double rotations = absolutePosition.getDegrees() / 360;
  //   double encoderTicks = rotations * TICKS_PER_ROTATION;
  //   steerMotor.setSelectedSensorPosition(encoderTicks);
  // }

  // private final Rotation2d cancoderPosition() {
  //   return
  // Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue()).minus(swerveWheelOffset);
  // }
}
