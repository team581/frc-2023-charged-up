// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.CircleConverter;
import frc.robot.util.CtreModuleState;
import frc.robot.util.GearingConverter;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final double MAX_SPEED = 13.5;
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
  private final VoltageOut driveMotorControl = new VoltageOut(0, true, true);
  private final DutyCycleOut driveRequest = new DutyCycleOut(0, true, true);
  private Rotation2d previousAngle = new Rotation2d();

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    Slot0Configs driveMotorSlot0Configs = new Slot0Configs();
    MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs driveMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveMotorSlot0Configs.kV = 0;
    driveMotorSlot0Configs.kP = 0;
    driveMotorSlot0Configs.kI = 0;
    driveMotorSlot0Configs.kD = 0;
    driveMotorCurrentLimitsConfigs.SupplyCurrentLimit = 15;
    driveMotorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    if (constants.driveInversion) {
      driveMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    driveMotor.getConfigurator().apply(driveMotorOutputConfigs);

    Slot0Configs steerMotorSlot0Configs = new Slot0Configs();
    MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs steerMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerMotorSlot0Configs.kV = 0;
    steerMotorSlot0Configs.kP = 0.67;
    steerMotorSlot0Configs.kI = 0;
    steerMotorSlot0Configs.kD = 0.17;
    steerMotorCurrentLimitsConfigs.SupplyCurrentLimit = 15;
    steerMotorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    if (constants.angleInversion) {
      steerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      steerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    steerMotor.getConfigurator().apply(steerMotorOutputConfigs);

    resetWheelAngle();
  }

  public void setDesiredState(SwerveModuleState state, boolean OpenLoop) {
    final var steerMotorPosition = getSteerMotorPosition();
    state = CtreModuleState.optimize(state, steerMotorPosition);

    final var steerMotorRequest =
        new PositionVoltage(
            STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(state.angle.getRotations()));
    steerMotorRequest.EnableFOC = true;
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Steer motor commanded angle",
            steerMotorRequest.Position);
    steerMotor.setControl(steerMotorRequest);

    boolean isStopped = false;
    Rotation2d angle = isStopped ? this.previousAngle : state.angle;
    this.previousAngle = angle;

    driveMotorControl.Output = 0;
    driveMotor.setControl(driveMotorControl);

    driveRequest.Output = Units.metersToFeet(state.speedMetersPerSecond) / MAX_SPEED;
    // if (OpenLoop) {
    driveMotor.setControl(driveRequest);
    // } else {
    //   driveMotor.VelocityDutyCycle(
    //       ControlMode.Velocity,
    //       true,
    //       DemandType.ArbitraryFeedForward,
    //       DRIV_SIMPLE_MOTOR_FEEDFORWARD.calculate(state.speedMetersPerSecond));
    // }

  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = getDriveMotorVelocity();

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  public void logValues() {
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Drive motor velocity (ft/sec)",
            this.getDriveMotorVelocity());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Steer motor position (deg)",
            this.getSteerMotorPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/CANcoder position (deg)",
            this.getCancoderPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Raw CANcoder position (deg)",
            this.getRawCancoderPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Steer motor commanded angle (deg)",
            this.previousAngle.getDegrees());
    Logger.getInstance().recordOutput(this.constants.corner.toString() + "/Steer Motor Position", getSteerMotorPosition().getDegrees());
    Logger.getInstance().recordOutput(null, MAX_SPEED);
    Logger.getInstance().recordOutput(null, MAX_SPEED);
    Logger.getInstance().recordOutput(null, MAX_SPEED);


  }

  private Rotation2d getSteerMotorPosition() {
    double rotationsBeforeGearing = steerMotor.getPosition().getValue();
    double rotations = STEER_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(rotationsBeforeGearing);
    return Rotation2d.fromRotations(rotations);
  }

  private double getDriveMotorVelocity() {
    final var rotationsPerSecondBeforeGearing = driveMotor.getVelocity().getValue();
    final var rotationsPerSecond =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(rotationsPerSecondBeforeGearing);
    final var inchesPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerSecond);
    return inchesPerSecond;
  }

  private void resetWheelAngle() {
    final var absolutePosition = getCancoderPosition();
    double rotations = absolutePosition.getRotations();
    double rotationsBeforeGearing = STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(rotations);
    steerMotor.setRotorPosition(rotationsBeforeGearing);
  }

  private final Rotation2d getCancoderPosition() {
    return getRawCancoderPosition().minus(constants.angleOffset);
  }

  private Rotation2d getRawCancoderPosition() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
  }
}
