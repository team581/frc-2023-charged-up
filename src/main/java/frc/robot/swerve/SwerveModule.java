// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.CircleConverter;
import frc.robot.util.CtreModuleState;
import frc.robot.util.GearingConverter;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final GearingConverter STEER_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(Config.SWERVE_STEER_GEARING_REDUCTION);

  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(Config.SWERVE_DRIVE_GEARING_REDUCTION);
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      CircleConverter.fromDiameter(Units.inchesToMeters(4));

  private final SwerveModuleConstants constants;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder encoder;
  private final PositionVoltage steerMotorControl = new PositionVoltage(0, true, 0, 0, false);
  private final VelocityVoltage driveVoltageClosedLoopRequest =
      new VelocityVoltage(0, true, 0, 0, false);
  private Rotation2d previousAngle = new Rotation2d();
  private double commandedDriveVelocity = 0;
  private boolean rotorPositionSet = false;

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    com.ctre.phoenixpro.configs.TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    driveMotorConfigs.Slot0.kP = Config.SWERVE_DRIVE_KP;
    driveMotorConfigs.Slot0.kI = Config.SWERVE_DRIVE_KI;
    driveMotorConfigs.Slot0.kD = Config.SWERVE_DRIVE_KD;
    driveMotorConfigs.Slot0.kV = Config.SWERVE_DRIVE_KV;
    driveMotorConfigs.Slot0.kS = Config.SWERVE_DRIVE_KS;

    driveMotorConfigs.Voltage.PeakForwardVoltage = 12;
    driveMotorConfigs.Voltage.PeakReverseVoltage = -12;

    driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 35;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    if (constants.driveInversion) {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = driveMotor.getConfigurator().apply(driveMotorConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    Slot0Configs steerMotorSlot0Configs = new Slot0Configs();
    MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs steerMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    ClosedLoopGeneralConfigs steerMotorClosedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
    steerMotorClosedLoopGeneralConfigs.ContinuousWrap = false;
    steerMotorSlot0Configs.kV = Config.SWERVE_STEER_KV;
    steerMotorSlot0Configs.kP = Config.SWERVE_STEER_KP;
    steerMotorSlot0Configs.kI = Config.SWERVE_STEER_KI;
    steerMotorSlot0Configs.kD = Config.SWERVE_STEER_KD;
    steerMotorSlot0Configs.kS = Config.SWERVE_STEER_KS;
    steerMotorCurrentLimitsConfigs.SupplyCurrentLimit = 35;
    steerMotorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    steerMotorOutputConfigs.DutyCycleNeutralDeadband = 0;
    if (constants.angleInversion) {
      steerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      steerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    steerMotor.getConfigurator().apply(steerMotorOutputConfigs);

    steerMotor.getConfigurator().apply(steerMotorOutputConfigs);
    steerMotor.getConfigurator().apply(steerMotorSlot0Configs);
    steerMotor.getConfigurator().apply(steerMotorCurrentLimitsConfigs);
    steerMotor.getConfigurator().apply(steerMotorClosedLoopGeneralConfigs);
  }

  public void setDesiredState(SwerveModuleState state, boolean OpenLoop) {
    final var steerMotorPosition = getSteerMotorPosition();
    state = CtreModuleState.optimize(state, steerMotorPosition);

    double commandedSteerPosition =
        STEER_MOTOR_GEARING_CONVERTER.gearingToMotor(state.angle.getRotations());
    steerMotor.setControl(steerMotorControl.withPosition(commandedSteerPosition));

    boolean isStopped =
        Math.abs(state.speedMetersPerSecond)
            <= SwerveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.01;
    Rotation2d angle = isStopped ? this.previousAngle : state.angle;
    this.previousAngle = angle;

    var wheelRotationsPerSecond =
        DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRotations(state.speedMetersPerSecond);
    var motorRotationsPerSecond =
        DRIVE_MOTOR_GEARING_CONVERTER.gearingToMotor(wheelRotationsPerSecond);

    this.commandedDriveVelocity = Units.metersToInches(state.speedMetersPerSecond);
    driveMotor.setControl(driveVoltageClosedLoopRequest.withVelocity(motorRotationsPerSecond));
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = Units.inchesToMeters(getDriveMotorVelocity());

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  public SwerveModulePosition getPosition() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorPosition = getDriveMotorPosition();

    return new SwerveModulePosition(driveMotorPosition, steerMotorPosition);
  }

  public void logValues() {
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Drive motor velocity (inches per/sec)",
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
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Steer Motor Position",
            getSteerMotorPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Drive Motor FF",
            driveMotor.getClosedLoopFeedForward().getValue());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + this.constants.corner.toString() + "/Drive Motor Reference",
            driveMotor.getClosedLoopReference().getValue());
    Logger.getInstance()
        .recordOutput(
            "Swerve/"
                + this.constants.corner.toString()
                + "/Drive Motor Commanded Velocity (inches/sec)",
            this.commandedDriveVelocity);
  }

  private Rotation2d getSteerMotorPosition() {
    double rotationsBeforeGearing = steerMotor.getPosition().getValue();
    double rotations = STEER_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(rotationsBeforeGearing);
    return Rotation2d.fromRotations(rotations);
  }

  private double getDriveMotorPosition() {
    final var rotationsBeforeGearing = driveMotor.getPosition().getValue();
    final var rotations =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(rotationsBeforeGearing);

    final var meters = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotations);
    return meters;
  }

  private double getDriveMotorVelocity() {
    final var rotationsPerSecondBeforeGearing = driveMotor.getVelocity().getValue();
    final var rotationsPerSecond =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(rotationsPerSecondBeforeGearing);
    final var metersPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerSecond);
    final var inchesPerSecond = metersPerSecond * 39.37;
    return inchesPerSecond;
  }

  public void resetWheelAngle() {
    if (!rotorPositionSet) {
      final var absolutePosition = getCancoderPosition();
      double rotations = absolutePosition.getRotations();
      double rotationsBeforeGearing = STEER_MOTOR_GEARING_CONVERTER.gearingToMotor(rotations);
      StatusCode status = steerMotor.setRotorPosition(rotationsBeforeGearing);
      if (!status.isError()) {
        rotorPositionSet = true;
      }
    }
  }

  private final Rotation2d getCancoderPosition() {
    return getRawCancoderPosition().minus(constants.angleOffset);
  }

  private Rotation2d getRawCancoderPosition() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
  }
}
