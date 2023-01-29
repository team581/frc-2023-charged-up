// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.StatusCode;
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

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.Slot0.kP = Config.SWERVE_DRIVE_KP;
    driveMotorConfig.Slot0.kI = Config.SWERVE_DRIVE_KI;
    driveMotorConfig.Slot0.kD = Config.SWERVE_DRIVE_KD;
    driveMotorConfig.Slot0.kV = Config.SWERVE_DRIVE_KV;
    driveMotorConfig.Slot0.kS = Config.SWERVE_DRIVE_KS;

    driveMotorConfig.Voltage.PeakForwardVoltage = 12;
    driveMotorConfig.Voltage.PeakReverseVoltage = -12;

    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    if (constants.driveInversion) {
      driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = driveMotor.getConfigurator().apply(driveMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    steerMotorConfig.Slot0.kV = Config.SWERVE_STEER_KV;
    steerMotorConfig.Slot0.kP = Config.SWERVE_STEER_KP;
    steerMotorConfig.Slot0.kI = Config.SWERVE_STEER_KI;
    steerMotorConfig.Slot0.kD = Config.SWERVE_STEER_KD;
    steerMotorConfig.Slot0.kS = Config.SWERVE_STEER_KS;
    steerMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0;
    if (constants.angleInversion) {
      steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      steerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    steerMotor.getConfigurator().apply(steerMotorConfig);
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
    final var absolutePosition = getCancoderPosition();
    double rotations = absolutePosition.getRotations();
    double rotationsBeforeGearing = STEER_MOTOR_GEARING_CONVERTER.gearingToMotor(rotations);
    steerMotor.setRotorPosition(rotationsBeforeGearing);
  }

  private final Rotation2d getCancoderPosition() {
    return getRawCancoderPosition().minus(constants.angleOffset);
  }

  private Rotation2d getRawCancoderPosition() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
  }
}
