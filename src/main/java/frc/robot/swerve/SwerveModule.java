package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
  private final static double TICKS_PER_ROTATION = 12.8 * 2048;
  private SwerveModuleConstants constants;
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANcoder encoder;
  private Rotation2d swerveWheelOffset;

  public SwerveModule(SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANcoder encoder, Rotation2d swerveWheeleOffset) {
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

    resetWheelAngle();
  }

  public void resetWheelAngle() {
    final var absolutePosition = cancoderPosition();
    double rotations = absolutePosition.getDegrees() / 360;
    double encoderTicks = rotations * TICKS_PER_ROTATION;
    steerMotor.setSelectedSensorPosition(encoderTicks);
  }

  private final Rotation2d cancoderPosition() {
    return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue()).minus(swerveWheelOffset);
  }
}
