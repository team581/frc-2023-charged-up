// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private static final StatorCurrentLimitConfiguration CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(false, 5, 10, 0.2);

  // numbers above are placeholders for current limits
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.STOPPED;

  private final TalonFX motor;

  public IntakeSubsystem(TalonFX motor) {
    this.motor = motor;
    motor.setInverted(true);
    motor.configStatorCurrentLimit(CURRENT_LIMIT);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getStatorCurrent());
  }

  @Override
  public void enabledPeriodic() {
    // TODO: Tune voltages and currents
    if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.2);
      if (motor.getStatorCurrent() > 10) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.2);
      if (motor.getStatorCurrent() > 20) {
        gamePiece = HeldGamePiece.CONE;
      }
    } else if (mode == IntakeMode.OUTTAKE) {
      if (gamePiece == HeldGamePiece.CUBE) {
        motor.set(TalonFXControlMode.PercentOutput, -0.2);
        if (motor.getStatorCurrent() < 5) {
          gamePiece = HeldGamePiece.NOTHING;
        }
      } else if (gamePiece == HeldGamePiece.CONE) {
        motor.set(TalonFXControlMode.PercentOutput, 0.2);
        if (motor.getStatorCurrent() < 5) {
          gamePiece = HeldGamePiece.NOTHING;
        }
      }
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
  }

  public boolean atGoal(IntakeMode goal) {
    if (mode != goal) {
      return false;
    }
    if (mode == IntakeMode.OUTTAKE) {
      return gamePiece == HeldGamePiece.NOTHING;
    }
    if (mode == IntakeMode.STOPPED) {
      return true;
    }
    if (mode == IntakeMode.INTAKE_CUBE) {
      return gamePiece == HeldGamePiece.CUBE;
    }
    if (mode == IntakeMode.INTAKE_CONE) {
      return gamePiece == HeldGamePiece.CONE;
    }
    return false;
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }
}
