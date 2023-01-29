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
  private static final StatorCurrentLimitConfiguration INTAKE_CUBE_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(false, 40, 40, 1);
  private static final StatorCurrentLimitConfiguration INTAKE_CONE_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(false, 40, 40, 1);
  private static final StatorCurrentLimitConfiguration OUTTAKE_CUBE_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(false, 40, 40, 1);
  private static final StatorCurrentLimitConfiguration OUTTAKE_CONE_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(false, 40, 40, 1);
  // numbers above are placeholders for current limits
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.STOPPED;

  private final TalonFX motor;

  public IntakeSubsystem(TalonFX motor) {
    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getStatorCurrent());
  }

  @Override
  public void enabledPeriodic() {
    if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.4);
      if (motor.getStatorCurrent() > 10) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.4);
      if (motor.getStatorCurrent() > 20) {
        gamePiece = HeldGamePiece.CONE;
      }
    } else if (mode == IntakeMode.OUTTAKE) {
      if (gamePiece == HeldGamePiece.CUBE) {
        motor.set(TalonFXControlMode.PercentOutput, -0.4);
          if (motor.getStatorCurrent() < 5) {
            gamePiece = HeldGamePiece.NOTHING;
          }
      } else if (gamePiece == HeldGamePiece.CONE) {
        motor.set(TalonFXControlMode.PercentOutput, 0.4);
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

    if (mode == IntakeMode.INTAKE_CUBE) {
      motor.configStatorCurrentLimit(INTAKE_CUBE_CURRENT_LIMIT);
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.configStatorCurrentLimit(INTAKE_CONE_CURRENT_LIMIT);
    } else if (mode == IntakeMode.OUTTAKE) {
      if (gamePiece == HeldGamePiece.CUBE) {
        motor.configStatorCurrentLimit(OUTTAKE_CUBE_CURRENT_LIMIT);
      } else if (gamePiece == HeldGamePiece.CONE) {
        motor.configStatorCurrentLimit(OUTTAKE_CONE_CURRENT_LIMIT);
      }
    }
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }
}
