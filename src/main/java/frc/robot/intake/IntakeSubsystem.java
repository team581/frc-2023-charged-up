// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
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
  }

  @Override
  public void enabledPeriodic() {
    if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.4);
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.4);
    } else if (mode == IntakeMode.OUTTAKE) {
      if (gamePiece == HeldGamePiece.CUBE) {
        motor.set(TalonFXControlMode.PercentOutput, -0.4);
      } else if (gamePiece == HeldGamePiece.CONE) {
        motor.set(TalonFXControlMode.PercentOutput, 0.4);
      }
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
    if (mode == IntakeMode.INTAKE_CUBE) {
      gamePiece = HeldGamePiece.CUBE;
    } else if (mode == IntakeMode.INTAKE_CONE) {
      gamePiece = HeldGamePiece.CONE;
    }
  }
}
