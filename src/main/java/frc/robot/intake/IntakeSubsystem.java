// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2);

  // numbers above are placeholders for current limits
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.OPEN;

  private final TalonFX motor;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  public IntakeSubsystem(TalonFX motor) {
    this.motor = motor;
    motor.setInverted(true);
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Intake/Voltage", motor.getMotorOutputVoltage());
  }

  @Override
  public void enabledPeriodic() {
    double filteredCurrent = currentFilter.calculate(motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Intake/FilteredCurrent", filteredCurrent);

    if (mode == IntakeMode.HOLDING_CUBE) {
      if (filteredCurrent > 40) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.HOLDING_CONE) {
      if (filteredCurrent > 45) {
        gamePiece = HeldGamePiece.CONE;
      }
    } else if (mode == IntakeMode.OPEN) {
      if (filteredCurrent > 5) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    if (mode == IntakeMode.OPEN && gamePiece != HeldGamePiece.NOTHING) {
      motor.set(TalonFXControlMode.PercentOutput, -0.2);
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.075);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.1);
    } else if (mode == IntakeMode.HOLDING_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.2);
    } else if (mode == IntakeMode.HOLDING_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.3);
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setMode(IntakeMode mode) {
    if (this.mode != mode && (mode == IntakeMode.HOLDING_CONE || mode == IntakeMode.HOLDING_CUBE)) {
      gamePiece = HeldGamePiece.NOTHING;
    }
    this.mode = mode;
  }

  public IntakeMode getMode() {
    return mode;
  }

  public boolean atGoal(IntakeMode goal) {
    if (mode != goal) {
      return false;
    }

    if (mode == IntakeMode.OPEN) {
      return gamePiece == HeldGamePiece.NOTHING;
    }
    if (mode == IntakeMode.STOPPED) {
      return true;
    }
    if (mode == IntakeMode.HOLDING_CUBE) {
      return gamePiece == HeldGamePiece.CUBE;
    }

    if (mode == IntakeMode.HOLDING_CONE) {
      return gamePiece == HeldGamePiece.CONE;
    }

    return false;
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }
}
