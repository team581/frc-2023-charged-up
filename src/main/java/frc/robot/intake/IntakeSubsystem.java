// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LifecycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private static final double CUBE_CLAMP_DURATION = 0.1;
  private static final double CONE_CLAMP_DURATION = 0.2;

  // numbers above are placeholders for current limits
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.OPEN;

  private final TalonFX motor;
  private final CANifier sensor;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  private final Timer timer = new Timer();

  private boolean initialGrab = false;

  public IntakeSubsystem(TalonFX motor, CANifier sensor) {
    this.motor = motor;
    this.sensor = sensor;
    motor.setInverted(false);

    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 25, 30, 0.2));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 15, 25, 0.2));

    timer.start();

    motor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Intake/Voltage", motor.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Intake/HasGamePiece", hasGamePiece());
    Logger.getInstance().recordOutput("Intake/Timer", timer.get());
  }

  @Override
  public void enabledPeriodic() {
    double filteredCurrent = currentFilter.calculate(motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Intake/FilteredCurrent", filteredCurrent);

    double outputVoltage = 0;

    if (mode == IntakeMode.OPEN) {
      if (gamePiece == HeldGamePiece.NOTHING) {
        outputVoltage = 0;
      } else {
        outputVoltage = -0.2;
      }

      if (filteredCurrent > 5) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    } else if (gamePiece == HeldGamePiece.CUBE) {
      outputVoltage = 0.1;
      initialGrab = false;
    } else if (gamePiece == HeldGamePiece.CONE) {
      outputVoltage = 0.1;
      initialGrab = false;
    } else if (mode == IntakeMode.HOLDING_CUBE) {
      outputVoltage = 0.7;

      if (!initialGrab && hasGamePiece()) {
        timer.restart();
        initialGrab = true;
      }

      if (initialGrab && timer.hasElapsed(CUBE_CLAMP_DURATION)) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.HOLDING_CONE) {
      outputVoltage = 0.7;

      if (!initialGrab && hasGamePiece()) {
        timer.restart();
        initialGrab = true;
      }

      if (initialGrab && timer.hasElapsed(CONE_CLAMP_DURATION)) {
        gamePiece = HeldGamePiece.CONE;
      }

    } else {
      outputVoltage = 0;
    }

    Logger.getInstance().recordOutput("Intake/GoalVoltage", outputVoltage);
    motor.set(TalonFXControlMode.PercentOutput, outputVoltage);
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

  private boolean hasGamePiece() {
    return motor.isFwdLimitSwitchClosed() == 0;
  }
}
