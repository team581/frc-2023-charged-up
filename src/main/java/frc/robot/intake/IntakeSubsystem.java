// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 15, 25, 0.2);

  // numbers above are placeholders for current limits
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.STOPPED;

  private final TalonFX motor;

  private final LinearFilter coneFilterIntakeCurrent =
      LinearFilter.movingAverage(Config.IS_SPIKE ? 48 : 30); // Was 24 for spike
  private final LinearFilter cubeFilterIntakeCurrent =
      LinearFilter.movingAverage(Config.IS_SPIKE ? 20 : 10); // Was 10 for spike
  private final LinearFilter coneFilterOuttakeCurrent =
      LinearFilter.movingAverage(Config.IS_SPIKE ? 28 : 30); // Was 14 for spike
  private final LinearFilter cubeFilterOuttakeCurrent =
      LinearFilter.movingAverage(Config.IS_SPIKE ? 20 : 10); // Was 10 for spike

  private final Debouncer coneFilterSensor = new Debouncer(5 * 0.02, DebounceType.kBoth);
  private final Debouncer cubeFilterSensor = new Debouncer(5 * 0.02, DebounceType.kBoth);

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE);

    this.motor = motor;
    motor.setInverted(Config.INVERTED_INTAKE);
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
    motor.overrideLimitSwitchesEnable(false);
  }

  private boolean sensorHasCube() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  private boolean sensorHasCone() {
    return motor.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Intake/Voltage", motor.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Intake/ConeIntakeSensor", sensorHasCone());
    Logger.getInstance().recordOutput("Intake/CubeIntakeSensor", sensorHasCube());
  }

  @Override
  public void enabledPeriodic() {
    double coneIntakeCurrent = coneFilterIntakeCurrent.calculate(motor.getStatorCurrent());
    double cubeIntakeCurrent = cubeFilterIntakeCurrent.calculate(motor.getStatorCurrent());
    double coneOuttakeCurrent = coneFilterOuttakeCurrent.calculate(motor.getStatorCurrent());
    double cubeOuttakeCurrent = cubeFilterOuttakeCurrent.calculate(motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Intake/FilteredConeIntakeCurrent", coneIntakeCurrent);
    Logger.getInstance().recordOutput("Intake/FilteredCubeIntakeCurrent", cubeIntakeCurrent);
    Logger.getInstance().recordOutput("Intake/FilteredConeOuttakeCurrent", coneOuttakeCurrent);
    Logger.getInstance().recordOutput("Intake/FilteredCubeOuttakeCurrent", cubeOuttakeCurrent);

    boolean coneSensor = coneFilterSensor.calculate(sensorHasCone());
    boolean cubeSensor = cubeFilterSensor.calculate(sensorHasCube());
    Logger.getInstance().recordOutput("Intake/FilteredConeIntakeSensor", coneSensor);
    Logger.getInstance().recordOutput("Intake/FilteredCubeIntakeSensor", cubeSensor);

    if (mode == IntakeMode.INTAKE_CUBE) {
      if (cubeIntakeCurrent > 35 || cubeSensor) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.INTAKE_CONE) {
      if (coneIntakeCurrent > (Config.IS_SPIKE ? 45 : 70)
          || coneSensor) { // TODO: Edit currents for tyke
        gamePiece = HeldGamePiece.CONE;
      }
    } else if (mode == IntakeMode.OUTTAKE_CUBE) {
      if (cubeOuttakeCurrent < (Config.IS_SPIKE ? 10 : 10) || !cubeSensor) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    } else if (mode == IntakeMode.OUTTAKE_CONE) {
      if (coneOuttakeCurrent < 10 || !coneSensor) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    if (mode == IntakeMode.MANUAL_INTAKE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.5);
    } else if (mode == IntakeMode.MANUAL_OUTTAKE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (mode == IntakeMode.OUTTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.3);
    } else if (mode == IntakeMode.OUTTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.4);
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.075);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.1);
    } else if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.5);
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.5);
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setMode(IntakeMode mode) {
    if (this.mode != mode && (mode == IntakeMode.INTAKE_CONE || mode == IntakeMode.INTAKE_CUBE)) {
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
    if (mode == IntakeMode.OUTTAKE_CONE || mode == IntakeMode.OUTTAKE_CUBE) {
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
    if (mode == IntakeMode.MANUAL_INTAKE || mode == IntakeMode.MANUAL_OUTTAKE) {
      return false;
    }
    return false;
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(HeldGamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public Command getCommand(IntakeMode newGoal) {
    return runOnce(() -> setMode(newGoal))
        .andThen(Commands.waitUntil(() -> atGoal(newGoal)))
        .andThen(Commands.runOnce(() -> setMode(IntakeMode.STOPPED)));
  }
}
