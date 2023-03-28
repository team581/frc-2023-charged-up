// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.config.Config;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.localization.VisionMode;
import frc.robot.managers.ScoringState;
import frc.robot.managers.SuperstructureManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends LifecycleSubsystem {
  /** The duration (in seconds) that lights should be on or off when in fast blink mode. */
  private static final double FAST_BLINK_DURATION = 0.08;

  /** The duration (in seconds) that lights should be on or off when in slow blink mode. */
  private static final double SLOW_BLINK_DURATION = 0.25;

  private final CANdle candle;
  private final IntakeSubsystem intake;
  private final SuperstructureManager superstructure;
  private final LocalizationSubsystem localization;
  private final Timer blinkTimer = new Timer();
  private Color color = Color.kWhite;
  private BlinkPattern blinkPattern = BlinkPattern.SOLID;

  public LightsSubsystem(
      CANdle candle,
      IntakeSubsystem intake,
      SuperstructureManager superstructure,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.LIGHTS);

    this.superstructure = superstructure;
    this.intake = intake;
    this.candle = candle;
    this.localization = localization;

    blinkTimer.start();
  }

  @Override
  public void robotPeriodic() {
    ScoringState scoringState = superstructure.getScoringState();
    HeldGamePiece gamePiece = intake.getGamePiece();
    IntakeMode intakeMode = intake.getMode();
    HeldGamePiece superstructureMode = superstructure.getMode();

    if (DriverStation.isDisabled()) {
      if (FmsSubsystem.isRedAlliance()) {
        color = Color.kRed;
        blinkPattern = BlinkPattern.SOLID;
      } else {
        color = Color.kBlue;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else if (scoringState == ScoringState.ALIGNING || scoringState == ScoringState.SCORING) {
      if (Config.VISION_MODE != VisionMode.FULLY_ENABLED || localization.isVisionWorking()) {
        color = Color.kGreen;
      } else {
        color = Color.kRed;
      }
      blinkPattern = BlinkPattern.BLINK_SLOW;
    } else if (scoringState == ScoringState.READY
        || scoringState == ScoringState.FINISHED_SCORING) {
      if (Config.VISION_MODE != VisionMode.FULLY_ENABLED || localization.isVisionWorking()) {
        color = Color.kGreen;
      } else {
        color = Color.kRed;
      }
      blinkPattern = BlinkPattern.BLINK_FAST;
    } else if (gamePiece == HeldGamePiece.CUBE) {
      if (intakeMode == IntakeMode.INTAKE_CUBE) {
        color = Color.kPurple;
        blinkPattern = BlinkPattern.BLINK_FAST;
      } else {
        color = Color.kPurple;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else if (gamePiece == HeldGamePiece.CONE) {
      if (intakeMode == IntakeMode.INTAKE_CONE) {
        color = Color.kYellow;
        blinkPattern = BlinkPattern.BLINK_FAST;
      } else {
        color = Color.kYellow;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else if (superstructureMode == HeldGamePiece.CUBE) {
      color = Color.kPurple;
      blinkPattern = BlinkPattern.BLINK_SLOW;
    } else if (superstructureMode == HeldGamePiece.CONE) {
      color = Color.kYellow;
      blinkPattern = BlinkPattern.BLINK_SLOW;
    } else {
      color = Color.kWhite;
      blinkPattern = BlinkPattern.SOLID;
    }

    Color8Bit color8Bit = new Color8Bit(color);

    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (blinkPattern == BlinkPattern.BLINK_FAST) {
        onDuration = FAST_BLINK_DURATION;
        offDuration = FAST_BLINK_DURATION * 2;
      } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
        onDuration = SLOW_BLINK_DURATION;
        offDuration = SLOW_BLINK_DURATION * 2;
      }

      if (time >= offDuration) {
        blinkTimer.reset();
        candle.setLEDs(0, 0, 0);
      } else if (time >= onDuration) {
        candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
      }
    }

    Logger.getInstance().recordOutput("Lights/Color", color.toString());
    Logger.getInstance().recordOutput("Lights/BlinkPattern", blinkPattern.toString());
  }
}
