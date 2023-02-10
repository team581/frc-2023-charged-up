// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeMode;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.ScoringState;
import frc.robot.managers.SuperstructureManager;
import frc.robot.util.LifecycleSubsystem;

public class LightsSubsystem extends LifecycleSubsystem {
  private final CANdle candle;
  private final IntakeSubsystem intake;
  private final SuperstructureManager superstructure;
  private final LocalizationSubsystem localization;
  private Color8Bit color = new Color8Bit(255, 255, 255);
  private BlinkPattern blinkPattern = BlinkPattern.SOLID;

  public LightsSubsystem(
      CANdle candle,
      IntakeSubsystem intake,
      SuperstructureManager superstructure,
      LocalizationSubsystem localization) {
    this.superstructure = superstructure;
    this.intake = intake;
    this.candle = candle;
    this.localization = localization;
  }

  @Override
  public void robotPeriodic() {

    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        color = new Color8Bit(Color.kBlue);
        blinkPattern = BlinkPattern.SOLID;
      } else {
        color = new Color8Bit(Color.kRed);
        blinkPattern = BlinkPattern.SOLID;
      }
    } else {
      ScoringState scoringState = superstructure.getScoringState();
      HeldGamePiece gamePiece = intake.getGamePiece();
      IntakeMode intakeMode = intake.getMode();
      HeldGamePiece superstructureMode = superstructure.getMode();

      if (scoringState == ScoringState.ALIGNING || scoringState == ScoringState.SCORING) {
        if (localization.isVisionWorking()) {
          color = new Color8Bit(Color.kGreen);
        } else {
          color = new Color8Bit(Color.kRed);
        }
        blinkPattern = BlinkPattern.BLINK_SLOW;
      } else if (scoringState == ScoringState.READY) {
        if (localization.isVisionWorking()) {
          color = new Color8Bit(Color.kGreen);
        } else {
          color = new Color8Bit(Color.kRed);
        }
        blinkPattern = BlinkPattern.BLINK_FAST;
      } else if (gamePiece == HeldGamePiece.CUBE) {
        if (intakeMode == IntakeMode.INTAKE_CUBE) {
          color = new Color8Bit(Color.kPurple);
          blinkPattern = BlinkPattern.BLINK_FAST;
        } else {
          color = new Color8Bit(Color.kPurple);
          blinkPattern = BlinkPattern.SOLID;
        }
      } else if (gamePiece == HeldGamePiece.CONE) {
        if (intakeMode == IntakeMode.INTAKE_CONE) {
          color = new Color8Bit(Color.kYellow);
          blinkPattern = BlinkPattern.BLINK_FAST;
        } else {
          color = new Color8Bit(Color.kYellow);
          blinkPattern = BlinkPattern.SOLID;
        }
      } else {
        if (superstructureMode == HeldGamePiece.CUBE) {
          color = new Color8Bit(Color.kPurple);
          blinkPattern = BlinkPattern.BLINK_SLOW;
        } else if (superstructureMode == HeldGamePiece.CONE) {
          color = new Color8Bit(Color.kYellow);
          blinkPattern = BlinkPattern.BLINK_SLOW;
        } else {
          color = new Color8Bit(Color.kWhite);
          blinkPattern = BlinkPattern.SOLID;
        }
      }
    }

    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color.red, color.green, color.blue);
    } else {
      StrobeAnimation animation = new StrobeAnimation(color.red, color.green, color.blue);

      // TODO: Check with Saikiran if this is how to use animations
      candle.animate(animation);
    }
  }
}
