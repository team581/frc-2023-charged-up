// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.managers.SuperstructureManager;
import frc.robot.util.LifecycleSubsystem;

public class LightsSubsystem extends LifecycleSubsystem {
  private final CANdle candle;
  private final IntakeSubsystem intake;
  private final SuperstructureManager superstructure;
  private Color8Bit color = new Color8Bit(255, 255, 255);
  private BlinkPattern blinkPattern = BlinkPattern.SOLID;

  public LightsSubsystem(
      CANdle candle, IntakeSubsystem intake, SuperstructureManager superstructure) {
    this.superstructure = superstructure;
    this.intake = intake;
    this.candle = candle;
  }

  @Override
  public void robotPeriodic() {
    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color.red, color.green, color.blue);
    } else {
      StrobeAnimation animation = new StrobeAnimation(color.red, color.green, color.blue);

      candle.animate(animation);
    }
  }
}
