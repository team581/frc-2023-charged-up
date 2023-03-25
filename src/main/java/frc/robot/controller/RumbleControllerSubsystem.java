// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class RumbleControllerSubsystem extends LifecycleSubsystem {
  private final Timer matchTimer = new Timer();
  private final XboxController controller;

  @Override
  public void autonomousInit() {
    matchTimer.reset();
    matchTimer.start();
  }

  public RumbleControllerSubsystem(XboxController controller) {
    super(SubsystemPriority.RUMBLE_CONTROLLER);
    this.controller = controller;

    new Trigger(() -> matchTimer.hasElapsed(Config.MATCH_DURATION - 90))
        .onTrue(getVibrateCommand());
    new Trigger(() -> matchTimer.hasElapsed(Config.MATCH_DURATION - 60))
        .onTrue(getVibrateCommand());
    new Trigger(() -> matchTimer.hasElapsed(Config.MATCH_DURATION - 30))
        .onTrue(getVibrateCommand());
  }

  private SequentialCommandGroup getVibrateOnceCommand() {
    return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
        .andThen(Commands.waitSeconds(0.1))
        .andThen(() -> controller.setRumble(RumbleType.kBothRumble, 0))
        .andThen(Commands.waitSeconds(0.1));
  }

  private Command getVibrateCommand() {

    return getVibrateOnceCommand()
        .andThen(getVibrateOnceCommand())
        .andThen(getVibrateOnceCommand());
  }
}
