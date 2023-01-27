// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonController extends XboxController {
  public final JoystickButton aButton;
  public final JoystickButton bButton;
  public final JoystickButton xButton;
  public final JoystickButton yButton;

  public final Trigger leftTrigger;
  public final Trigger rightTrigger;

  public final JoystickButton leftBumper;
  public final JoystickButton rightBumper;

  public final JoystickButton leftStick;
  public final JoystickButton rightStick;

  public final JoystickButton backButton;
  public final JoystickButton startButton;

  public ButtonController(int port) {
    super(port);

    xButton = new JoystickButton(this, XboxController.Button.kX.value);
    aButton = new JoystickButton(this, XboxController.Button.kA.value);
    bButton = new JoystickButton(this, XboxController.Button.kB.value);
    yButton = new JoystickButton(this, XboxController.Button.kY.value);

    leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
    rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);

    leftTrigger = new Trigger(() -> getLeftTriggerAxis() > 0.5);
    rightTrigger = new Trigger(() -> getRightTriggerAxis() > 0.5);

    backButton = new JoystickButton(this, XboxController.Button.kBack.value);
    startButton = new JoystickButton(this, XboxController.Button.kStart.value);

    leftStick = new JoystickButton(this, XboxController.Button.kLeftStick.value);
    rightStick = new JoystickButton(this, XboxController.Button.kRightStick.value);
  }
}
