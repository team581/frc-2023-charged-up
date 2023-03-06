// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;
import java.util.ArrayList;

public class SuperstructureMotionManager extends LifecycleSubsystem {
  public final ElevatorSubsystem elevator;
  public final WristSubsystem wrist;
  private final DriveController controller;
  private final ArrayList<SuperstructurePosition> positionList =
      new ArrayList<SuperstructurePosition>();
  private SuperstructurePosition currentPoint = Positions.STOWED;
  private SuperstructurePosition goalPosition = Positions.STOWED;
  private double previousHeight;

  public SuperstructureMotionManager(
      ElevatorSubsystem elevator, WristSubsystem wrist, DriveController controller) {
    super(SubsystemPriority.SUPERSTRUCTURE_MOTION_MANAGER);

    this.elevator = elevator;
    this.wrist = wrist;
    this.controller = controller;
  }

  public void set(SuperstructurePosition newGoal) {
    if (newGoal.height == goalPosition.height
        && newGoal.angle.equals(goalPosition.angle)
        && newGoal.earlyTransitionHeight == goalPosition.earlyTransitionHeight) {
      return;
    }

    goalPosition = newGoal;
    double wristRange = Config.SUPERSTRUCTURE_WRIST_RANGE.getDegrees();
    double goalDegrees = newGoal.angle.getDegrees();
    double wristAngle = wrist.getAngle().getDegrees();
    double intermediatePointDegrees = 50;

    boolean wristGoalInCollisionArea = goalDegrees < wristRange;
    boolean currentWristAngleInCollisionArea = wristAngle < wristRange;
    double goalHeight =
        MathUtil.clamp(newGoal.height, Config.ELEVATOR_MIN_HEIGHT, Config.ELEVATOR_MAX_HEIGHT);
    boolean leavingBumperArea =
        goalHeight > Config.SUPERSTRUCTURE_COLLISION_HEIGHT
            && elevator.getHeight() < Config.SUPERSTRUCTURE_COLLISION_HEIGHT;
    boolean goingToBumperArea =
        goalHeight < Config.SUPERSTRUCTURE_COLLISION_HEIGHT
            && elevator.getHeight() > Config.SUPERSTRUCTURE_COLLISION_HEIGHT;

    positionList.clear();

    if ((wristGoalInCollisionArea || currentWristAngleInCollisionArea)
        && (leavingBumperArea || goingToBumperArea)) {
      positionList.add(
          new SuperstructurePosition(elevator.getHeight(), Rotation2d.fromDegrees(50), -1));
      positionList.add(
          new SuperstructurePosition(goalHeight, Rotation2d.fromDegrees(50), goalHeight / 3));
    }

    positionList.add(new SuperstructurePosition(goalHeight, newGoal.angle, -1));
  }

  public boolean atGoal(SuperstructurePosition position) {
    if ((wrist.atAngle(position.angle) && elevator.atHeight(position.height))
        || ((position.earlyTransitionHeight > previousHeight
            && position.earlyTransitionHeight < elevator.getHeight()))
        || (position.earlyTransitionHeight < previousHeight
            && position.earlyTransitionHeight > elevator.getHeight())) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void enabledInit() {
    positionList.clear();
  }

  @Override
  public void enabledPeriodic() {
    if (atGoal(currentPoint) && !positionList.isEmpty()) {
      currentPoint = positionList.remove(0);
    }
    wrist.setAngle(currentPoint.angle);
    elevator.setGoalPosition(currentPoint.height);
    previousHeight = elevator.getHeight();
  }

  @Override
  public void robotPeriodic() {
    if (goalPosition.height > 1) {
      controller.slowModeToggle(true);
    } else if (elevator.getHeight() < 20) {
      controller.slowModeToggle(false);
    }
  }
}
