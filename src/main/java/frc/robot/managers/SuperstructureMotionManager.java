// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Positions;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.LifecycleSubsystem;
import frc.robot.wrist.WristSubsystem;
import java.util.ArrayList;

public class SuperstructureMotionManager extends LifecycleSubsystem {
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final ArrayList<SuperstructurePosition> positionList =
      new ArrayList<SuperstructurePosition>();
  private SuperstructurePosition currentPoint = Positions.STOWED;

  public SuperstructureMotionManager(ElevatorSubsystem elevator, WristSubsystem wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public void set(SuperstructurePosition goalPosition) {
    double wristRange = 13;
    double goalDegrees = goalPosition.angle.getDegrees();
    double wristAngle = wrist.getAngle().getDegrees();

    boolean wristGoalInCollisionArea = goalDegrees < wristRange;
    boolean currentWristAngleInCollisionArea = wristAngle < wristRange;
    double goalHeight = MathUtil.clamp(goalPosition.height, 0.5, 24);
    boolean leavingBumperArea = goalHeight > 26 && elevator.getHeight() < 26;
    boolean goingToBumperArea = goalHeight < 26 && elevator.getHeight() > 26;

    positionList.clear();

    if ((wristGoalInCollisionArea || currentWristAngleInCollisionArea)
        && (leavingBumperArea || goingToBumperArea)) {
      positionList.add(
          new SuperstructurePosition(elevator.getHeight(), Rotation2d.fromDegrees(50)));
      positionList.add(new SuperstructurePosition(goalHeight, Rotation2d.fromDegrees(50)));
    }
    // TODO: Make elevator & wrist move to goal position in fewest movements

    positionList.add(new SuperstructurePosition(goalHeight, goalPosition.angle));
  }

  public boolean atGoal(SuperstructurePosition position) {
    if (wrist.atAngle(position.angle) && elevator.atHeight(position.height)) {
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
  }
}
