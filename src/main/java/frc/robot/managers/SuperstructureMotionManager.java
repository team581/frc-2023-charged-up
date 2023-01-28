// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.LifecycleSubsystem;
import frc.robot.wrist.WristSubsystem;
import java.util.ArrayList;

public class SuperstructureMotionManager extends LifecycleSubsystem {
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private ArrayList<SuperstructurePosition> positionList = new ArrayList<SuperstructurePosition>();
  private SuperstructurePosition currentPoint =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(0));

  public SuperstructureMotionManager(ElevatorSubsystem elevator, WristSubsystem wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public void set(double goalHeight, Rotation2d goalAngle) {
    double wristRange = 90;
    double goalDegrees = goalAngle.getDegrees();
    positionList.clear();

    if (goalDegrees < wristRange && ((goalHeight > 10 && elevator.getHeight() < 10) || (goalHeight < 10 && elevator.getHeight() > 10))) {
      positionList.add(new SuperstructurePosition(elevator.getHeight(), Rotation2d.fromDegrees(135)));
      positionList.add(new SuperstructurePosition(goalHeight, Rotation2d.fromDegrees(135)));
    }

    positionList.add(new SuperstructurePosition(goalHeight, goalAngle));
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
