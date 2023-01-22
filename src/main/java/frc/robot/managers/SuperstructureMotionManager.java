package frc.robot.managers;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.LifecycleSubsystem;
import frc.robot.wrist.WristSubsystem;

public class SuperstructureMotionManager extends LifecycleSubsystem{
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private ArrayList<SuperstructurePosition> positionList = new ArrayList<SuperstructurePosition>();

  public SuperstructureMotionManager(ElevatorSubsystem elevator, WristSubsystem wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public void set(double height, Rotation2d angle) {
    positionList.add(new SuperstructurePosition(height, angle));

  }

  public boolean atGoal(SuperstructurePosition position) {
    if (wrist.atAngle(position.angle) && )
  }

  @Override
  public void enabledInit() {
    positionList.clear();
  }

  @Override
  public void enabledPeriodic() {

  }
}
