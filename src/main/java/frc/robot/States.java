// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.intake.IntakeMode;
import frc.robot.managers.SuperstructureState;

public class States {
  public static final SuperstructureState STOWED =
      new SuperstructureState(Positions.STOWED, IntakeMode.STOPPED, true);
  public static final SuperstructureState FULL_EXTENSION =
      new SuperstructureState(Positions.FULL_EXTENSION, IntakeMode.STOPPED, true);

  public static final SuperstructureState INTAKING_CUBE_FLOOR =
      new SuperstructureState(Positions.INTAKING_CUBE_FLOOR, IntakeMode.INTAKE_CUBE, true);
  public static final SuperstructureState INTAKING_CUBE_SHELF =
      new SuperstructureState(Positions.INTAKING_CUBE_SHELF, IntakeMode.INTAKE_CUBE, true);
  public static final SuperstructureState CUBE_NODE_LOW =
      new SuperstructureState(Positions.CUBE_NODE_LOW, IntakeMode.OUTTAKE_CUBE, false);
  public static final SuperstructureState CUBE_NODE_MID =
      new SuperstructureState(Positions.CUBE_NODE_MID, IntakeMode.OUTTAKE_CUBE, false);
  public static final SuperstructureState CUBE_NODE_HIGH =
      new SuperstructureState(Positions.CUBE_NODE_HIGH, IntakeMode.OUTTAKE_CUBE, false);

  public static final SuperstructureState INTAKING_CONE_FLOOR =
      new SuperstructureState(Positions.INTAKING_CONE_FLOOR, IntakeMode.INTAKE_CONE, true);
  public static final SuperstructureState INTAKING_CONE_SHELF =
      new SuperstructureState(Positions.INTAKING_CONE_SHELF, IntakeMode.INTAKE_CONE, true);
  public static final SuperstructureState CONE_NODE_LOW =
      new SuperstructureState(Positions.CONE_NODE_LOW, IntakeMode.OUTTAKE_CONE, false);
  public static final SuperstructureState CONE_NODE_MID =
      new SuperstructureState(Positions.CONE_NODE_MID, IntakeMode.OUTTAKE_CONE, false);
  public static final SuperstructureState CONE_NODE_HIGH =
      new SuperstructureState(Positions.CONE_NODE_HIGH, IntakeMode.OUTTAKE_CONE, false);

  private States() {}
}
