package frc.robot.visualization;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer {

  private final DoubleSupplier elevatorHeightSupplier;

  private final Supplier<Rotation2d> wristAngleSupplier;

  // Mechanism w/ grid
  private final Mechanism2d mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d midNodeHome = mech2d.getRoot("Mid Node", 27.83, 0);
  private final MechanismLigament2d midNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d highNodeHome = mech2d.getRoot("High Node", 10.58, 0);
  private final MechanismLigament2d highNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d gridHome = mech2d.getRoot("Grid Home", 49.75, 0);
  private final MechanismLigament2d gridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));

  // Robot
  private final MechanismLigament2d bumper = gridHome.append(
    new MechanismLigament2d("Bumper", 33, 0, 60, new Color8Bit(Color.kRed)));
    private final MechanismRoot2d elevatorHome = mech2d.getRoot("Elevator Home", 49.75 +25, 10);
  private final MechanismLigament2d elevatorStatic = elevatorHome.append(
    new MechanismLigament2d("Elevator", 40, 115, 15, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d elevatorCarriage = elevatorHome.append(
    new MechanismLigament2d("Elevator Carriage", 15, 115, 10, new Color8Bit(Color.kBlue)));
  private final MechanismLigament2d elevatorSupport = elevatorCarriage.append(
    new MechanismLigament2d("Elevator Support", 20, 180 - 115, 10, new Color8Bit(Color.kBlue)));




  public ElevatorVisualizer(DoubleSupplier elevatorHeightSupplier, Supplier<Rotation2d> wristAngleSupplier) {
    this.elevatorHeightSupplier = elevatorHeightSupplier;
    this.wristAngleSupplier = wristAngleSupplier;

    SmartDashboard.putData("ElevatorVisualization", mech2d);
    Logger.getInstance().recordOutput("ElevatorVisualization", mech2d);
  }


  public void update() {
    elevatorCarriage.setLength(this.elevatorHeightSupplier.getAsDouble() + 15);



  }

}
