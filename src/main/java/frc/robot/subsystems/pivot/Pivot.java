package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotMetrics;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    RobotMetrics.start("PivotPeriodic");
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    RobotMetrics.stop("PivotPeriodic");
  }

  public Command setAngleCommand(double rad) {
    return runOnce(() -> this.setGoal(rad));
  }

  public void setGoal(double rad) {
    io.setPosition(rad);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }
}
