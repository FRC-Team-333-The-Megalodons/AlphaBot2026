package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotMetrics;

import java.util.function.Supplier;

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

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public boolean atTarget(double degrees) {
    return io.atTarget(degrees);
  }

  public Command rotateTo(double degrees, boolean waitForCompletion) {
    return waitForCompletion
        ? run(() -> io.moveTo(degrees)).until(() -> io.atTarget(degrees))
        : runOnce(() -> io.moveTo(degrees));
  }

  public Command goUp() {
    return rotateTo(PivotConstants.kUpAngleDeg, true).withName("Pivot.goUp");
  }

  public Command goDown() {
    return rotateTo(PivotConstants.kDownAngleDeg, true).withName("Pivot.goDown");
  }

  public Command motionMagicTo(double degrees, boolean waitForCompletion) {
    return waitForCompletion
        ? run(() -> io.motionMagicTo(degrees)).until(() -> io.atTarget(degrees))
        : runOnce(() -> io.motionMagicTo(degrees));
  }

  public Command motionMagicUp() {
    return motionMagicTo(PivotConstants.kUpAngleDeg, true).withName("Pivot.motionMagicUp");
  }

  public Command motionMagicDown() {
    return motionMagicTo(PivotConstants.kDownAngleDeg, true).withName("Pivot.motionMagicDown");
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }
}
