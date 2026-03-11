package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
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

  public Command coordinatedPivot(Flywheel flywheel, Intake intake) {
    return run(
        () -> {
          boolean intakeActive = Math.abs(intake.getAppliedVolts()) > 0.1;
          boolean shooterActive = flywheel.isPreSpunUp();

          // Intake active + shooter active - pivot down
          if (intakeActive && shooterActive) {
            io.moveTo(PivotConstants.kDownAngleDeg);
            return;
          }

          // Intake active - pivot fast going down
          if (intakeActive && !shooterActive) {
            if (!atTarget(PivotConstants.kDownAngleDeg)) {
              io.moveTo(PivotConstants.kDownAngleDeg); // fast
            }
            return;
          }

          // Intake released + shooter active - slowing up
          if (!intakeActive && shooterActive) {
            if (!atTarget(PivotConstants.kUpAngleDeg)) {
              io.moveTo(PivotConstants.kUpAngleDeg); // slow
            }
            return;
          }

          // Intake released + shooter inactive - pivot stays down
          if (!intakeActive && !shooterActive) {
            io.moveTo(PivotConstants.kDownAngleDeg);
            return;
          }
        });
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
