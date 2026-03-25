package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Characterizable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.util.LiveTuning;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase implements Characterizable {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    LiveTuning.publish("Pivot/AngleDegrees", getPositionDeg());
    LiveTuning.publish("Pivot/AppliedVoltage", io.getAppliedVoltage());
  }

  public double getPositionDeg() {
    return io.getPositionDeg();
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

  public Command goUpOrDownBasedOnMovement(Drive drive) {
    return Commands.either(goUp(), goDown(), drive.isStationarySupplier());
  }

  public Command coordinatedPivot(Flywheel flywheel, Intake intake) {
    return run(
        () -> {
          boolean intakeActive = Math.abs(intake.getAppliedVolts()) > 0.1;
          boolean shooterActive = flywheel.isPreSpunUp() || flywheel.ready();

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

  public Command runPercent(DoubleSupplier percentSupplier) {
    return runEnd(
        () -> io.setVoltage(percentSupplier.getAsDouble() * 12.0), () -> io.setVoltage(0.0));
  }

  @Override
  public Command characterize() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.5),
                Volts.of(4),
                Seconds.of(8), 
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) -> {
                  log.motor("pivot-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularPosition(Rotations.of(inputs.positionDeg / 360.0))
                      .angularVelocity(RotationsPerSecond.of(inputs.velocityRPM / 60.0));
                },
                this));

    return Commands.sequence(
        Commands.print("Starting Pivot SysId"),
        runSysIdSequence(routine),
        Commands.print("Pivot SysId Completed"));
  }
}