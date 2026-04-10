package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.BatteryLogger;
import frc.robot.interfaces.Characterizable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase implements Characterizable {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final BatteryLogger batteryLogger;

  public Pivot(PivotIO io, BatteryLogger batteryLogger) {
    this.io = io;
    this.batteryLogger = batteryLogger;

    SmartDashboard.putData("Pivot/Zero Encoder", zeroEncoder());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    batteryLogger.reportCurrentUsage("Mechanisms/Pivot", false, inputs.supplyAmps);
  }

  public double getPositionDeg() {
    return io.getPositionDeg();
  }

  public boolean atTarget(double degrees) {
    return io.atTarget(degrees);
  }

  public boolean isAtPosition(double degrees) {
    return Math.abs(getPositionDeg() - degrees) < PivotConstants.AUTON_POSITION_TOLERANCE_DEG;
  }

  public boolean isUp() {
    return isAtPosition(PivotConstants.kUpAngleDeg);
  }

  public boolean isDown() {
    return isAtPosition(PivotConstants.kDownAngleDeg);
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
              io.moveTo(PivotConstants.kDownAngleDeg);
            }
            return;
          }

          // Intake released + shooter active - slowing up
          if (!intakeActive && shooterActive) {
            if (!atTarget(PivotConstants.kUpAngleDeg)) {
              io.moveTo(PivotConstants.kUpAngleDeg);
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

  public Command slowRaise() {
    return runEnd(() -> io.setVoltage(PivotConstants.SLOW_RAISE_VOLTAGE), () -> io.setVoltage(0.0))
        .withTimeout(PivotConstants.SLOW_RAISE_DURATION_SEC)
        .withName("Pivot.slowRaise");
  }

  public Command zeroEncoder() {
    return Commands.runOnce(io::zeroPosition, this)
        .ignoringDisable(true)
        .withName("Pivot.zeroEncoder");
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
