package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.BatteryLogger;
import frc.robot.interfaces.Characterizable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Characterizable {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Supplier<Twist2d> robotVelocitySupplier;
  private final BatteryLogger batteryLogger;

  public Intake(IntakeIO io, Supplier<Twist2d> robotVelocitySupplier, BatteryLogger batteryLogger) {
    this.io = io;
    this.robotVelocitySupplier = robotVelocitySupplier;
    this.batteryLogger = batteryLogger;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/VelocityRPM", inputs.velocityRpm);
    batteryLogger.reportCurrentUsage("Mechanisms/Intake", false, inputs.supplyAmps);
  }

  public Twist2d fieldVelocity() {
    return robotVelocitySupplier.get();
  }

  public Command ingest() {
    return runEnd(() -> io.moveTo(IntakeConstants.INTAKE_RPM), () -> io.setVoltage(0.0));
  }

  public Command eject() {
    return runEnd(() -> io.moveTo(IntakeConstants.EJECT_RPM), () -> io.setVoltage(0.0));
  }

  public Command ingestAt(double rpm) {
    return runEnd(() -> io.moveTo(rpm), () -> io.setVoltage(0.0));
  }

  public boolean atTarget(double rpm) {
    return io.atTarget(rpm);
  }

  public Command ingestVoltage() {
    return runEnd(() -> io.setVoltage(IntakeConstants.INTAKE_VOLTS), () -> io.setVoltage(0.0));
  }

  public Command ejectVoltage() {
    return runEnd(() -> io.setVoltage(IntakeConstants.EJECT_VOLTS), () -> io.setVoltage(0.0));
  }

  public Command dynamicIngest() {
    return runEnd(
        () -> {
          var fieldVelocity = robotVelocitySupplier.get();
          double absX = Math.abs(fieldVelocity.dx);
          double absY = Math.abs(fieldVelocity.dy);
          double currentRobotSpeed = Math.max(absX, absY);
          double targetVolts = io.getVoltageFromSpeed(currentRobotSpeed);
          Logger.recordOutput("Intake/DynamicSpeedInput", currentRobotSpeed);
          Logger.recordOutput("Intake/DynamicVoltsOutput", targetVolts);
          io.setVoltage(targetVolts);
        },
        () -> io.setVoltage(0.0));
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  @Override
  public Command characterize() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null, null),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) -> {
                  log.motor("intake-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(RotationsPerSecond.of(inputs.velocityRpm / 60.0));
                },
                this));

    return Commands.sequence(
        Commands.print("Starting Intake SysId"),
        runSysIdSequence(routine),
        Commands.print("Intake SysId Completed"));
  }

  public boolean isStuck() {
    return io.isStuck();
  }
}
