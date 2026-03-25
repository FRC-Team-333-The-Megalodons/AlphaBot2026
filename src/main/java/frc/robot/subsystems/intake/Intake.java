package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Characterizable;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Characterizable {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/VelocityRPM", inputs.velocityRpm);
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

  public Command dynamicIngest(DoubleSupplier maxSpeedSupplier) {
    return runEnd(
        () -> {
          double currentRobotSpeed = maxSpeedSupplier.getAsDouble();
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
}
