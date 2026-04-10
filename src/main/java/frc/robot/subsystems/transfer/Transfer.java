package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.BatteryLogger;
import frc.robot.interfaces.Characterizable;
import frc.robot.util.LiveTuning;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase implements Characterizable {
  private final TransferIO io;
  private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();
  private final BatteryLogger batteryLogger;

  public Transfer(TransferIO io, BatteryLogger batteryLogger) {
    this.io = io;
    this.batteryLogger = batteryLogger;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);
    Logger.recordOutput("Transfer/VelocityRPM", inputs.velocityRpm);
    batteryLogger.reportCurrentUsage("Mechanisms/Transfer", false, inputs.supplyAmps);
  }

  public Command feedShooter() {
    return runEnd(() -> io.moveTo(TransferConstants.TARGET_RPM), () -> io.setVoltage(0.0));
  }

  public Command feedAt(double rpm) {
    return runEnd(() -> io.moveTo(rpm), () -> io.setVoltage(0.0));
  }

  public void runVelocity() {
    io.moveTo(TransferConstants.TARGET_RPM);
  }

  public boolean currentVelocityTarget()
  {
    return io.getCurrentRPM() >= 100; // TODO: Make constant
  }

  public boolean atTarget() {
    return io.atTarget(TransferConstants.TARGET_RPM);
  }

  public Command feedShooterVoltage() {
    return runEnd(() -> io.setVoltage(TransferConstants.FEED_VOLTAGE), () -> io.setVoltage(0.0));
  }

  public Command feedProportional(DoubleSupplier flywheelRPMSupplier) {
    return runEnd(
            () -> {
              double flywheelRPM = flywheelRPMSupplier.getAsDouble();
              double ratio =
                  LiveTuning.getDouble(
                      "Transfer/ProportionalRatio", TransferConstants.PROPORTIONAL_FEED_RATIO);
              double transferRPM = flywheelRPM * ratio;

              transferRPM = Math.max(transferRPM, TransferConstants.MIN_FEED_RPM);

              io.moveTo(transferRPM);
              Logger.recordOutput("Transfer/FlywheelRPMInput", flywheelRPM);
              Logger.recordOutput("Transfer/ProportionalTargetRPM", transferRPM);
              Logger.recordOutput("Transfer/ProportionalRatio", ratio);
            },
            () -> io.setVoltage(0.0))
        .withName("Transfer.feedProportional");
  }

  public Command feedAdditive(DoubleSupplier flywheelRPMSupplier) {
    return runEnd(
            () -> {
              double flywheelRPM = flywheelRPMSupplier.getAsDouble();
              double offset =
                  LiveTuning.getDouble(
                      "Transfer/AdditiveOffsetRPM", TransferConstants.ADDITIVE_FEED_OFFSET_RPM);
              double transferRPM = flywheelRPM + offset;

              transferRPM = Math.max(transferRPM, TransferConstants.MIN_FEED_RPM);

              io.moveTo(transferRPM);
              Logger.recordOutput("Transfer/FlywheelRPMInput", flywheelRPM);
              Logger.recordOutput("Transfer/AdditiveTargetRPM", transferRPM);
              Logger.recordOutput("Transfer/AdditiveOffset", offset);
            },
            () -> io.setVoltage(0.0))
        .withName("Transfer.feedAdditive");
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
                  log.motor("transfer-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(RotationsPerSecond.of(inputs.velocityRpm / 60.0));
                },
                this));

    return Commands.sequence(
        Commands.print("Starting Transfer SysId"),
        runSysIdSequence(routine),
        Commands.print("Transfer SysId Completed"));
  }
}
