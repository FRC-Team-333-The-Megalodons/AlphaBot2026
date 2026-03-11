package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command ingest() {
    return runEnd(() -> this.run(false), this::stop);
  }

  public Command eject() {
    return runEnd(() -> this.run(true), this::stop);
  }

  public Command dynamicIngest(DoubleSupplier maxSpeedSupplier) {
    return runEnd(
        () -> {
          double currentRobotSpeed = maxSpeedSupplier.getAsDouble();

          double targetVolts = io.getVoltageFromSpeed(currentRobotSpeed);

          Logger.recordOutput("Intake/DynamicSpeedInput", currentRobotSpeed);
          Logger.recordOutput("Intake/DynamicVoltsOutput", targetVolts);

          // 4. Apply the voltage
          io.setVoltage(targetVolts);
        },
        () -> io.setVoltage(0.0) // Safely stop when the button is released
        );
  }

  public void run(boolean forward) {
    io.setVoltage(forward ? IntakeConstants.INTAKE_VOLTS : -IntakeConstants.INTAKE_VOLTS);
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public void stop() {
    io.setVoltage(0.0);
  }
}
