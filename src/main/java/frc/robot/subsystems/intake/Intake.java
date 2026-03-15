package frc.robot.subsystems.intake;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotMetrics;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final InterpolatingDoubleTreeMap dynamicIntake = new InterpolatingDoubleTreeMap();

  public Intake(IntakeIO io) {
    this.io = io;
    dynamicIntake.put(0.2, 7.2);
    dynamicIntake.put(0.4, 7.0);
    dynamicIntake.put(0.5, 6.8);
    dynamicIntake.put(0.7, 6.5);
    dynamicIntake.put(0.9, 6.2);
    dynamicIntake.put(1.1, 5.6);
    dynamicIntake.put(1.5, 5.5);
    dynamicIntake.put(1.8, 5.35);
    dynamicIntake.put(2.0, 5.3);
    dynamicIntake.put(2.5, 5.2);
    dynamicIntake.put(5.0, 5.0);
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

          RobotMetrics.recordOutput("Intake/DynamicSpeedInput", currentRobotSpeed);
          RobotMetrics.recordOutput("Intake/DynamicVoltsOutput", targetVolts);

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
