package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double targetRPM = 0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public boolean isAtSpeed() {
    double currentRPM = inputs.velocityRadPerSec * (30.0 / Math.PI);
    return targetRPM > 0
        && Math.abs(currentRPM - targetRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  public Command spinUpCommand(double rpm) {
    return runEnd(() -> this.setRPM(rpm), this::stop);
  }

  public void setRPM(double rpm) {
    this.targetRPM = rpm;
    io.setVelocity(rpm * (Math.PI / 30.0));
  }

  public void stop() {
    this.targetRPM = 0;
    io.setVoltage(0.0);
  }
}
