package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command spinUpCommand(double rpm) {
    return runEnd(() -> this.setRPM(rpm), this::stop);
  }

  public Command stopShooterCommand() {
    return runOnce(this::stop);
  }

  public void setRPM(double rpm) {
    io.setVelocity(rpm * (Math.PI / 30.0));
  }

  public void stop() {
    io.setVoltage(0.0);
  }
}
