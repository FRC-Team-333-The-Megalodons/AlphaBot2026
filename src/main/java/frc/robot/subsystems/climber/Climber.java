package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LiveTuning;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    LiveTuning.publish("Climber/PostionRot", inputs.positionRot);
    LiveTuning.publish("Climber/IsTriggered", inputs.limitSwitchTriggered);
    LiveTuning.publish("Climber/HasZeroed", inputs.hasZeroed);
    LiveTuning.publish("Climber/RawPosition", inputs.rawPosition);
  }

  public double getPositionRot() {
    return inputs.positionRot;
  }

  public boolean isLimitSwitchTriggered() {
    return inputs.limitSwitchTriggered;
  }

  public boolean hasZeroed() {
    return inputs.hasZeroed;
  }

  public boolean atTarget(double positionRot) {
    return io.atTarget(positionRot);
  }

  public Command extend() {
    return run(() -> io.moveTo(ClimberConstants.kClimbPosition))
        .until(() -> io.atTarget(ClimberConstants.kClimbPosition))
        .withName("Climber.extend");
  }

  public Command retract() {
    return run(() -> io.moveTo(ClimberConstants.kStowedPosition))
        .until(() -> io.atTarget(ClimberConstants.kStowedPosition))
        .withName("Climber.retract");
  }

  public Command moveTo(double positionRot) {
    return run(() -> io.moveTo(positionRot))
        .until(() -> io.atTarget(positionRot))
        .withName("Climber.moveTo(" + positionRot + ")");
  }

  public Command driveUp(double percent) {
    return runEnd(() -> io.setDutyCycle(Math.abs(percent)), io::stop).withName("Climber.driveUp");
  }

  // Will naturally zero when the limit switch triggers
  public Command driveDown(double percent) {
    return runEnd(() -> io.setDutyCycle(-Math.abs(percent)), io::stop)
        .withName("Climber.driveDown");
  }

  public Command runVoltage(double volts) {
    return runEnd(() -> io.setVoltage(volts), io::stop)
        .withName("Climber.runVoltage(" + volts + ")");
  }

  public Command zeroSequence() {
    return Commands.sequence(
            runEnd(() -> io.setDutyCycle(-0.1), io::stop).until(this::isLimitSwitchTriggered),
            Commands.runOnce(io::zeroPosition))
        .withName("Climber.zeroSequence");
  }

  public Command stop() {
    return runOnce(io::stop).withName("Climber.stop");
  }
}