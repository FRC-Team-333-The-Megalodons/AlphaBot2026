package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.energy.BatteryLogger;
import frc.robot.util.LiveTuning;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final BatteryLogger batteryLogger;

  public Climber(ClimberIO io, BatteryLogger batteryLogger) {
    this.io = io;
    this.batteryLogger = batteryLogger;

    // ── SmartDashboard commands ──
    SmartDashboard.putData("Climber/Zero Climber Encoder", zeroEncoder());
    SmartDashboard.putData("Climber/Go To Zero", goToZero());
    SmartDashboard.putData("Climber/Go To Extended", extend());
    SmartDashboard.putData("Climber/Go To Climb (Retract)", retract());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    batteryLogger.reportCurrentUsage("Mechanisms/Climber", false, inputs.supplyAmps);
    LiveTuning.publish("Climber/PostionRot", inputs.positionRot);
    LiveTuning.publish("Climber/HasZeroed", inputs.hasZeroed);
  }

  public double getPositionRot() {
    return inputs.positionRot;
  }

  public boolean hasZeroed() {
    return inputs.hasZeroed;
  }

  public boolean atTarget(double positionRot) {
    return io.atTarget(positionRot);
  }

  /** Returns true if the climber is at the extended climbing position. */
  public boolean isExtended() {
    return io.atTarget(ClimberConstants.kClimbPosition);
  }

  /** Returns true if the climber is at the retracted (climbed) position. */
  public boolean isRetracted() {
    return io.atTarget(ClimberConstants.kStowedPosition);
  }

  public boolean isHighEnoughToHitTunnel()
  {
    return io.pastTarget(ClimberConstants.kClimberUpEnoughToHitTunnel);
  }

  public Command zeroEncoder() {
    return Commands.runOnce(io::zeroPosition, this)
        .ignoringDisable(true)
        .withName("Climber.zeroEncoder");
  }

  public Command goToZero() {
    return run(() -> io.moveTo(ClimberConstants.kStowedPosition))
        .until(() -> io.atTarget(ClimberConstants.kStowedPosition))
        .withName("Climber.goToZero");
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

  public Command driveDown(double percent) {
    return runEnd(() -> io.setDutyCycle(-Math.abs(percent)), io::stop)
        .withName("Climber.driveDown");
  }

  public Command runVoltage(double volts) {
    return runEnd(() -> io.setVoltage(volts), io::stop)
        .withName("Climber.runVoltage(" + volts + ")");
  }

  public Command fullClimbSequence() {
    return Commands.sequence(extend(), Commands.waitSeconds(0.3), retract())
        .withName("Climber.fullClimbSequence");
  }

  public Command extendWithTimeOut() {
    return Commands.sequence(extend().withTimeout(1));
  }

  public Command retractWithTimeOut() {
    return Commands.sequence(retract().withTimeout(1));
  }

  public Command stop() {
    return runOnce(io::stop).withName("Climber.stop");
  }
}
