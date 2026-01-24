package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SpindexerIOKrakenSim implements SpindexerIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), 0.01, SpindexerConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          0.001);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRps = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVelocity(double rps) {
    setVoltage(rps * 0.5); //rough kV -> will tune once i get the robot
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
