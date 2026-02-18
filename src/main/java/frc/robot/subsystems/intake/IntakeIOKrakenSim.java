package frc.robot.subsystems.intake;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOKrakenSim implements IntakeIO {
  private static final LinearSystem<N1, N1, N1> intakePlant =
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 1.0);

  private FlywheelSim sim = new FlywheelSim(intakePlant, DCMotor.getKrakenX60(1), 0.001);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(0.02);
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRps = sim.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
