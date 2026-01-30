package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOKrakenSim implements HoodIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getKrakenX44(1), 0.1, HoodConstants.GEAR_RATIO),
          DCMotor.getKrakenX44(1),
          HoodConstants.GEAR_RATIO,
          0.2,
          0.0,
          Math.toRadians(60),
          true,
          0.0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.update(0.02);
    inputs.positionRad = sim.getAngleRads();
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
