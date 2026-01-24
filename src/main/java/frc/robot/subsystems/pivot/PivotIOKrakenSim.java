package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOKrakenSim implements PivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getKrakenX60(1), 0.8, PivotConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          PivotConstants.GEAR_RATIO,
          0.4,
          0.0,
          Math.PI / 2.0,
          true,
          0.0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
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
