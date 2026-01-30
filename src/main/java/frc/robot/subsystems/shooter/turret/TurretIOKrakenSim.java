package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOKrakenSim implements TurretIO {
  // Correctly using LinearSystemId for a turret (treated as a jointed arm)
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getKrakenX60(1), 0.3, TurretConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          TurretConstants.GEAR_RATIO,
          0.5,
          -Math.PI,
          Math.PI,
          false,
          0.0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.update(0.02);
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
