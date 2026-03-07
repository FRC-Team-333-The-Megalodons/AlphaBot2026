package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOKrakenSim implements FlywheelIO {

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.008, 1.0),
          DCMotor.getKrakenX60(1),
          0.001);

  
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV);

  private double appliedVolts = 0.0;

  // Track the current RPM target so atTarget() can compare against it
  private double targetRPM = 0.0;

  public FlywheelIOKrakenSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
  }


  @Override
  public void moveTo(double rpm) {
    targetRPM = rpm;
    // kV is in V per rot/s — convert RPM → RPS for the feedforward calculation
    double volts = ff.calculate(rpm / 60.0);
    setVoltage(volts);
  }

  
  @Override
  public boolean atTarget(double rpm) {
    double currentRPM =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    return Math.abs(rpm) > 0
        && Math.abs(currentRPM - Math.abs(rpm)) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
