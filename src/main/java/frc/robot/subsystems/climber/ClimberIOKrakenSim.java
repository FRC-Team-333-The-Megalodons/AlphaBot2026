package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOKrakenSim implements ClimberIO {

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimberConstants.GEAR_RATIO,
          4.0,
          Units.inchesToMeters(1.0),
          0.0,
          Units.inchesToMeters(24.0),
          true,
          0.0);

  private final PIDController simController =
      new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

  private double appliedVolts = 0.0;
  private boolean closedLoopMode = false;
  private double targetPositionRot = 0.0;
  private boolean hasZeroed = false;

  public ClimberIOKrakenSim() {
    hasZeroed = true;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (closedLoopMode) {
      double currentPos = sim.getPositionMeters();
      appliedVolts = simController.calculate(currentPos, targetPositionRot);
      appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    boolean limitTriggered = sim.getPositionMeters() <= 0.001;

    inputs.positionRot = sim.getPositionMeters();
    inputs.velocityRps = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.limitSwitchTriggered = limitTriggered;
    inputs.hasZeroed = hasZeroed;
  }

  @Override
  public void moveTo(double positionRot) {
    closedLoopMode = true;
    targetPositionRot = positionRot;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopMode = false;
    appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
  }

  @Override
  public void setDutyCycle(double percent) {
    setVoltage(percent * 12.0);
  }

  @Override
  public void stop() {
    closedLoopMode = false;
    appliedVolts = 0.0;
  }

  @Override
  public void zeroPosition() {
    hasZeroed = true;
  }

  @Override
  public boolean atTarget(double positionRot) {
    double currentPos = sim.getPositionMeters();
    double currentVel = sim.getVelocityMetersPerSecond();
    boolean atPosition =
        Math.abs(positionRot - currentPos) < ClimberConstants.POSITION_TOLERANCE_ROT;
    boolean notMoving = Math.abs(currentVel) < ClimberConstants.VELOCITY_TOLERANCE_RPS;
    return atPosition && notMoving;
  }
}
