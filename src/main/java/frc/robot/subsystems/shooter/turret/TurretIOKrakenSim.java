package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOKrakenSim implements TurretIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          TurretConstants.kMotorToTurretRatio,
          0.5,
          0.3,
          Units.degreesToRadians(TurretConstants.kMinAngle),
          Units.degreesToRadians(TurretConstants.kMaxAngle),
          false,
          Units.degreesToRadians(0));
  private final PIDController simController =
      new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

  private double appliedVolts = 0.0;
  private boolean closedLoopMode = false;
  private double targetPositionRad = 0.0;
  private double targetVelocityRadPerSec = 0.0;

  public TurretIOKrakenSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoopMode) {
      appliedVolts = simController.calculate(sim.getAngleRads(), targetPositionRad);
      appliedVolts += Math.signum(targetPositionRad - sim.getAngleRads()) * TurretConstants.kS;

      // Velocity feedforward: kV * targetVelocity compensates for the moving setpoint
      appliedVolts += TurretConstants.kV * targetVelocityRadPerSec;

      appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    sim.setInput(appliedVolts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.turretPositionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.turretVelocityRPM = sim.getVelocityRadPerSec() * 60.0;
    inputs.turretAppliedVolts = appliedVolts;
    inputs.turretStatorAmps = sim.getCurrentDrawAmps();

    double turretRotations = Units.degreesToRotations(inputs.turretPositionDeg);

    double enc17Raw =
        turretRotations * (TurretConstants.kTurretGearTeeth / TurretConstants.kEncoder1Teeth);
    inputs.encoder17Rotations = MathUtil.inputModulus(enc17Raw, 0.0, 1.0);

    double enc18Raw =
        turretRotations * (TurretConstants.kTurretGearTeeth / TurretConstants.kEncoder2Teeth);
    inputs.encoder18Rotations = MathUtil.inputModulus(enc18Raw, 0.0, 1.0);

    inputs.calculatedAbsPositionRot =
        calculateAbsolutePosition(inputs.encoder17Rotations, inputs.encoder18Rotations);
  }

  @Override
  public void moveTo(double degrees) {
    closedLoopMode = true;
    targetPositionRad = Units.degreesToRadians(degrees);
    targetVelocityRadPerSec = 0.0; // No velocity feedforward for position-only
  }

  @Override
  public void moveToWithVelocity(double degrees, double degreesPerSecond) {
    closedLoopMode = true;
    targetPositionRad = Units.degreesToRadians(degrees);
    targetVelocityRadPerSec = Units.degreesToRadians(degreesPerSecond);
  }

  @Override
  public void setTurretVoltage(double volts) {
    closedLoopMode = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    closedLoopMode = false;
    appliedVolts = 0.0;
  }

  private double calculateAbsolutePosition(double p17, double p18) {
    double n1 = 17.0;
    double n2 = 18.0;
    double N = 105.0;

    double delta = (n2 * p18) - (n1 * p17);
    long k_diff = Math.round(delta);

    double turretRot = (-k_diff + p17) * (n1 / N);

    double period = (n1 * n2) / N; // ~2.914
    while (turretRot > period / 2.0) turretRot -= period;
    while (turretRot < -period / 2.0) turretRot += period;

    return turretRot;
  }
}
