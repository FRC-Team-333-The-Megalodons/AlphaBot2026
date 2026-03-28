package frc.robot.subsystems.shooter.turret;

import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends Automatable {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double turretPositionDeg = 0.0;
    public double turretVelocityRPM = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretStatorAmps = 0.0;
    public double turretSupplyAmps = 0.0;

    public double encoder17Rotations = 0.0;
    public double encoder18Rotations = 0.0;
    public double calculatedAbsPositionRot = 0.0;
  }

  public default boolean encodersGood() {
    return true;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretVoltage(double volts) {}

  public default void seedTurretPosition() {}

  public default void stop() {}

  public default boolean atTarget(double angle) {
    return false;
  }

  /**
   * Position-only control. Moves to the target angle with no velocity feedforward. Use for
   * stationary shots or when velocity feedforward is not available.
   */
  public default void moveTo(double degrees) {}

  /**
   * Combined position + velocity tracking. Moves to the target angle while feedforwarding the
   * expected angular velocity of the mechanism.
   */
  public default void moveToWithVelocity(double degrees, double degreesPerSecond) {
    // Default fallback: ignore velocity and just do position control
    moveTo(degrees);
  }
}
