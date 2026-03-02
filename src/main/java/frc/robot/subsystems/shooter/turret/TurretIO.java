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
    public double turretCurrentAmps = 0.0;

    public double encoder17Rotations = 0.0;
    public double encoder18Rotations = 0.0;
    public double calculatedAbsPositionRot = 0.0;
  }

  public default boolean encodersGood() { return true; };

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretVoltage(double volts) {}

  public default void seedTurretPosition() {}

  public default void stop() {}
}
