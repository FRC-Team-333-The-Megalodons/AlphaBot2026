package frc.robot.energy;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger {
  private double totalCurrent = 0.0;
  private double driveCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;
  private double batteryVoltage = 12.6;
  private double rioCurrent = 0.0;

  private Map<String, Double> subsytemCurrents = new HashMap<>();
  private Map<String, Double> subsytemPowers = new HashMap<>();
  private Map<String, Double> subsytemEnergies = new HashMap<>();

  // Setters for Robot.java to update globally
  public void setBatteryVoltage(double volts) {
    this.batteryVoltage = volts;
  }

  public void setRioCurrent(double amps) {
    this.rioCurrent = amps;
  }

  public double getDriveCurrent() {
    return driveCurrent;
  }

  /** Call this from your subsystem periodic() loops */
  public void reportCurrentUsage(String key, boolean isDrive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.abs(amp);

    if (isDrive) {
      driveCurrent += totalAmps;
    }

    double power = totalAmps * batteryVoltage;
    double energy = power * 0.02;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsytemCurrents.put(key, totalAmps);
    subsytemPowers.put(key, power);
    subsytemEnergies.merge(key, energy, Double::sum);

    // Create the tree structure in AdvantageScope
    String[] keys = key.split("/|-");
    if (keys.length < 2) return;

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) subkey += "/";
      subsytemCurrents.merge(subkey, totalAmps, Double::sum);
      subsytemPowers.merge(subkey, power, Double::sum);
      subsytemEnergies.merge(subkey, energy, Double::sum);
    }
  }

  /** Call this at the very end of robotPeriodic() */
  public void periodicAfterScheduler() {
    reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
    reportCurrentUsage("Controls/Radio", false, 0.5); // Radio uses roughly 0.5A

    Logger.recordOutput("EnergyLogger/Current", totalCurrent);
    Logger.recordOutput("EnergyLogger/Power", totalPower);
    Logger.recordOutput("EnergyLogger/Energy_WattHours", totalEnergy / 3600.0);

    for (var entry : subsytemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue());
      subsytemCurrents.put(entry.getKey(), 0.0); // Reset for next loop
    }
    for (var entry : subsytemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue());
      subsytemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy_WattHours/" + entry.getKey(), entry.getValue() / 3600.0);
    }

    // Reset totals before next loop
    totalPower = 0.0;
    totalCurrent = 0.0;
    driveCurrent = 0.0;
  }
}
