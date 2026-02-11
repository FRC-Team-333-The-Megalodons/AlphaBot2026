package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double targetRPM = 0;

  private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  private final SysIdRoutine sysIdRoutine;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    distanceToRPM.put(1.0, -1800.0);
    distanceToRPM.put(1.3, -1850.0);
    distanceToRPM.put(1.5, -1870.0);
    distanceToRPM.put(1.7, -1900.0);
    distanceToRPM.put(1.9, -1960.0);
    distanceToRPM.put(2.1, -2050.0);
    distanceToRPM.put(2.4, -2100.0);
    distanceToRPM.put(2.6, -2130.0);
    distanceToRPM.put(2.8, -2170.0);
    distanceToRPM.put(3.0, -2200.0);
    distanceToRPM.put(3.2, -2225.0);

    distanceToRPM.put(3.4, -2250.0);

    distanceToRPM.put(3.6, -2280.0);
    distanceToRPM.put(3.8, -2340.0);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null, null),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> io.setVoltage(volts.in(Volts)),
                (log) -> {
                  log.motor("flywheel-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec));
                },
                this));
  }

  public double getRPMForDistance(double distanceMeters) {
    return distanceToRPM.get(distanceMeters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double currentRPM = inputs.velocityRadPerSec * (30.0 / Math.PI);
    Logger.recordOutput("Shooter/CurrentRPM", currentRPM);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);

    Logger.processInputs("Shooter", inputs);
  }

  public boolean isAtSpeed() {
    double currentRPM = inputs.velocityRadPerSec * (30.0 / Math.PI);
    return Math.abs(targetRPM) > 0
        && Math.abs(Math.abs(currentRPM) - Math.abs(targetRPM))
            < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  public Command spinUpCommand(double rpm) {
    return runEnd(() -> this.setRPM(rpm), this::stop);
  }

  public void runVelocity(double rpm) {
    this.setRPM(rpm);
  }

  public void runMotionMagic(double rpm) {
    this.setRPM(rpm);
  }

  public Command runMotionMagicTest(double rpm) {
    return runEnd(() -> this.runMotionMagic(rpm), this::stop).withName("MotionMagicTest");
  }

  public void setRPM(double rpm) {
    this.targetRPM = rpm;
    io.setVelocity(rpm * (Math.PI / 30.0));
  }

  public void stop() {
    this.targetRPM = 0;
    io.setVoltage(0.0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
