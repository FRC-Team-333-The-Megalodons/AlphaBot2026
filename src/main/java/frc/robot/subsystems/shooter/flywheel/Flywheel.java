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

    distanceToRPM.put(2.0, -1800.0);
    distanceToRPM.put(2.65, -1950.0);
    distanceToRPM.put(3.0, -2100.0);
    distanceToRPM.put(3.78, -2300.0);
    distanceToRPM.put(4.5, -2600.0);
    distanceToRPM.put(5.5, -2700.0);
    distanceToRPM.put(7.0, -3000.0);

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
