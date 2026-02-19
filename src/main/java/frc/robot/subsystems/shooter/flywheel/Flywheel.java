package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double targetRPM = 0;
  private DoubleSupplier distanceSupplier;

  private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  private final SysIdRoutine sysIdRoutine;

  public Flywheel(FlywheelIO io, DoubleSupplier distanceSupplier) {
    this.io = io;
    distanceToRPM.put(1.57, -2100.0);
    distanceToRPM.put(1.7, -2180.0);
    distanceToRPM.put(1.9, -2220.0);
    distanceToRPM.put(2.1, -2250.0);
    distanceToRPM.put(2.3, -2280.0);
    distanceToRPM.put(2.67, -2300.0);
    distanceToRPM.put(2.82, -2350.0);
    distanceToRPM.put(3.15, -2390.0);
    distanceToRPM.put(3.5, -2530.0);
    distanceToRPM.put(3.7, -2610.0);
    distanceToRPM.put(4.0, -2660.0);

    distanceToRPM.put(4.2, -2850.0);

    distanceToRPM.put(4.4, -3050.0);
    // distanceToRPM.put(4.6, -2340.0);

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

  public double getRPMForDistance() {
    return distanceToRPM.get(distanceSupplier.getAsDouble());
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
  public double calculateRPM(Drive drive){
    return targetRPM = this.getRPMForDistance();
  }

  public Command dynamicSpinUp(boolean waitUntilCompletion) {
    Command com = runEnd(() -> this.setRPM(this.getRPMForDistance()), this::stop);

    return waitUntilCompletion ? com.until(() -> this.isAtSpeed()) : com;
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
