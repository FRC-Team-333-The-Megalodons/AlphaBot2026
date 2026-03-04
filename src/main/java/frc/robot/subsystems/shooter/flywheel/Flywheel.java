package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private Supplier<Distance> distanceSupplier;
  
  private final SysIdRoutine sysIdRoutine;

  public Flywheel(FlywheelIO io, Supplier<Distance> distanceSupplier) {
    this.io = io;
    this.distanceSupplier = distanceSupplier;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null, null),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> io.setVoltage(volts.in(Volts)),
                (log) -> {
                  log.motor("flywheel-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(io.rpmToRPS(inputs.velocityRPM));
                },
                this));
  }

  private double dynamicRPM() {
    inputs.targetRPM = io.getRPMFromDistance(distanceSupplier.get());
    return inputs.targetRPM;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double currentRPM = inputs.velocityRPM;
  }

  public boolean ready() {
    return isAt(dynamicRPM());
  }

  public boolean isAt(double rpm) {
    return io.atTarget(rpm);
  }

  public Command dynamicSpinUp(boolean waitUntilCompletion) {

    Command com = waitUntilCompletion ?
      run(() -> io.moveTo(dynamicRPM())).until(this::ready) :
      runOnce(() -> io.moveTo(dynamicRPM()));

    return com.handleInterrupt(() -> io.setVoltage(0.0));
  }

  public Command spinAt(double rpm, boolean waitUntilCompletion) {
    inputs.targetRPM = rpm;

    Command com = waitUntilCompletion ?
      run(() -> io.moveTo(rpm)).until(() -> isAt(rpm)) :
      runOnce(() -> io.moveTo(rpm));

    return com.handleInterrupt(() -> io.setVoltage(0.0));
  }

  public Command stop() {
    return runOnce(() -> {
      io.setVoltage(0.0);
    });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
