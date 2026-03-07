package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private Supplier<Distance> distanceSupplier;

  private final SysIdRoutine sysIdRoutine;


  private enum PreSpinState {
    IDLE,
   
    SPINNING,
  
    COASTING
  }

  private PreSpinState preSpinState = PreSpinState.IDLE;

  private boolean spinRequested = false;

  private final Timer coastDownTimer = new Timer();

  private static final double COAST_DOWN_SECONDS = 3.0;

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
    double target = io.getRPMFromDistance(distanceSupplier.get());
    Logger.recordOutput("Flywheel/TargetRPM", target);
    return target;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    switch (preSpinState) {
      case IDLE:
        if (spinRequested) {
          preSpinState = PreSpinState.SPINNING;
          coastDownTimer.stop();
          coastDownTimer.reset();
        }
        
        break;

      case SPINNING:
        if (spinRequested) {
         
          io.moveTo(dynamicRPM());
          coastDownTimer.stop();
          coastDownTimer.reset();
        } else {
          preSpinState = PreSpinState.COASTING;
          coastDownTimer.restart();
        }
        break;

      case COASTING:
        if (spinRequested) {
         
          preSpinState = PreSpinState.SPINNING;
          coastDownTimer.stop();
          coastDownTimer.reset();
        } else if (coastDownTimer.hasElapsed(COAST_DOWN_SECONDS)) {
          preSpinState = PreSpinState.IDLE;
          coastDownTimer.stop();
          coastDownTimer.reset();
          io.setVoltage(0.0);
        } else {
       
          io.moveTo(dynamicRPM());
        }
        break;
    }

    Logger.recordOutput("Flywheel/PreSpinState", preSpinState.toString());
    Logger.recordOutput("Flywheel/CoastDownTimer", coastDownTimer.get());
    Logger.recordOutput("Flywheel/VelocityRPM", inputs.velocityRPM);
  }

 
  private void requestPreSpin() {
    spinRequested = true;
  }

  private void stopPreSpin() {
    spinRequested = false;
  }

 

  public boolean ready() {
    return isAt(dynamicRPM());
  }

  public boolean isAt(double rpm) {
    return io.atTarget(rpm);
  }

  
  public boolean isPreSpunUp() {
    return preSpinState == PreSpinState.SPINNING || preSpinState == PreSpinState.COASTING;
  }

  
  public Command shootOnMoveSpinUp() {
    return runEnd(this::requestPreSpin, this::stopPreSpin);
  }

  public Command dynamicSpinUp(boolean waitUntilCompletion) {
    Command com =
        waitUntilCompletion
            ? run(() -> io.moveTo(dynamicRPM())).until(this::ready)
            : run(() -> io.moveTo(dynamicRPM()));

    return com.handleInterrupt(() -> io.setVoltage(0.0));
  }

  public Command spinAt(double rpm, boolean waitUntilCompletion) {
    Command com =
        waitUntilCompletion
            ? run(() -> io.moveTo(rpm)).until(() -> isAt(rpm))
            : run(() -> io.moveTo(rpm));

    return com.handleInterrupt(() -> io.setVoltage(0.0));
  }

 
  public void setRPMDirect(double rpm) {
    io.moveTo(rpm);
  }

  public void stopMotor() {
    io.setVoltage(0.0);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0.0));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
