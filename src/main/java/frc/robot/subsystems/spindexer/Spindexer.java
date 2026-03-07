package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private enum SpinState {
    IDLE,

    FORWARD,

    REVERSING
  }

  private SpinState state = SpinState.IDLE;

  private boolean spinRequested = false;

  private final Timer jamTimer = new Timer();

  private final Timer reverseTimer = new Timer();

  private final Timer startupTimer = new Timer();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  private boolean isJammedByVelocity() {
    return inputs.velocityRps < SpindexerConstants.JAM_VELOCITY_THRESHOLD_RPS;
  }

  private boolean isJammedByCurrent() {
    return inputs.currentAmps > SpindexerConstants.JAM_CURRENT_THRESHOLD_AMPS;
  }

  private boolean jamConditionMet() {
    if (startupTimer.get() < SpindexerConstants.STARTUP_GRACE_SECONDS) return false;
    return isJammedByVelocity() || isJammedByCurrent();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double spinVoltage = decide_spin_voltage();
    io.setVoltage(spinVoltage);
    Logger.recordOutput("Spindexer/Voltage", spinVoltage);
  }

  public double decide_spin_voltage() {
    if (!spinRequested) {
      return SpindexerConstants.SPIN_VOLTAGE_STOPPED;
    }
    // For the beginning of the second, we spin fast; for the rest of the second, we spin slow.
    final long cutoff_ms = 500; // If this is 750, it means spend 750 at fast, and 250 at slow.
    final long current_ms = System.currentTimeMillis() % 1000;

    if (current_ms > cutoff_ms) {
      return SpindexerConstants.SPIN_VOLTAGE_SLOW;
    }
    return SpindexerConstants.SPIN_VOLTAGE;
  }

  public void periodic_with_jam_detection() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

    // --- Spin not requested: shut everything down and reset ---
    if (!spinRequested) {
      state = SpinState.IDLE;
      jamTimer.stop();
      jamTimer.reset();
      reverseTimer.stop();
      reverseTimer.reset();
      startupTimer.stop();
      startupTimer.reset();
      io.setVoltage(0.0);

      Logger.recordOutput("Spindexer/State", state.toString());
      Logger.recordOutput("Spindexer/JamDetected", false);
      return;
    }

    switch (state) {
      case IDLE:
        state = SpinState.FORWARD;
        startupTimer.restart();
        jamTimer.reset();
        io.setVoltage(SpindexerConstants.SPIN_VOLTAGE);
        break;

      case FORWARD:
        io.setVoltage(SpindexerConstants.SPIN_VOLTAGE);

        if (jamConditionMet()) {
          jamTimer.start();

          if (jamTimer.hasElapsed(SpindexerConstants.JAM_DETECT_SECONDS)) {
            state = SpinState.REVERSING;
            jamTimer.stop();
            jamTimer.reset();
            reverseTimer.restart();
            Logger.recordOutput("Spindexer/JamDetected", true);
          }
        } else {
          jamTimer.stop();
          jamTimer.reset();
        }
        break;

      case REVERSING:
        io.setVoltage(SpindexerConstants.REVERSE_VOLTAGE);

        if (reverseTimer.hasElapsed(SpindexerConstants.REVERSE_DURATION_SECONDS)) {
          state = SpinState.FORWARD;
          reverseTimer.stop();
          reverseTimer.reset();
          startupTimer.restart();
          Logger.recordOutput("Spindexer/JamDetected", false);
        }
        break;
    }

    Logger.recordOutput("Spindexer/State", state.toString());
    Logger.recordOutput("Spindexer/VelocityRps", inputs.velocityRps);
    Logger.recordOutput("Spindexer/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Spindexer/JamTimerSeconds", jamTimer.get());
  }

  private void requestSpin() {
    if (state == SpinState.IDLE) {
      state = SpinState.FORWARD;
      startupTimer.restart();
    }
    spinRequested = true;
  }

  private void stopSpin() {
    spinRequested = false;
  }

  public Command spin() {
    return runEnd(this::requestSpin, this::stopSpin);
  }
}
