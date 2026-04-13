package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tracker.RobotStateTracker;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {

  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  private LedState currentState = LedState.IDLE;

  public Led(LedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

  private void setState(LedState state) {
    currentState = state;
    inputs.currentState = state.name();
    io.setState(state);
  }

  public Command gameStateAwareLeds(RobotStateTracker tracker) {
    return run(() -> {
          if (tracker.isDisabled()) {
            setState(LedState.DISABLED);
            return;
          }
          /*
          if (tracker.isShooterReady()) {
            setState(LedState.READY_TO_FIRE);
            return;
          }
          if (tracker.isShooterSpinningUp()) {
            setState(LedState.SPINNING_UP);
            return;
          }
          if (tracker.isIntaking()) {
            setState(LedState.INTAKING);
            return;
          }
          */
          if (tracker.isShooting()) {
            if (io.anyCameraSeesTag()) {
              setState(LedState.SHOOTER_HAS_TAG);
            } else {
              setState(LedState.ERROR);
            }
            return;
          }

          if (tracker.isIntakeStuck()) {
            setState(LedState.INTAKE_IS_STUCK);
            return;
          }

          if (tracker.isHighEnoughToHitTunnel()) {
            setState(LedState.CLIMBER_IS_UP);
            return;
          }

          setState(LedState.IDLE);
        })
        .ignoringDisable(true)
        .withName("Led.gameStateAwareLeds");
  }

  public Command forceState(LedState state) {
    return runEnd(() -> setState(state), () -> setState(LedState.IDLE))
        .withName("Led.forceState(" + state.name() + ")");
  }

  public Command idle() {
    return forceState(LedState.IDLE);
  }

  public Command haspiece() {
    return forceState(LedState.HAS_PIECE);
  }

  public Command neutralZone() {
    return forceState(LedState.NEUTRAL_ZONE);
  }
}
