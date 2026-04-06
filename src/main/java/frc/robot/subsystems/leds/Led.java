package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tracker.RobotStateTracker;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {

  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  private final BooleanSupplier camera0SeesTagSupplier;
  private final BooleanSupplier camera1SeesTagSupplier;

  private LedState currentState = LedState.IDLE;

  public Led(LedIO io, BooleanSupplier camera0SeesTag, BooleanSupplier camera1SeesTag) {
    this.io = io;
    this.camera0SeesTagSupplier = camera0SeesTag;
    this.camera1SeesTagSupplier = camera1SeesTag;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);

    // Vision indicators always update, independent of game state
    boolean cam0 = camera0SeesTagSupplier.getAsBoolean();
    boolean cam1 = camera1SeesTagSupplier.getAsBoolean();
    io.setVisionState(cam0, cam1);
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
