package frc.robot.subsystems.leds;

/**
 * Simulated implementation of LedIO for use in AdvantageScope and simulation.
 * I have no idead if we can simulate LEDS but this just tracks a state and nothing more.
 */
public class LedIOSim implements LedIO {

  private LedState currentState = LedState.IDLE;

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.currentState = currentState.name();
  }

  @Override
  public void setState(LedState state) {
    currentState = state;
  }
}