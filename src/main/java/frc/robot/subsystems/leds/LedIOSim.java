package frc.robot.subsystems.leds;

public class LedIOSim implements LedIO {

  private LedState currentState = LedState.IDLE;
  private boolean camera0SeesTag = false;
  private boolean camera1SeesTag = false;

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.currentState = currentState.name();
    inputs.camera0SeesTag = camera0SeesTag;
    inputs.camera1SeesTag = camera1SeesTag;
  }

  @Override
  public void setState(LedState state) {
    currentState = state;
  }

  @Override
  public void setVisionState(boolean camera0SeesTag, boolean camera1SeesTag) {
    this.camera0SeesTag = camera0SeesTag;
    this.camera1SeesTag = camera1SeesTag;
  }
}
