package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;


/**
 * Simulated implementation of LedIO for use in AdvantageScope and simulation. I have no idead if we
 * can simulate LEDS but this just tracks a state and nothing more.
 */
public class LedIOSim implements LedIO {

  private LedState currentState = LedState.IDLE;

  
  private final ArrayList<BooleanSupplier> cameraSeesTagSuppliers = new ArrayList<>();

  public LedIOSim(BooleanSupplier... cameraTagSuppliers) {
    for (BooleanSupplier supplier : cameraTagSuppliers) {
      cameraSeesTagSuppliers.add(supplier);
    }
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.currentState = currentState.name();
  }

  @Override
  public void setState(LedState state) {
    currentState = state;
  }

  @Override
  public boolean anyCameraSeesTag()
  {
    for (BooleanSupplier supplier : cameraSeesTagSuppliers) {
      if (supplier.getAsBoolean()) {
        return true;
      }
    }
    return false;
  }
}
