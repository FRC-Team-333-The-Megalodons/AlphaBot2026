package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class LedIOCANdle implements LedIO {

  private static final int CANDLE_ID = 46;

  private static final int STRIP_LENGTH = 32;
  private static final int CANDLE_LENGTH = 8;
  private static final int TOTAL_LENGTH = STRIP_LENGTH + CANDLE_LENGTH;

  private final CANdle candle;

  private final ArrayList<BooleanSupplier> cameraSeesTagSuppliers = new ArrayList<>();

  private LedState lastState = null;

  public LedIOCANdle(BooleanSupplier... cameraTagSuppliers) {
    CANBus rio = CANBus.roboRIO();
    candle = new CANdle(CANDLE_ID, rio);

    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }

    for (BooleanSupplier supplier : cameraTagSuppliers) {
      cameraSeesTagSuppliers.add(supplier);
    }
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    // LEDs are only output, so no inputs need to recorded or updated.
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

  @Override
  public void setState(LedState state) {
    if (state == lastState) return;
    lastState = state;

    switch (state) {
      case IDLE:
        // All LEDs off
        candle.setControl(new SolidColor(0, TOTAL_LENGTH).withColor(LedConstants.OFF));
        break;

      case INTAKING:
        // Slow yellow Larson scanner
        candle.setControl(
            new LarsonAnimation(0, TOTAL_LENGTH)
                .withColor(new RGBWColor(255, 150, 0, 0))
                .withFrameRate(20)
                .withSize(6));
        break;

      case HAS_PIECE:
        // Solid green
        candle.setControl(new SolidColor(0, TOTAL_LENGTH).withColor(new RGBWColor(0, 255, 0, 0)));
        break;

      case SPINNING_UP:
        // Slow blue Larson scanner
        candle.setControl(
            new LarsonAnimation(0, TOTAL_LENGTH)
                .withColor(new RGBWColor(32, 40, 255, 0))
                .withFrameRate(20)
                .withSize(6));
        break;

      case READY_TO_FIRE:
        // Fast blue Larson
        candle.setControl(
            new LarsonAnimation(0, TOTAL_LENGTH)
                .withColor(new RGBWColor(32, 40, 255, 0))
                .withFrameRate(60)
                .withSize(3));
        break;

      case NEUTRAL_ZONE:
        // Yellow Larson
        candle.setControl(
            new LarsonAnimation(0, TOTAL_LENGTH)
                .withColor(new RGBWColor(255, 255, 0, 0))
                .withFrameRate(35)
                .withSize(6));
        break;

      case DISABLED:
        // Rainbow
        candle.setControl(new RainbowAnimation(TOTAL_LENGTH, 0).withFrameRate(6));
        break;

      case INTAKE_IS_AT_SPEED:
        candle.setControl(new SolidColor(TOTAL_LENGTH, 0).withColor(LedConstants.YELLOW));
        break;

      case SHOOTER_HAS_TAG:
        candle.setControl(new SolidColor(TOTAL_LENGTH, 0).withColor(LedConstants.GREEN));
        break;

        // Climber is UP
      case CLIMBER_IS_UP:
        candle.setControl(new SolidColor(TOTAL_LENGTH, 0).withColor(LedConstants.MAGENTA));
        break;
    }
  }
}
