package frc.robot.subsystems.leds;

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

  // Vision indicator sizing
  private static final int VISION_SECTION_SIZE = 6;
  private static final int VISION_LED_COUNT = VISION_SECTION_SIZE * 2; // 12

  // Game state animations only touch LEDs 0 through GAME_STATE_LENGTH-1
  private static final int GAME_STATE_LENGTH = TOTAL_LENGTH - VISION_LED_COUNT; // 28

  // Vision LED ranges (indices into the full 40-LED chain)
  private static final int CAMERA1_START = GAME_STATE_LENGTH; // 28 — back camera
  private static final int CAMERA0_START =
      GAME_STATE_LENGTH + VISION_SECTION_SIZE; // 34 — front camera

  // Only the game-state animation uses a slot (Larson/Rainbow support .withSlot())
  private static final int GAME_STATE_SLOT = 0;

  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);

  private final CANdle candle;

  private LedState lastState = null;
  private boolean lastCamera0 = false;
  private boolean lastCamera1 = false;
  private boolean forceVisionRefresh = true;

  public LedIOCANdle() {
    CANBus rio = CANBus.roboRIO();
    candle = new CANdle(CANDLE_ID, rio);

    // Clear all animation slots
    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }

    // Initialize vision LEDs to red (no tags seen at boot)
    candle.setControl(new SolidColor(CAMERA1_START, VISION_SECTION_SIZE).withColor(RED));
    candle.setControl(new SolidColor(CAMERA0_START, VISION_SECTION_SIZE).withColor(RED));
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    // LEDs are output-only, no inputs to record.
  }

  @Override
  public void setState(LedState state) {
    if (state == lastState) return;
    lastState = state;

    // After any game state change, vision LEDs need re-applying
    // in case the control switch momentarily reset them.
    forceVisionRefresh = true;

    switch (state) {
      case IDLE:
        // SolidColor has no .withSlot(), so clear the game-state animation slot first
        // then set the base color for the game-state LEDs.
        candle.setControl(new EmptyAnimation(GAME_STATE_SLOT));
        candle.setControl(
            new SolidColor(0, GAME_STATE_LENGTH).withColor(new RGBWColor(0, 0, 0, 0)));
        break;

      case INTAKING:
        candle.setControl(
            new LarsonAnimation(0, GAME_STATE_LENGTH)
                .withColor(new RGBWColor(255, 150, 0, 0))
                .withFrameRate(20)
                .withSize(6)
                .withSlot(GAME_STATE_SLOT));
        break;

      case HAS_PIECE:
        candle.setControl(new EmptyAnimation(GAME_STATE_SLOT));
        candle.setControl(
            new SolidColor(0, GAME_STATE_LENGTH).withColor(new RGBWColor(0, 255, 0, 0)));
        break;

      case SPINNING_UP:
        candle.setControl(
            new LarsonAnimation(0, GAME_STATE_LENGTH)
                .withColor(new RGBWColor(32, 40, 255, 0))
                .withFrameRate(20)
                .withSize(6)
                .withSlot(GAME_STATE_SLOT));
        break;

      case READY_TO_FIRE:
        candle.setControl(
            new LarsonAnimation(0, GAME_STATE_LENGTH)
                .withColor(new RGBWColor(32, 40, 255, 0))
                .withFrameRate(60)
                .withSize(3)
                .withSlot(GAME_STATE_SLOT));
        break;

      case NEUTRAL_ZONE:
        candle.setControl(
            new LarsonAnimation(0, GAME_STATE_LENGTH)
                .withColor(new RGBWColor(255, 255, 0, 0))
                .withFrameRate(35)
                .withSize(6)
                .withSlot(GAME_STATE_SLOT));
        break;

      case DISABLED:
        candle.setControl(
            new RainbowAnimation(GAME_STATE_LENGTH, 0).withFrameRate(6).withSlot(GAME_STATE_SLOT));
        break;
    }
  }

  @Override
  public void setVisionState(boolean camera0SeesTag, boolean camera1SeesTag) {
    /*
    boolean changed =
        camera0SeesTag != lastCamera0 || camera1SeesTag != lastCamera1 || forceVisionRefresh;

    if (!changed) return;
    */

    // SolidColor is a direct LED write — no slot needed.
    // The slotted game-state animation only touches LEDs 0–27,
    // so these writes to LEDs 28–39 persist undisturbed.

    // We only need to change something if it actually changed.
    if (camera1SeesTag != lastCamera1) {
      candle.setControl(
          new SolidColor(CAMERA1_START, VISION_SECTION_SIZE).withColor(camera1SeesTag ? GREEN : RED));    
      lastCamera1 = camera1SeesTag;
    }
    if (camera0SeesTag != lastCamera0) {
      lastCamera0 = camera0SeesTag;
      candle.setControl(
          new SolidColor(CAMERA0_START, VISION_SECTION_SIZE).withColor(camera0SeesTag ? GREEN : RED));
    }

    forceVisionRefresh = false;
  }
}
