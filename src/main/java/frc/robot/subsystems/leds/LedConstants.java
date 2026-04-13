package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;

public class LedConstants {
  public static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);
  public static final RGBWColor WHITE = new RGBWColor(0, 0, 0, 255);

  public static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
  public static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 255, 0);

  public static final RGBWColor MAGENTA = new RGBWColor(255, 0, 255, 0);
  public static final RGBWColor YELLOW = new RGBWColor(255, 255, 0, 0);
  public static final RGBWColor CYAN = new RGBWColor(0, 255, 255, 0);
}
