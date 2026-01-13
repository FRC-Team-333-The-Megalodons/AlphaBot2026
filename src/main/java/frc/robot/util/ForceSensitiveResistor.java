package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;

public class ForceSensitiveResistor {

    // Adjust these two constants to your liking,
    private static final double LOGIC_LOW_VOLTAGE_THRESHOLD = 0.7;
    private static final double LOGIC_HIGH_VOLTAGE_THRESHOLD = 3.3;

    AnalogInput resistorInput;
    private double lowThreshold;
    private double highThreshold;
    private boolean loaded;
    
    public ForceSensitiveResistor(int pin) {
        this(pin, LOGIC_LOW_VOLTAGE_THRESHOLD, LOGIC_HIGH_VOLTAGE_THRESHOLD);
    }

    public ForceSensitiveResistor(int pin, double customLogicLowThreshold, double customLogicHighThreshold) {
        resistorInput = new AnalogInput(pin);
        resistorInput.setOversampleBits(4);

        lowThreshold = customLogicLowThreshold;
        highThreshold = customLogicHighThreshold;
        loaded = false;
    }

    public boolean hasLoad() {
        double input = this.getLoad();

        // Implement Schmitt Trigger & Hysteresis in code
        loaded = (loaded || input > highThreshold) && input > lowThreshold;
        return loaded;
    }

    public double getLoad() {
        return resistorInput.getValue();
    }
}
