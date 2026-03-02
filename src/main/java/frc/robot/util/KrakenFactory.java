package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KrakenFactory {
  public static TalonFX creatKraken(int id, String bus, boolean invert, NeutralModeValue mode) {
    TalonFX motor = new TalonFX(id, bus);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = mode;
    motor.getConfigurator().apply(config, 0.050);
    return motor;
  }
}
