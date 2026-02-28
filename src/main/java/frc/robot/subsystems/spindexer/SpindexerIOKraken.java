package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.pivot.PivotConstants;

public class SpindexerIOKraken implements SpindexerIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(SpindexerConstants.MOTOR_ID, rio);

  public SpindexerIOKraken() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);

    config.Feedback.SensorToMechanismRatio = SpindexerConstants.GEAR_RATIO;
    motor.getConfigurator().apply(config);
  }

  @Override
  public boolean atTarget(double rpm) {
    boolean atVelocity = Math.abs(rpm - motor.getVelocity().getValueAsDouble() * 60.0) < SpindexerConstants.VELOCITY_TOLERANCE;
    return atVelocity;
  }

  @Override
  public void moveTo(double rpm) {
    motor.setControl(new VelocityVoltage(rpm/60.0));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // Commenting out un-needed stats for logging to improve 20ms cycle overrun issues.
    // inputs.velocityRps = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts =
        motor
            .getMotorVoltage()
            .getValueAsDouble(); // Keeping applied volts, because we really need to know when a
    // motor is powered for debugging
    // inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
