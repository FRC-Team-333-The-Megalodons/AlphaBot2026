package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class SpindexerIOKraken implements SpindexerIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(SpindexerConstants.MOTOR_ID, rio);
  private final MotionMagicVelocityDutyCycle magicRequest = new MotionMagicVelocityDutyCycle(0).withSlot(0);

  public SpindexerIOKraken() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = SpindexerConstants.GEAR_RATIO;
    config.Slot0.kP = SpindexerConstants.kP;
    config.Slot0.kS = SpindexerConstants.kS;
    config.Slot0.kV = SpindexerConstants.kV;

    motor.getConfigurator().apply(config);
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
  public void setVelocity(double rps) {
    motor.setControl(magicRequest.withVelocity(Units.radiansToRotations(rps)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
