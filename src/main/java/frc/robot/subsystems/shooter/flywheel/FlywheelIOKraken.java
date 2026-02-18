package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOKraken implements FlywheelIO {
  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, rio);
  private final TalonFX motor2 = new TalonFX(FlywheelConstants.MOTOR_2_ID, rio);

  private final MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);

  public FlywheelIOKraken() {
    var config = new TalonFXConfiguration();

    // Slot 0 Gains
    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;

    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = FlywheelConstants.MAX_JERK;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Motor 2 follows Motor 1
    motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVelocity(double radPerSec) {
    double rps = Units.radiansToRotations(radPerSec);
    motor.setControl(mmVelocity.withVelocity(rps));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
