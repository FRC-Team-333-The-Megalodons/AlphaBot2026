package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelIOKraken implements FlywheelIO {
  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, rio);
  private final TalonFX motor2 = new TalonFX(FlywheelConstants.MOTOR_2_ID, rio);

  private final MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);

  public FlywheelIOKraken() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Slot 0 Gains
    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kA = FlywheelConstants.kA;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;

    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = FlywheelConstants.MAX_JERK;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 15.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.CurrentLimits.StatorCurrentLimit = 70.0;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = 35.0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Motor 1 follows Motor 2 in the opposite direction (counter-rotation for shooting)
    motor.setControl(new Follower(motor2.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = rpsToRPM(motor2.getVelocity().getValue());
    inputs.appliedVolts = motor2.getMotorVoltage().getValueAsDouble();
    inputs.statorAmps = motor2.getStatorCurrent().getValueAsDouble();
    inputs.supplyAmps = motor2.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public boolean atTarget(double targetRPM) {
    double currentRPM = rpsToRPM(motor.getVelocity().getValue());
    return Math.abs(targetRPM) > 0
        && Math.abs(Math.abs(currentRPM) - Math.abs(targetRPM))
            < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void moveTo(double rpm) {
    motor2.setControl(mmVelocity.withVelocity(rpmToRPS(rpm)));
  }

  @Override
  public void setVoltage(double volts) {
    motor2.setControl(new VoltageOut(volts));
  }
}
