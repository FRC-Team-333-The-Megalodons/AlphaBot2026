package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.EnergyLimitMode;

public class FlywheelIOKraken implements FlywheelIO {

  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, rio);
  private final TalonFX motor2 = new TalonFX(FlywheelConstants.MOTOR_2_ID, rio);

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Temperature> leftMotorSignal;
  private final StatusSignal<Temperature> rightMotorSignal;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);
  // private final BangBangController bangBang = new BangBangController();

  private EnergyLimitMode lastMode = EnergyLimitMode.UNSET;
  private TalonFXConfiguration config;

  public FlywheelIOKraken() {
    config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kA = FlywheelConstants.kA;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;

    config.MotionMagic.MotionMagicCruiseVelocity = FlywheelConstants.MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = FlywheelConstants.MAX_JERK;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    // This is critical, we don't actually set the config without this call!
    // If you want to "remove" this, just instead comment this, and uncomment the next line.
    applyEnergyLimits(EnergyLimitMode.DEFAULT);
    // applyEnergyLimits(EnergyLimitMode.UNLIMITED);

    motor.setControl(new Follower(motor2.getDeviceID(), MotorAlignmentValue.Opposed));

    velocitySignal = motor2.getVelocity();
    voltageSignal = motor2.getMotorVoltage();
    statorCurrentSignal = motor2.getStatorCurrent();
    supplyCurrentSignal = motor2.getSupplyCurrent();
    leftMotorSignal = motor2.getDeviceTemp();
    rightMotorSignal = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal, leftMotorSignal, rightMotorSignal);

    ParentDevice.optimizeBusUtilizationForAll(motor, motor2);
  }

  @Override
  public void applyEnergyLimits(EnergyLimitMode mode) {
    if (mode == lastMode) {
      return;
    }

    switch (mode) {
      case DEFAULT:
        {
          config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.HIGH_SUPPLY_LIMIT;
          config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.LOW_SUPPLY_LIMIT;
          config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.DROP_TO_LOW_SUPPLY_TIME_s;
          config.CurrentLimits.SupplyCurrentLimitEnable = true;
          Commands.print(
              "Applying Flywheel Energy Mode "
                  + mode
                  + " with SupplyLim="
                  + config.CurrentLimits.SupplyCurrentLimit
                  + ", SupplyLowerLim="
                  + config.CurrentLimits.SupplyCurrentLowerLimit
                  + ", SupplyDropToLowerTime="
                  + config.CurrentLimits.SupplyCurrentLowerTime);
          break;
        }
      case UNLIMITED:
        {
          config.CurrentLimits.SupplyCurrentLimitEnable = false;
          Commands.print("Applying Flywheel Energy Mode " + mode);
          break;
        }
      default:
        {
          Commands.print("Unexpected Flywheel Energy Mode = " + mode);
          break;
        }
    }

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal, leftMotorSignal, rightMotorSignal);

    inputs.velocityRPM = velocitySignal.getValueAsDouble() * 60.0;
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.statorAmps = statorCurrentSignal.getValueAsDouble();
    inputs.supplyAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.rightMotorTempCelsius = rightMotorSignal.getValueAsDouble();
    inputs.leftMotorTempCelsius = leftMotorSignal.getValueAsDouble();
  }

  @Override
  public boolean atTarget(double targetRPM) {
    double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    return Math.abs(targetRPM) > 0
        && Math.abs(Math.abs(currentRPM) - Math.abs(targetRPM))
            < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void moveTo(double rpm) {
    // double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    // double error = rpm - currentRPM;

    // double bangBangOutput = bangBang.calculate(currentRPM, rpm);

    // double rps = rpm / 60.0;
    // double ffVolts = FlywheelConstants.kS_FF * Math.signum(rpm) + FlywheelConstants.kV_FF * rps;
    // double pTrimVolts = FlywheelConstants.kP_TRIM * error;

    // // Total output:
    // // Below setpoint: 12V + FF + P_trim (Phoenix caps at battery voltage)
    // // Above setpoint: FF + P_trim (P_trim is negative here, reducing below FF)
    // double outputVolts = bangBangOutput * 12.0 + ffVolts + pTrimVolts;

    // motor2.setControl(voltageRequest.withOutput(outputVolts));
    motor2.setControl(mmVelocity.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor2.setControl(voltageRequest.withOutput(volts));
  }
}
