package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOYams implements TurretIO {
  private final SmartMotorController motor;

  public TurretIOYams(Supplier<Angle> enc1, Supplier<Angle> enc2) {
    TalonFX turretMotor = new TalonFX(5);
    // 1. Configure the Motor Controller
    // Reduction is 105:17 for the main drive
    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(105.0 / 17.0))
            .withIdleMode(MotorMode.BRAKE);

    // TalonFXS (ID 5) using the YAMS remote wrapper
    motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);
    // 2. Configure EasyCRT Solver
    // Common driver is the 105T ring, pinions are 17T and 18T
    EasyCRTConfig crtConfig =
        new EasyCRTConfig(enc1, enc2)
            .withCommonDriveGear(1.0, 105, 17, 18)
            .withMechanismRange(Rotations.of(-300.0 / 360.0), Rotations.of(300.0 / 360.0))
            .withMatchTolerance(Rotations.of(0.06));

    EasyCRT solver = new EasyCRT(crtConfig);

    // 3. Initialize motor position ONCE at startup
    // This seeds the relative encoder with the absolute CRT position
    solver.getAngleOptional().ifPresent(motor::setEncoderPosition);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = motor.getRotorPosition().in(Radians);
    inputs.velocityRadPerSec = motor.getRotorVelocity().in(RadiansPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(Volts.of(volts));
  }
}
