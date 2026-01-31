package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  
  private final PIDController pid = new PIDController(1.0, 0.0, 0.0);

  private double targetPositionRad = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    double outputVolts = pid.calculate(inputs.positionRad, targetPositionRad);

    io.setVoltage(outputVolts);
    
    Logger.recordOutput("Turret/TargetPositionRad", targetPositionRad);
  }

  public void setTargetPosition(double rad) {
    this.targetPositionRad = rad;
  }
}