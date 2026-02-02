package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldLayout;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements FieldLayout {
  private final NetworkTableInstance inst;
  private DoubleArraySubscriber poseSub;
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Vision vision;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  public Turret(TurretIO io, Vision vision) {
    inst = NetworkTableInstance.getDefault();
    poseSub = inst.getTable("Drive").getDoubleArrayTopic("Pose").subscribe(new double[] {0, 0, 0});
    this.io = io;
    this.vision = vision;
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

