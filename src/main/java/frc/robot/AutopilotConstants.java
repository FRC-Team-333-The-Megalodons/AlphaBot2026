package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class AutopilotConstants {
  public static final APConstraints K_CONSTRAINTS = new APConstraints(3.0, 10);
  public static final APProfile kProfile =
      new APProfile(K_CONSTRAINTS)
          .withErrorXY(Distance.ofRelativeUnits(3, Inch))
          .withErrorTheta(Angle.ofRelativeUnits(4, Degree))
          .withBeelineRadius(Distance.ofRelativeUnits(10, Inch));

  public static final Autopilot kAutopilot = new Autopilot(kProfile);

  // Slow autopilot  used for the precise final approach during climbing.
  public static final APConstraints K_CLIMBING_CONSTRAINTS = new APConstraints(0.8, 3.0);
  public static final APProfile kClimbingProfile =
      new APProfile(K_CLIMBING_CONSTRAINTS)
          .withErrorXY(Distance.ofRelativeUnits(0.5, Inch))
          .withErrorTheta(Angle.ofRelativeUnits(1, Degree))
          .withBeelineRadius(Distance.ofRelativeUnits(4, Inch));

  public static final Autopilot kClimbingAutopilot = new Autopilot(kClimbingProfile);
}
