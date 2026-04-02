// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera-0";
  // This initiziliazes the camera on the left-side
  public static String camera1Name = "camera-1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(-0.2991866, 0.0761492, 0.4451096, new Rotation3d(0.0, 0.314, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.3434334, 0.0761746, 0.337566, new Rotation3d(0.0, 0.314, Math.toRadians(180)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.15;
  public static double maxZError = 0.6;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.04; // Meters
  public static double angularStdDevBaseline = 0.12; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        0.4, // Camera 1
        1.0 // Turret Camera
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = 1.0; // No rotation data available
}
