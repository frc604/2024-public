package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.quixlib.vision.Fiducial;

public class Fiducials {
  // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
  private static final double aprilTagSize = Units.inchesToMeters(6.5); // m

  public static final Fiducial[] aprilTagFiducials =
      new Fiducial[] {
        new Fiducial(
            Fiducial.Type.APRILTAG,
            1,
            new Pose3d(
                Units.inchesToMeters(593.68),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            2,
            new Pose3d(
                Units.inchesToMeters(637.21),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            3,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            4,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            5,
            new Pose3d(
                Units.inchesToMeters(578.77),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            6,
            new Pose3d(
                Units.inchesToMeters(72.50),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            7,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            8,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            9,
            new Pose3d(
                Units.inchesToMeters(14.02),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            10,
            new Pose3d(
                Units.inchesToMeters(57.54),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            11,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(300.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            12,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            13,
            new Pose3d(
                Units.inchesToMeters(441.74),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            14,
            new Pose3d(
                Units.inchesToMeters(209.48),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            15,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            16,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(240.0))),
            aprilTagSize),
      };

  public static final Pose3d[] centerNotes = {
    new Pose3d(
        Units.inchesToMeters(325.61),
        Units.inchesToMeters(293.62),
        Units.inchesToMeters(1.0),
        new Rotation3d()),
    new Pose3d(
        Units.inchesToMeters(325.61),
        Units.inchesToMeters(227.62),
        Units.inchesToMeters(1.0),
        new Rotation3d()),
    new Pose3d(
        Units.inchesToMeters(325.61),
        Units.inchesToMeters(161.62),
        Units.inchesToMeters(1.0),
        new Rotation3d()),
    new Pose3d(
        Units.inchesToMeters(325.61),
        Units.inchesToMeters(95.62),
        Units.inchesToMeters(1.0),
        new Rotation3d()),
    new Pose3d(
        Units.inchesToMeters(325.61),
        Units.inchesToMeters(29.62),
        Units.inchesToMeters(1.0),
        new Rotation3d()),
  };
}
