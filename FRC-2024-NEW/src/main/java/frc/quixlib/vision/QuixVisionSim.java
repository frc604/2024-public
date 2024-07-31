package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class QuixVisionSim {
  private final VisionSystemSim m_visionSim = new VisionSystemSim("main");

  public QuixVisionSim(
      final ArrayList<QuixVisionCamera> cameras,
      final Fiducial[] aprilTags,
      final Pose3d[] notePoses) {
    for (var camera : cameras) {
      switch (camera.getPipelineConfig().fiducialType) {
        case APRILTAG:
          {
            for (var tag : aprilTags) {
              m_visionSim.addVisionTargets(
                  "apriltag",
                  new VisionTargetSim(tag.getPose(), TargetModel.kAprilTag36h11, tag.id()));
            }
            break;
          }
        case NOTE:
          {
            for (var notePose : notePoses) {
              m_visionSim.addVisionTargets(
                  "note",
                  new VisionTargetSim(
                      notePose,
                      new TargetModel(
                          Units.inchesToMeters(14),
                          Units.inchesToMeters(14),
                          Units.inchesToMeters(2))));
            }
            break;
          }
        default:
          break;
      }
      m_visionSim.addCamera(camera.getCameraSim(), camera.getTransform());
    }
  }

  public void resetSimPose(final Pose2d pose) {
    m_visionSim.resetRobotPose(pose);
  }

  public void updatePose(final Pose2d pose) {
    m_visionSim.update(pose);
  }

  public Field2d getSimField() {
    return m_visionSim.getDebugField();
  }
}
