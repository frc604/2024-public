package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.quixlib.vision.Fiducial;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class NTManager {
  private final NetworkTable m_localizerTable;
  // Combined Robot to DS odometry and vision measurements
  private final DoubleArrayPublisher m_measurementsPub;
  // Robot to DS targets
  private final DoubleArrayPublisher m_targetPub;

  // Use an AtomicReference to make updating the value thread-safe
  private final AtomicReference<NTPoseEstimate> m_latestPoseEstimate =
      new AtomicReference<NTPoseEstimate>();

  private final PoseEstimateInputsAutoLogged m_inputs = new PoseEstimateInputsAutoLogged();

  @AutoLog
  public static class PoseEstimateInputs {
    protected int id = 0;
    protected Pose2d pose = new Pose2d();
    protected boolean hasVision = false;
  }

  public NTManager() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_localizerTable = inst.getTable("localizer");
    m_measurementsPub =
        m_localizerTable.getDoubleArrayTopic("measurements").publish(PubSubOption.sendAll(true));
    m_targetPub =
        m_localizerTable.getDoubleArrayTopic("targets").publish(PubSubOption.sendAll(true));

    // Setup listener for when the estimate is updated.
    final var estimatesSub =
        m_localizerTable
            .getDoubleArrayTopic("estimates")
            .subscribe(new double[] {}, PubSubOption.sendAll(true));
    inst.addListener(
        estimatesSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          m_latestPoseEstimate.set(
              new NTPoseEstimate(
                  (int) event.valueData.value.getDoubleArray()[0],
                  new Pose2d(
                      event.valueData.value.getDoubleArray()[1],
                      event.valueData.value.getDoubleArray()[2],
                      new Rotation2d(event.valueData.value.getDoubleArray()[3])),
                  event.valueData.value.getDoubleArray()[4] == 1.0 ? true : false));
        });
  }

  public void publishMeasurement(final Measurement measurement, final int id) {
    m_measurementsPub.set(measurement.toArray(id));
    NetworkTableInstance.getDefault().flush();
  }

  public void publishTargets(final Fiducial[] targets) {
    final int kDataLength = 8;
    final double[] data = new double[kDataLength * targets.length];
    for (int i = 0; i < targets.length; i++) {
      data[kDataLength * i] = targets[i].id();
      data[kDataLength * i + 1] = targets[i].getX();
      data[kDataLength * i + 2] = targets[i].getY();
      data[kDataLength * i + 3] = targets[i].getZ();
      data[kDataLength * i + 4] = targets[i].getXRot();
      data[kDataLength * i + 5] = targets[i].getYRot();
      data[kDataLength * i + 6] = targets[i].getZRot();
      data[kDataLength * i + 7] = targets[i].getSize();
    }
    m_targetPub.set(data);
  }

  public void updateInputs() {
    final var latestEstimate = m_latestPoseEstimate.get();
    if (latestEstimate != null) {
      m_inputs.id = latestEstimate.getID();
      m_inputs.pose = latestEstimate.getPose();
      m_inputs.hasVision = latestEstimate.hasVision();
    }
    Logger.processInputs("Inputs/NTManager", m_inputs);
  }

  /** Get the latest estimate over NT. */
  public NTPoseEstimate getLatestPoseEstimate() {
    return new NTPoseEstimate(m_inputs.id, m_inputs.pose, m_inputs.hasVision);
  }
}
