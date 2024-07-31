package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionCamera implements QuixVisionCamera {
  private final String m_loggingName;
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_sim;
  private final Transform3d m_transform;
  private final double m_fovWidth;
  private final double m_fovHeight;
  private final PipelineConfig[] m_pipelineConfigs;

  // Magic number from LL website.
  // TODO: Figure out why sim is "early" by one period.
  // TODO: Figure out why this is negative.
  private static final double kImageCaptureLatencySeconds =
      Robot.isSimulation() ? -LoggedRobot.defaultPeriodSecs : -0.02;

  private final PhotonCameraInputs m_inputs = new PhotonCameraInputs();

  public class PhotonCameraInputs implements LoggableInputs {
    // TODO: Monitor performance and consider not logging the whole PhotonPipelineResult.
    protected int pipelineIndex = 0;
    protected PhotonPipelineResult latestResult = new PhotonPipelineResult();
    protected Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
    protected Optional<Matrix<N8, N1>> distCoeffs = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      table.put("PipelineIndex", pipelineIndex);
      table.put("LatestResult", latestResult);
      table.put("CameraMatrixIsPresent", cameraMatrix.isPresent());
      if (cameraMatrix.isPresent()) {
        table.put("CameraMatrixData", cameraMatrix.get().getData());
      }
      table.put("DistCoeffsIsPresent", distCoeffs.isPresent());
      if (distCoeffs.isPresent()) {
        table.put("DistCoeffsData", distCoeffs.get().getData());
      }
    }

    @Override
    public void fromLog(LogTable table) {
      pipelineIndex = table.get("PipelineIndex", pipelineIndex);
      latestResult = table.get("LatestResult", latestResult);
      cameraMatrix =
          table.get("CameraMatrixIsPresent", false)
              ? Optional.of(
                  new Matrix<N3, N3>(
                      new SimpleMatrix(3, 3, true, table.get("CameraMatrixData", new double[9]))))
              : Optional.empty();
      distCoeffs =
          table.get("DistCoeffsIsPresent", false)
              ? Optional.of(
                  new Matrix<N8, N1>(
                      new SimpleMatrix(8, 1, true, table.get("DistCoeffsData", new double[8]))))
              : Optional.empty();
    }
  }

  public PhotonVisionCamera(
      final String cameraName,
      final Transform3d transform,
      final double fovWidth,
      final double fovHeight,
      final PipelineConfig[] pipelineConfigs) {
    m_loggingName = "Inputs/PhotonVisionCamera [" + cameraName + "]";
    m_camera = new PhotonCamera(cameraName);
    m_transform = transform;
    m_fovWidth = fovWidth;
    m_fovHeight = fovHeight;
    m_pipelineConfigs = pipelineConfigs;

    setPipelineIndex(0);

    // Setup sim
    final SimCameraProperties props = new SimCameraProperties();
    // TODO: Sim more than one pipeline.
    props.setCalibration(
        pipelineConfigs[0].imageWidth,
        pipelineConfigs[0].imageHeight,
        pipelineConfigs[0].camIntrinsics,
        pipelineConfigs[0].distCoeffs);
    props.setCalibError(0.25, 0.08);
    props.setFPS(20.0);
    props.setAvgLatencyMs(35.0);
    props.setLatencyStdDevMs(5.0);
    m_sim = new PhotonCameraSim(m_camera, props);
    m_sim.enableDrawWireframe(true);
  }

  public PhotonCameraSim getCameraSim() {
    return m_sim;
  }

  public void updateInputs() {
    m_inputs.pipelineIndex = m_camera.getPipelineIndex();
    // TODO: Handle all results, not just the latest.
    var latestResults = m_camera.getAllUnreadResults();
    m_inputs.latestResult =
        latestResults.size() > 0
            ? latestResults.get(latestResults.size() - 1)
            : new PhotonPipelineResult();
    // Only update these once, since they shouldn't be changing.
    final var cameraMatrix = m_camera.getCameraMatrix();
    if (m_inputs.cameraMatrix.isEmpty() && cameraMatrix.isPresent()) {
      m_inputs.cameraMatrix = cameraMatrix;
    }
    final var distCoeffs = m_camera.getDistCoeffs();
    if (m_inputs.distCoeffs.isEmpty() && distCoeffs.isPresent()) {
      m_inputs.distCoeffs = distCoeffs;
    }
    Logger.processInputs(m_loggingName, m_inputs);
  }

  public void setPipelineIndex(final int index) {
    if (index > m_pipelineConfigs.length) {
      System.out.println("Invalid pipeline index: " + index);
      return;
    }
    m_camera.setPipelineIndex(index);
  }

  public PipelineConfig getPipelineConfig() {
    return m_pipelineConfigs[m_inputs.pipelineIndex];
  }

  public Transform3d getTransform() {
    return m_transform;
  }

  public Fiducial.Type getFiducialType() {
    return getPipelineConfig().fiducialType;
  }

  public PipelineVisionPacket getLatestMeasurement() {
    final double startTimestampUs = RobotController.getFPGATime();
    final var result = m_inputs.latestResult;
    final boolean hasTargets = result.hasTargets();
    if (!hasTargets) {
      return new PipelineVisionPacket(false, null, null, -1);
    }

    final PipelineConfig pipelineConfig = getPipelineConfig();
    // Attempt to use the camera matrix and distortion coefficients if they are available, otherwise
    // fall back to estimate based on image size and FOV.
    final boolean hasCalibration =
        m_inputs.cameraMatrix.isPresent() && m_inputs.distCoeffs.isPresent();
    final Target bestTarget =
        hasCalibration
            ? Target.fromPhotonTrackedTargetWithCalibration(
                result.getBestTarget(), m_inputs.cameraMatrix.get(), m_inputs.distCoeffs.get())
            : Target.fromPhotonTrackedTargetWithoutCalibration(
                result.getBestTarget(),
                pipelineConfig.imageWidth,
                pipelineConfig.imageHeight,
                m_fovWidth,
                m_fovHeight);
    final ArrayList<Target> targets = new ArrayList<Target>();
    for (var t : result.getTargets()) {
      targets.add(
          hasCalibration
              ? Target.fromPhotonTrackedTargetWithCalibration(
                  t, m_inputs.cameraMatrix.get(), m_inputs.distCoeffs.get())
              : Target.fromPhotonTrackedTargetWithoutCalibration(
                  t,
                  pipelineConfig.imageWidth,
                  pipelineConfig.imageHeight,
                  m_fovWidth,
                  m_fovHeight));
    }

    final double endTimestampUs = RobotController.getFPGATime();
    Logger.recordOutput(
        m_loggingName + "/GetLatestMeasurementMs", (endTimestampUs - startTimestampUs) / 1000.0);

    return new PipelineVisionPacket(
        hasTargets,
        bestTarget,
        targets,
        result.getTimestampSeconds() - kImageCaptureLatencySeconds);
  }
}
