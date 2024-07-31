package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentSkipListMap;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class QuikPlanSwervePartialTrajectoryReader {
  private final List<PolynomialSplineFunction> m_interpolatedStateData = new ArrayList<>();
  private final ConcurrentSkipListMap<Double, QuikPlanAction> m_timeToActionMap =
      new ConcurrentSkipListMap<>();
  private final LinearInterpolator m_interpolator = new LinearInterpolator();
  private final File m_file;

  private boolean m_isBlueAlliance = true; // Read the file in the perspective of blue/red alliance.
  private double m_totalTime = 0.0;

  public QuikPlanSwervePartialTrajectoryReader(final String filename) {
    m_file = new File(Filesystem.getDeployDirectory().toPath() + "/partial/" + filename);
    loadSelectedFile();
  }

  /** Loads the selected file if it has changed. */
  public void loadSelectedFile() {
    final var alliance = DriverStation.getAlliance();
    final boolean isBlueAlliance = alliance.isPresent() && alliance.get() == Alliance.Blue;
    m_isBlueAlliance = isBlueAlliance;
    loadTrajectory(m_file);
  }

  private void loadTrajectory(final File file) {
    clearLoadedTrajectory();

    // Temp data.
    final ArrayList<Double> timeData = new ArrayList<>();
    final ArrayList<ArrayList<Double>> stateData = new ArrayList<ArrayList<Double>>();
    for (int i = 0; i < CSVState.values().length; i++) {
      stateData.add(new ArrayList<Double>());
    }

    // Read time and state data.
    try (final BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        final double[] data =
            Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
        timeData.add(data[0]);
        for (int i = 1; i < 7; i++) {
          stateData.get(i - 1).add(data[i]);
        }
        final int actionType = (int) data[7];
        if (actionType != 0) { // Only store non-null actions.
          m_timeToActionMap.put(
              data[0], new QuikPlanAction(actionType, (int) data[8], (int) data[9]));
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    // Save interpolated state data.
    for (final List<Double> dataList : stateData) {
      m_interpolatedStateData.add(
          m_interpolator.interpolate(
              timeData.stream().mapToDouble(d -> d).toArray(),
              dataList.stream().mapToDouble(d -> d).toArray()));
    }
    m_totalTime = timeData.get(timeData.size() - 1);
  }

  private void clearLoadedTrajectory() {
    m_totalTime = 0.0;
    m_interpolatedStateData.clear();
    m_timeToActionMap.clear();
  }

  private List<Double> getInterpolatedCSVState(final double time) {
    final List<Double> state = new ArrayList<>();
    for (PolynomialSplineFunction psf : m_interpolatedStateData) {
      state.add(psf.value(Math.min(time, getTotalTime())));
    }
    return state;
  }

  public QuikplanTrajectoryState getState(final double time) {
    final var state = getInterpolatedCSVState(time);
    // TODO: Generalize utils for different years.
    final double midline = Units.inchesToMeters(325.61);
    return new QuikplanTrajectoryState(
        new Pose2d(
            // Field is mirrored, X position is reflected over the midline
            m_isBlueAlliance
                ? state.get(CSVState.x.index)
                : 2.0 * midline - state.get(CSVState.x.index),
            // Field is mirrored, Y position stays the same
            state.get(CSVState.y.index),
            new Rotation2d(
                m_isBlueAlliance
                    ? state.get(CSVState.theta.index)
                    : Math.PI - state.get(CSVState.theta.index))),
        (m_isBlueAlliance ? 1.0 : -1.0) * state.get(CSVState.dx.index),
        // Field is mirrored, Y velocity stays the same.
        state.get(CSVState.dy.index),
        (m_isBlueAlliance ? 1.0 : -1.0) * state.get(CSVState.dTheta.index));
  }

  public Pose2d getInitialPose() {
    return getState(0.0).pose;
  }

  public double getTotalTime() {
    return m_totalTime;
  }

  /** Returns the latest action less than or equal to the given time. */
  public Entry<Double, QuikPlanAction> getAction(final double time) {
    return m_timeToActionMap.floorEntry(time);
  }

  private enum CSVState {
    // Common across years
    x(0),
    y(1),
    theta(2),
    dx(3),
    dy(4),
    dTheta(5);

    public final int index;

    private CSVState(int index) {
      this.index = index;
    }
  }

  public class QuikplanTrajectoryState {
    public final Pose2d pose;
    public final double xVel;
    public final double yVel;
    public final double thetaVel;

    public QuikplanTrajectoryState(
        final Pose2d pose, final double xVel, final double yVel, final double thetaVel) {
      this.pose = pose;
      this.xVel = xVel;
      this.yVel = yVel;
      this.thetaVel = thetaVel;
    }
  }

  public class QuikPlanAction {
    public final int actionType;
    public final int gridID;
    public final int nodeID;

    public QuikPlanAction(final int actionType, final int gridID, final int nodeID) {
      this.actionType = actionType;
      this.gridID = gridID;
      this.nodeID = nodeID;
    }
  }
}
