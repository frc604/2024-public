package frc.quixlib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.quixlib.phoenix.PhoenixIO;
import frc.quixlib.phoenix.PhoenixUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class QuixPigeon2 implements PhoenixIO, QuixIMU {
  private static final double kCANTimeoutS = 0.1; // s
  private final String m_name;
  private final String m_loggingName;
  private final Pigeon2 m_pigeon;
  private final Pigeon2SimState m_simState;
  private final QuixPigeon2Configuration m_config;

  private final QuixStatusSignal<Integer> m_faultFieldSignal;
  private final QuixStatusSignal<Integer> m_stickyFaultFieldSignal;
  private final QuixStatusSignal<Angle> m_rollSignal;
  private final QuixStatusSignal<Angle> m_pitchSignal;
  private final QuixStatusSignal<Angle> m_yawSignal;
  private final QuixStatusSignal<AngularVelocity> m_rollRateSignal;
  private final QuixStatusSignal<AngularVelocity> m_pitchRateSignal;
  private final QuixStatusSignal<AngularVelocity> m_yawRateSignal;
  private final BaseStatusSignal[] m_allSignals;

  private double continuousYawOffset = 0.0;

  private final Pigeon2InputsAutoLogged m_inputs = new Pigeon2InputsAutoLogged();

  @AutoLog
  public static class Pigeon2Inputs {
    protected StatusCode status = StatusCode.OK;
    protected int faultField = 0;
    protected int stickyFaultField = 0;
    protected double roll = 0.0;
    protected double pitch = 0.0;
    protected double yaw = 0.0;
    protected double latencyCompensatedYaw = 0.0;
    protected double rollRate = 0.0;
    protected double pitchRate = 0.0;
    protected double yawRate = 0.0;
  }

  public static class QuixPigeon2Configuration {
    private double mountPoseRoll = 0.0; // rads
    private double mountPosePitch = 0.0; // rads
    private double mountPoseYaw = 0.0; // rads
    private double gyroTrimX = 0.0; // rads
    private double gyroTrimY = 0.0; // rads
    private double gyroTrimZ = 0.0; // rads

    public QuixPigeon2Configuration setGyroTrimZ(final double rads) {
      gyroTrimZ = rads;
      return this;
    }

    public Pigeon2Configuration toPigeon2Configuration() {
      Pigeon2Configuration config = new Pigeon2Configuration();

      config.MountPose.MountPoseRoll = Math.toDegrees(mountPoseYaw);
      config.MountPose.MountPosePitch = Math.toDegrees(mountPosePitch);
      config.MountPose.MountPoseYaw = Math.toDegrees(mountPoseRoll);

      config.GyroTrim.GyroScalarX = Math.toDegrees(gyroTrimX);
      config.GyroTrim.GyroScalarY = Math.toDegrees(gyroTrimY);
      config.GyroTrim.GyroScalarZ = Math.toDegrees(gyroTrimZ);

      return config;
    }
  }

  public static QuixPigeon2Configuration makeDefaultConfig() {
    return new QuixPigeon2Configuration();
  }

  /** Default constructor */
  public QuixPigeon2(final CANDeviceID canID) {
    this(canID, makeDefaultConfig());
  }

  /** Constructor with full configuration */
  public QuixPigeon2(final CANDeviceID canID, final QuixPigeon2Configuration config) {
    m_name = "Pigeon2 " + canID.toString();
    m_loggingName = "Inputs/" + m_name;
    m_pigeon = new Pigeon2(canID.deviceNumber, canID.CANbusName);
    m_simState = m_pigeon.getSimState();
    m_config = config;

    m_faultFieldSignal = new QuixStatusSignal<>(m_pigeon.getFaultField());
    m_stickyFaultFieldSignal = new QuixStatusSignal<>(m_pigeon.getStickyFaultField());
    m_rollSignal = new QuixStatusSignal<>(m_pigeon.getRoll(), Math::toRadians);
    m_pitchSignal = new QuixStatusSignal<>(m_pigeon.getPitch(), Math::toRadians);
    m_yawSignal =
        new QuixStatusSignal<>(
            m_pigeon.getYaw(),
            (Double value) -> {
              return Math.toRadians(value) - continuousYawOffset;
            });
    m_rollRateSignal =
        new QuixStatusSignal<>(m_pigeon.getAngularVelocityXDevice(), Math::toRadians);
    m_pitchRateSignal =
        new QuixStatusSignal<>(m_pigeon.getAngularVelocityYDevice(), Math::toRadians);
    m_yawRateSignal = new QuixStatusSignal<>(m_pigeon.getAngularVelocityZDevice(), Math::toRadians);
    m_allSignals =
        QuixStatusSignal.toBaseStatusSignals(
            m_faultFieldSignal,
            m_stickyFaultFieldSignal,
            m_rollSignal,
            m_pitchSignal,
            m_yawSignal,
            m_rollRateSignal,
            m_pitchRateSignal,
            m_yawRateSignal);

    // Clear reset flag and sticky faults.
    m_pigeon.hasResetOccurred();
    m_pigeon.clearStickyFaults();

    Logger.recordOutput("Configuration/" + m_name, setConfiguration());
  }

  public boolean setConfiguration() {
    boolean allSuccess = true;

    // Set configuration.
    Pigeon2Configuration config = m_config.toPigeon2Configuration();
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pigeon.getConfigurator().apply(config, kCANTimeoutS),
            () -> {
              Pigeon2Configuration readConfig = new Pigeon2Configuration();
              m_pigeon.getConfigurator().refresh(readConfig, kCANTimeoutS);
              return PhoenixUtil.Pigeon2ConfigsEqual(config, readConfig);
            },
            m_name + ": applyConfiguration");

    // Set update frequencies.
    final double kFaultUpdateFrequency = 4.0; // Hz
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_faultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> m_faultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            m_name + ": m_faultFieldSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_stickyFaultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> m_stickyFaultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            m_name + ": m_stickyFaultFieldSignal.setUpdateFrequency()");

    final double kUpdateFrequency = 100.0; // Hz
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_rollSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_rollSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_rollSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pitchSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_pitchSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_pitchSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_yawSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_yawSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_yawSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_rollRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_rollRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_rollRateSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pitchRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_pitchRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_pitchRateSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_yawRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_yawRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_yawRateSignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    // TODO: Figure out why this sometimes causes stale signals.
    // allSuccess &=
    //     PhoenixUtil.retryUntilSuccess(
    //         () -> m_pigeon.optimizeBusUtilization(kCANTimeoutS),
    //         m_name + ": optimizeBusUtilization");

    // Block until we get valid signals.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> waitForInputs(kCANTimeoutS), m_name + ": waitForInputs()");

    // Check if unlicensed.
    allSuccess &= !m_pigeon.getStickyFault_UnlicensedFeatureInUse().getValue();

    return allSuccess;
  }

  public StatusCode updateInputs() {
    return waitForInputs(0.0);
  }

  public StatusCode waitForInputs(final double timeoutSec) {
    m_inputs.status = BaseStatusSignal.waitForAll(timeoutSec, m_allSignals);

    m_inputs.faultField = m_faultFieldSignal.getRawValue();
    m_inputs.stickyFaultField = m_stickyFaultFieldSignal.getRawValue();
    m_inputs.roll = m_rollSignal.getUnitConvertedValue();
    m_inputs.pitch = m_pitchSignal.getUnitConvertedValue();
    m_inputs.yaw = m_yawSignal.getUnitConvertedValue();
    m_inputs.latencyCompensatedYaw =
        QuixStatusSignal.getLatencyCompensatedValue(m_yawSignal, m_yawRateSignal);
    m_inputs.rollRate = m_rollRateSignal.getUnitConvertedValue();
    m_inputs.pitchRate = m_pitchRateSignal.getUnitConvertedValue();
    m_inputs.yawRate = m_yawRateSignal.getUnitConvertedValue();

    Logger.processInputs(m_loggingName, m_inputs);

    return m_inputs.status;
  }

  public void zeroContinuousYaw() {
    setContinuousYaw(0.0);
  }

  public void setContinuousYaw(final double rad) {
    continuousYawOffset += getContinuousYaw() - rad;
    updateInputs();
  }

  public double getRoll() {
    return m_inputs.roll;
  }

  public double getPitch() {
    return m_inputs.pitch;
  }

  public double getContinuousYaw() {
    return m_inputs.yaw;
  }

  public double getLatencyCompensatedContinuousYaw() {
    return m_inputs.latencyCompensatedYaw;
  }

  public double getRollRate() {
    return m_inputs.rollRate;
  }

  public double getPitchRate() {
    return m_inputs.pitchRate;
  }

  public double getYawRate() {
    return m_inputs.yawRate;
  }

  public void setSimContinuousYaw(final double rad) {
    m_simState.setRawYaw(Math.toDegrees(rad));
  }
}
