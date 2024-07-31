package frc.quixlib.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixStatusSignal;
import frc.quixlib.phoenix.PhoenixIO;
import frc.quixlib.phoenix.PhoenixUtil;
import frc.robot.Robot;
import java.util.function.Function;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class QuixTalonFX implements PhoenixIO, QuixMotorControllerWithEncoder, AutoCloseable {
  private static final double kCANTimeoutS = 0.1; // s
  private final String m_name;
  private final String m_loggingName;
  private final TalonFX m_controller;
  private final TalonFXSimState m_simState;
  private final MechanismRatio m_ratio;
  private final QuixTalonFXConfiguration m_config;

  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
  private final VoltageOut m_voltageControl = new VoltageOut(0);
  private final TorqueCurrentFOC m_currentControl = new TorqueCurrentFOC(0);
  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0);
  private final PositionVoltage m_positionControl = new PositionVoltage(0);
  private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage m_dynamicMotionMagicControl =
      new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private final QuixStatusSignal<Integer> m_faultFieldSignal;
  private final QuixStatusSignal<Integer> m_stickyFaultFieldSignal;
  private final QuixStatusSignal<Double> m_percentOutputSignal;
  private final QuixStatusSignal<Current> m_supplyCurrentSignal;
  private final QuixStatusSignal<Current> m_statorCurrentSignal;
  private final QuixStatusSignal<Current> m_torqueCurrentSignal;
  private final QuixStatusSignal<Angle> m_rotorPositionSignal;
  private final QuixStatusSignal<Angle> m_sensorPositionSignal;
  private final QuixStatusSignal<AngularVelocity> m_sensorVelocitySignal;
  private final QuixStatusSignal<Double> m_closedLoopReferenceSignal;
  private final QuixStatusSignal<Double> m_closedLoopReferenceSlopeSignal;
  private final BaseStatusSignal[] m_allSignals;

  private final TalonFXInputsAutoLogged m_inputs = new TalonFXInputsAutoLogged();

  @AutoLog
  public static class TalonFXInputs {
    protected StatusCode status = StatusCode.OK;
    protected int faultField = 0;
    protected int stickyFaultField = 0;
    protected double percentOutput = 0.0;
    protected double supplyCurrent = 0.0;
    protected double statorCurrent = 0.0;
    protected double torqueCurrent = 0.0;
    protected double closedLoopReference = 0.0;
    protected double closedLoopReferenceSlope = 0.0;
    protected double rotorPosition = 0.0;
    protected double sensorPosition = 0.0;
    protected double latencyCompensatedSensorPosition = 0.0;
    protected double sensorVelocity = 0.0;
  }

  public static class QuixTalonFXConfiguration {
    private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    private boolean INVERTED = false;
    private double SUPPLY_CURRENT_LIMIT = 40.0; // A
    private double STATOR_CURRENT_LIMIT = 40.0; // A
    private boolean FWD_SOFT_LIMIT_ENABLED = false;
    private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private boolean REV_SOFT_LIMIT_ENABLED = false;
    private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private PIDConfig slot0Config = new PIDConfig();
    private PIDConfig slot1Config = new PIDConfig();
    private PIDConfig slot2Config = new PIDConfig();
    private double motionMagicCruiseVelocity = 0.0; // In MechanismRatio units
    private double motionMagicAcceleration = 0.0; // In MechanismRatio units
    private double motionMagicJerk = 0.0; // In MechanismRatio units
    private double bootPositionOffset = 0.0; // In MechanismRatio units

    public QuixTalonFXConfiguration setBrakeMode() {
      NEUTRAL_MODE = NeutralModeValue.Brake;
      return this;
    }

    public QuixTalonFXConfiguration setInverted(final boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public QuixTalonFXConfiguration setStatorCurrentLimit(final double amps) {
      STATOR_CURRENT_LIMIT = amps;
      return this;
    }

    public QuixTalonFXConfiguration setSupplyCurrentLimit(final double amps) {
      SUPPLY_CURRENT_LIMIT = amps;
      return this;
    }

    public QuixTalonFXConfiguration setForwardSoftLimit(final double pos) {
      FWD_SOFT_LIMIT_ENABLED = true;
      FWD_SOFT_LIMIT = pos;
      return this;
    }

    public QuixTalonFXConfiguration setReverseSoftLimit(final double pos) {
      REV_SOFT_LIMIT_ENABLED = true;
      REV_SOFT_LIMIT = pos;
      return this;
    }

    public QuixTalonFXConfiguration setPIDConfig(final int slot, final PIDConfig config) {
      switch (slot) {
        case 0:
          slot0Config = config;
          break;
        case 1:
          slot1Config = config;
          break;
        case 2:
          slot2Config = config;
          break;
        default:
          throw new RuntimeException("Invalid PID slot " + slot);
      }
      return this;
    }

    public QuixTalonFXConfiguration setMotionMagicConfig(
        final double cruiseVelocity, final double acceleration, final double jerk) {
      motionMagicCruiseVelocity = cruiseVelocity;
      motionMagicAcceleration = acceleration;
      motionMagicJerk = jerk;
      return this;
    }

    public QuixTalonFXConfiguration setBootPositionOffset(final double pos) {
      bootPositionOffset = pos;
      return this;
    }

    public TalonFXConfiguration toTalonFXConfiguration(
        final Function<Double, Double> toNativeSensorPosition,
        final Function<Double, Double> toNativeSensorVelocity) {
      final TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NEUTRAL_MODE;
      config.MotorOutput.Inverted =
          INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.DutyCycleNeutralDeadband = 0.0;

      if (Robot.isReal()) {
        // TODO: Figure out why stator current limits break simulation.
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
      }
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1; // s

      config.TorqueCurrent.PeakForwardTorqueCurrent = STATOR_CURRENT_LIMIT;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -STATOR_CURRENT_LIMIT;
      config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          toNativeSensorPosition.apply(REV_SOFT_LIMIT);

      config.Voltage.SupplyVoltageTimeConstant = 0.0;
      config.Voltage.PeakForwardVoltage = 16.0;
      config.Voltage.PeakReverseVoltage = -16.0;

      config.Slot0 = slot0Config.fillCTRE(new Slot0Configs());
      config.Slot1 = slot1Config.fillCTRE(new Slot1Configs());
      config.Slot2 = slot2Config.fillCTRE(new Slot2Configs());

      config.MotionMagic.MotionMagicCruiseVelocity =
          toNativeSensorVelocity.apply(motionMagicCruiseVelocity);
      config.MotionMagic.MotionMagicAcceleration =
          toNativeSensorVelocity.apply(motionMagicAcceleration);
      config.MotionMagic.MotionMagicJerk = toNativeSensorVelocity.apply(motionMagicJerk);

      return config;
    }
  }

  public static QuixTalonFXConfiguration makeDefaultConfig() {
    return new QuixTalonFXConfiguration();
  }

  /** Follower constructor */
  public QuixTalonFX(
      final CANDeviceID canID,
      final QuixTalonFX leader,
      final boolean opposeLeader,
      final QuixTalonFXConfiguration config) {
    this(canID, leader.getMechanismRatio(), config);
    m_controller.setControl(new Follower(leader.getDeviceID(), opposeLeader));
  }

  /** Constructor with full configuration */
  public QuixTalonFX(
      final CANDeviceID canID, final MechanismRatio ratio, final QuixTalonFXConfiguration config) {
    m_name = "TalonFX " + canID.toString();
    m_loggingName = "Inputs/" + m_name;
    m_controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
    m_simState = m_controller.getSimState();
    m_ratio = ratio;
    m_config = config;

    m_faultFieldSignal = new QuixStatusSignal<>(m_controller.getFaultField());
    m_stickyFaultFieldSignal = new QuixStatusSignal<>(m_controller.getStickyFaultField());
    m_percentOutputSignal = new QuixStatusSignal<>(m_controller.getDutyCycle());
    m_supplyCurrentSignal = new QuixStatusSignal<>(m_controller.getSupplyCurrent());
    m_statorCurrentSignal = new QuixStatusSignal<>(m_controller.getStatorCurrent());
    m_torqueCurrentSignal = new QuixStatusSignal<>(m_controller.getTorqueCurrent());
    m_rotorPositionSignal = new QuixStatusSignal<>(m_controller.getRotorPosition());
    m_sensorPositionSignal =
        new QuixStatusSignal<>(m_controller.getRotorPosition(), this::fromNativeSensorPosition);
    m_sensorVelocitySignal =
        new QuixStatusSignal<>(m_controller.getRotorVelocity(), this::fromNativeSensorVelocity);
    m_closedLoopReferenceSignal =
        new QuixStatusSignal<>(
            m_controller.getClosedLoopReference(), this::fromNativeSensorPosition);
    m_closedLoopReferenceSlopeSignal =
        new QuixStatusSignal<>(
            m_controller.getClosedLoopReferenceSlope(), this::fromNativeSensorVelocity);
    m_allSignals =
        QuixStatusSignal.toBaseStatusSignals(
            m_faultFieldSignal,
            m_stickyFaultFieldSignal,
            m_percentOutputSignal,
            m_supplyCurrentSignal,
            m_statorCurrentSignal,
            m_torqueCurrentSignal,
            m_rotorPositionSignal,
            m_sensorPositionSignal,
            m_sensorVelocitySignal,
            m_closedLoopReferenceSignal,
            m_closedLoopReferenceSlopeSignal);

    // Clear reset flag and sticky faults.
    m_controller.hasResetOccurred();
    m_controller.clearStickyFaults();

    Logger.recordOutput("Configuration/" + m_name, setConfiguration());
  }

  public boolean setConfiguration() {
    boolean allSuccess = true;

    // Set motor controller configuration.
    final TalonFXConfiguration config =
        m_config.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_controller.getConfigurator().apply(config, kCANTimeoutS),
            () -> {
              TalonFXConfiguration readConfig = new TalonFXConfiguration();
              m_controller.getConfigurator().refresh(readConfig, kCANTimeoutS);
              return PhoenixUtil.TalonFXConfigsEqual(config, readConfig);
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
            () -> m_percentOutputSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_percentOutputSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_percentOutputSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_supplyCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_supplyCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_supplyCurrentSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_statorCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_statorCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_statorCurrentSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_torqueCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_torqueCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_torqueCurrentSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_rotorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_rotorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_rotorPositionSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_sensorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_sensorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_sensorPositionSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_sensorVelocitySignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_sensorVelocitySignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_sensorVelocitySignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_closedLoopReferenceSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_closedLoopReferenceSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_closedLoopReferenceSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () ->
                m_closedLoopReferenceSlopeSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_closedLoopReferenceSlopeSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_closedLoopReferenceSlopeSignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_controller.optimizeBusUtilization(0.0, kCANTimeoutS),
            m_name + ": optimizeBusUtilization");

    // Block until we get valid signals.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> waitForInputs(kCANTimeoutS), m_name + ": waitForInputs()");

    // Check if unlicensed.
    allSuccess &= !m_controller.getStickyFault_UnlicensedFeatureInUse().getValue();

    return allSuccess;
  }

  public boolean checkFaultsAndReconfigureIfNecessary() {
    // TODO: Log other faults.
    if (m_controller.hasResetOccurred()) {
      DriverStation.reportError(m_name + ": reset occured", false);
      setConfiguration();
      return true;
    }
    return false;
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceID();
  }

  public StatusCode updateInputs() {
    return waitForInputs(0.0);
  }

  public StatusCode waitForInputs(final double timeoutSec) {
    m_inputs.status = BaseStatusSignal.waitForAll(timeoutSec, m_allSignals);
    // TODO: Figure out why these signals aren't refreshing in waitForAll().
    m_closedLoopReferenceSignal.refresh();
    m_closedLoopReferenceSlopeSignal.refresh();

    m_inputs.faultField = m_faultFieldSignal.getRawValue();
    m_inputs.stickyFaultField = m_stickyFaultFieldSignal.getRawValue();
    m_inputs.percentOutput = m_percentOutputSignal.getUnitConvertedValue();
    m_inputs.supplyCurrent = m_supplyCurrentSignal.getUnitConvertedValue();
    m_inputs.statorCurrent = m_statorCurrentSignal.getUnitConvertedValue();
    m_inputs.torqueCurrent = m_torqueCurrentSignal.getUnitConvertedValue();
    m_inputs.closedLoopReference = m_closedLoopReferenceSignal.getUnitConvertedValue();
    m_inputs.closedLoopReferenceSlope = m_closedLoopReferenceSlopeSignal.getUnitConvertedValue();
    m_inputs.rotorPosition = m_rotorPositionSignal.getUnitConvertedValue();
    m_inputs.sensorPosition = m_sensorPositionSignal.getUnitConvertedValue();
    m_inputs.latencyCompensatedSensorPosition =
        QuixStatusSignal.getLatencyCompensatedValue(m_sensorPositionSignal, m_sensorVelocitySignal);
    m_inputs.sensorVelocity = m_sensorVelocitySignal.getUnitConvertedValue();

    Logger.processInputs(m_loggingName, m_inputs);

    return m_inputs.status;
  }

  public void setBrakeMode(final boolean on) {
    m_config.NEUTRAL_MODE = on ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_controller
        .getConfigurator()
        .apply(
            m_config.toTalonFXConfiguration(
                    this::toNativeSensorPosition, this::toNativeSensorVelocity)
                .MotorOutput);
  }

  public void setStatorCurrentLimit(final double amps) {
    m_config.STATOR_CURRENT_LIMIT = amps;

    // TODO: Consider a shorter non-blocking timeout
    m_controller
        .getConfigurator()
        .apply(
            m_config.toTalonFXConfiguration(
                    this::toNativeSensorPosition, this::toNativeSensorVelocity)
                .CurrentLimits,
            kCANTimeoutS);
  }

  public void setPercentOutput(final double percent) {
    m_dutyCycleControl.Output = percent;
    m_controller.setControl(m_dutyCycleControl);
  }

  public void setVoltageOutput(final double voltage) {
    m_voltageControl.Output = voltage;
    m_controller.setControl(m_voltageControl);
  }

  public void setCurrentOutput(final double current, final double maxAbsDutyCycle) {
    m_currentControl.Output = current;
    m_currentControl.MaxAbsDutyCycle = maxAbsDutyCycle;
    m_controller.setControl(m_currentControl);
  }

  public void setPositionSetpoint(final int slot, final double setpoint) {
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    m_positionControl.Slot = slot;
    m_positionControl.Position = toNativeSensorPosition(setpoint);
    m_positionControl.FeedForward = feedforwardVolts;
    m_controller.setControl(m_positionControl);
  }

  public void setMotionMagicPositionSetpoint(final int slot, final double setpoint) {
    setMotionMagicPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setMotionMagicPositionSetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    m_motionMagicControl.Slot = slot;
    m_motionMagicControl.Position = toNativeSensorPosition(setpoint);
    m_motionMagicControl.FeedForward = feedforwardVolts;
    m_controller.setControl(m_motionMagicControl);
  }

  public void setDynamicMotionMagicPositionSetpoint(
      final int slot,
      final double setpoint,
      final double velocity,
      final double acceleration,
      final double jerk) {
    setDynamicMotionMagicPositionSetpoint(slot, setpoint, velocity, acceleration, jerk, 0.0);
  }

  public void setDynamicMotionMagicPositionSetpoint(
      final int slot,
      final double setpoint,
      final double velocity,
      final double acceleration,
      final double jerk,
      final double feedforwardVolts) {
    m_dynamicMotionMagicControl.Slot = slot;
    m_dynamicMotionMagicControl.Position = toNativeSensorPosition(setpoint);
    m_dynamicMotionMagicControl.FeedForward = feedforwardVolts;
    m_dynamicMotionMagicControl.Velocity = toNativeSensorVelocity(velocity);
    m_dynamicMotionMagicControl.Acceleration = toNativeSensorVelocity(acceleration);
    m_dynamicMotionMagicControl.Jerk = toNativeSensorVelocity(jerk);
    m_controller.setControl(m_dynamicMotionMagicControl);
  }

  public void setVelocitySetpoint(final int slot, final double setpoint) {
    setVelocitySetpoint(slot, setpoint, 0.0);
  }

  public void setVelocitySetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    m_velocityControl.Slot = slot;
    m_velocityControl.Velocity = toNativeSensorVelocity(setpoint);
    m_velocityControl.FeedForward = feedforwardVolts;
    m_controller.setControl(m_velocityControl);
  }

  public double getPercentOutput() {
    return m_inputs.percentOutput;
  }

  public double getPhysicalPercentOutput() {
    return (getInverted() ? -1.0 : 1.0) * getPercentOutput();
  }

  public double getSupplyCurrent() {
    return m_inputs.supplyCurrent;
  }

  public double getStatorCurrent() {
    return m_inputs.statorCurrent;
  }

  public double getClosedLoopReference() {
    return m_inputs.closedLoopReference;
  }

  public double getClosedLoopReferenceSlope() {
    return m_inputs.closedLoopReferenceSlope;
  }

  public boolean getInverted() {
    // This assumes that the config has been properly applied.
    return m_config.INVERTED;
  }

  public void zeroSensorPosition() {
    setSensorPosition(0.0);
  }

  public void setSensorPosition(final double pos) {
    // TODO: Handle zero offset internally.
    m_controller.setPosition(toNativeSensorPosition(pos));
  }

  public double getSensorPosition() {
    return m_inputs.sensorPosition;
  }

  public double getLatencyCompensatedSensorPosition() {
    return m_inputs.latencyCompensatedSensorPosition;
  }

  public double getSensorVelocity() {
    return m_inputs.sensorVelocity;
  }

  public MechanismRatio getMechanismRatio() {
    return m_ratio;
  }

  public double toNativeSensorPosition(final double pos) {
    return toNativeSensorPosition(pos, m_ratio, m_config.bootPositionOffset);
  }

  public static double toNativeSensorPosition(
      final double pos, final MechanismRatio mr, final double bootPositionOffset) {
    // Native position is rotations. There is 1 rotation per revolution (lol).
    return mr.mechanismPositionToSensorRadians(pos - bootPositionOffset) / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(final double pos) {
    return (pos / toNativeSensorPosition(1.0, m_ratio, 0.0)) + m_config.bootPositionOffset;
  }

  public double toNativeSensorVelocity(final double vel) {
    return toNativeSensorVelocity(vel, m_ratio);
  }

  public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
    // Native velocity is rotations per second.
    return toNativeSensorPosition(vel, mr, 0.0);
  }

  public double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorPositionAndVelocity(
      final double pos, final double vel, final double dt, final MechanismRatio mr) {
    // Convert position into rotations.
    final double rotations = toNativeSensorPosition(pos, mr, 0.0);
    // Convert velocity into rotations per second.
    final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = getInverted() ? -1.0 : 1.0;
    m_simState.setRotorVelocity(sign * rotationsPerSecond);
    m_simState.setRawRotorPosition(sign * rotations);
  }

  public void setSimSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
    // Convert velocity into rotations per second.
    final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = getInverted() ? -1.0 : 1.0;
    m_simState.setRotorVelocity(sign * rotationsPerSecond);
    m_simState.addRotorPosition(sign * rotationsPerSecond * dt);
  }
}
