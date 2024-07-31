package frc.quixlib.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;
import org.littletonrobotics.junction.LoggedRobot;

public class QuixSwerveModule {
  private static final int kDriveVelocityPIDSlot = 0;
  private static final int kSteeringPIDSlot = 0;

  private final Translation2d m_position;
  private final double m_absEncoderOffsetRad;
  protected final QuixTalonFX m_driveMotor;
  protected final QuixTalonFX m_steeringMotor;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final MechanismRatio m_driveRatio;
  private final MechanismRatio m_steeringRatio;
  // For every steering rotation with the wheel fixed, the drive motor turns this much. May be
  // negative.
  private final double m_steerDriveCouplingRatio;
  private final QuixCANCoder m_absSteeringEncoder;
  private final double m_wheelCircumference;

  private SwerveModuleState m_lastCommandedState;
  private double m_steeringZeroPosition = 0.0;

  public QuixSwerveModule(
      final Translation2d position,
      final CANDeviceID driveMotorID,
      final CANDeviceID steeringMotorID,
      final CANDeviceID absEncoderID,
      final PIDConfig drivePIDConfig,
      final SimpleMotorFeedforward driveFeedforward,
      final PIDConfig steeringPIDConfig,
      final MechanismRatio driveRatio,
      final MechanismRatio steeringRatio,
      final double steerDriveCouplingRatio,
      final double absEncoderOffsetRad,
      final double wheelCircumference) {
    m_position = position;
    m_absEncoderOffsetRad = absEncoderOffsetRad;
    m_driveMotor =
        new QuixTalonFX(
            driveMotorID,
            driveRatio,
            QuixTalonFX.makeDefaultConfig()
                .setBrakeMode()
                .setSupplyCurrentLimit(40)
                .setStatorCurrentLimit(80)
                .setPIDConfig(kDriveVelocityPIDSlot, drivePIDConfig));
    m_driveFeedforward = driveFeedforward;
    m_steeringMotor =
        new QuixTalonFX(
            steeringMotorID,
            steeringRatio,
            QuixTalonFX.makeDefaultConfig()
                .setInverted(true)
                .setSupplyCurrentLimit(30)
                .setStatorCurrentLimit(60)
                .setPIDConfig(kSteeringPIDSlot, steeringPIDConfig));
    m_driveRatio = driveRatio;
    m_steeringRatio = steeringRatio;
    m_steerDriveCouplingRatio = steerDriveCouplingRatio;
    m_absSteeringEncoder = new QuixCANCoder(absEncoderID, new MechanismRatio());
    m_wheelCircumference = wheelCircumference;

    zeroToAbsPosition();
    m_lastCommandedState = getState();

    // For simulation
    final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
    m_driveSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(m_simMotor, 0.01, driveRatio.reduction()),
            m_simMotor);
    m_steeringSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            steeringRatio.reduction(),
            0.001, // MOI
            0.0, // Length (m)
            Double.NEGATIVE_INFINITY, // Min angle
            Double.POSITIVE_INFINITY, // Max angle
            false, // Simulate gravity
            0.0 // Starting angle (rads)
            );
  }

  public void updateInputs() {
    m_absSteeringEncoder.updateInputs();
    m_driveMotor.updateInputs();
    m_steeringMotor.updateInputs();
  }

  public Translation2d getPosition() {
    return m_position;
  }

  public void zeroToAbsPosition() {
    final double absAngle = m_absSteeringEncoder.getAbsPosition() - m_absEncoderOffsetRad;
    m_steeringZeroPosition = m_steeringMotor.getSensorPosition() - absAngle;
  }

  public double getAbsoluteSensorAngle() {
    return m_absSteeringEncoder.getAbsPosition();
  }

  public double getSteeringAngle() {
    return m_steeringMotor.getSensorPosition() - m_steeringZeroPosition;
  }

  public void setDesiredState(final SwerveModuleState desiredState, final boolean isClosedLoop) {
    if (isClosedLoop) {
      m_driveMotor.setVelocitySetpoint(
          kDriveVelocityPIDSlot,
          desiredState.speedMetersPerSecond,
          m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    } else {
      m_driveMotor.setVoltageOutput(
          m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    m_steeringMotor.setPositionSetpoint(
        kSteeringPIDSlot, desiredState.angle.getRadians() + m_steeringZeroPosition);

    // Save this state
    m_lastCommandedState = desiredState;
  }

  public SwerveModuleState getState() {
    final double velocity = m_driveMotor.getSensorVelocity();
    final double angle = getSteeringAngle();
    return new SwerveModuleState(velocity, new Rotation2d(angle));
  }

  public SwerveModuleState getLastCommandedState() {
    return m_lastCommandedState;
  }

  public SwerveModulePosition getPositionState() {
    final double position = m_driveMotor.getLatencyCompensatedSensorPosition();
    final double angle =
        m_steeringMotor.getLatencyCompensatedSensorPosition() - m_steeringZeroPosition;

    // Compute drive position offset (in meters) due to steer coupling.
    final double steerCouplingDriveOffset =
        m_driveMotor
            .getMechanismRatio()
            .sensorRadiansToMechanismPosition(m_steerDriveCouplingRatio * angle);

    return new SwerveModulePosition(position - steerCouplingDriveOffset, new Rotation2d(angle));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final FlywheelSim m_driveSim;
  private final SingleJointedArmSim m_steeringSim;

  /** Simulate one module with naive physics model. */
  public void updateSimPeriodic() {
    // Simulate drive
    m_driveSim.setInput(m_driveMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_driveSim.update(LoggedRobot.defaultPeriodSecs);
    double metersPerSecond =
        m_driveSim.getAngularVelocityRadPerSec() * m_wheelCircumference / (2.0 * Math.PI);
    m_driveMotor.setSimSensorVelocity(metersPerSecond, LoggedRobot.defaultPeriodSecs, m_driveRatio);

    // Simulate steering
    m_steeringSim.setInput(
        m_steeringMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_steeringSim.update(LoggedRobot.defaultPeriodSecs);
    m_steeringMotor.setSimSensorPositionAndVelocity(
        m_steeringSim.getAngleRads(),
        m_steeringSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        m_steeringRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
