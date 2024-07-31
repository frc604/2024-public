package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.devices.QuixStatusSignal;
import frc.quixlib.localization.QuixSwerveLocalizer;
import frc.quixlib.localization.SwerveDriveOdometryMeasurement;
import frc.quixlib.math.MathUtils;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineVisionPacket;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import java.util.ArrayList;

public abstract class QuixSwerve extends SubsystemBase {
  private final QuixPigeon2 m_imu;
  private final ArrayList<QuixVisionCamera> m_cameras;
  private final QuixSwerveModule[] m_modules;
  private final SwerveDriveKinematics m_kinematics;
  private final QuixSwerveModuleSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint m_prevSetpoint;

  private static final SwerveSetpointGenerator.KinematicLimits m_kinematicLimits =
      new SwerveSetpointGenerator.KinematicLimits();
  private final SwerveSetpointGenerator m_setpointGenerator254;
  private SwerveSetpoint m_prevSetpoint254;
  private SwerveModuleState[] m_prevStates;

  private final QuixSwerveLocalizer m_localizer;
  private final QuixSwerveController m_driveController;

  private final QuixVisionSim m_visionSim;

  private final Field2d m_fieldViz;
  private final Mechanism2d m_viz = new Mechanism2d(100, 100);
  private final MechanismRoot2d[] m_vizModuleRoots;
  private final MechanismLigament2d[] m_vizModuleCurrentState;
  private final MechanismLigament2d[] m_vizModuleTargetState;

  private final StructArrayPublisher<SwerveModuleState> m_swerveStatePublishser =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve: Actual Module States", SwerveModuleState.struct)
          .publish();
  private final StructArrayPublisher<SwerveModuleState> m_swerveStateTargetPublishser =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve: Target Module States", SwerveModuleState.struct)
          .publish();

  /**
   * Swerve drive class that handles all swerve-related functions, including kinematics, control,
   * and localization.
   */
  public QuixSwerve(
      final QuixPigeon2 imu,
      final ArrayList<QuixVisionCamera> cameras,
      final double maxDriveSpeed,
      final double maxModuleAcceleration,
      final double maxModuleSteeringRate,
      final QuixSwerveController driveController,
      final Fiducial[] targets,
      final QuixVisionSim visionSim,
      final Field2d fieldViz) {
    m_imu = imu;
    m_cameras = cameras;
    m_modules = createModules();
    m_kinematics = new SwerveDriveKinematics(getModulePositions());
    m_setpointGenerator =
        new QuixSwerveModuleSetpointGenerator(
            m_kinematics, maxDriveSpeed, maxModuleAcceleration, maxModuleSteeringRate);
    m_prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

    m_kinematicLimits.kMaxDriveVelocity = maxDriveSpeed;
    m_kinematicLimits.kMaxDriveAcceleration = maxModuleAcceleration;
    m_kinematicLimits.kMaxSteeringVelocity = maxModuleSteeringRate;
    m_setpointGenerator254 = new SwerveSetpointGenerator(m_kinematics, getModulePositions());
    m_prevSetpoint254 = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
    m_prevStates = getModuleStates();

    m_localizer =
        new QuixSwerveLocalizer(
            m_kinematics,
            new Rotation2d(m_imu.getContinuousYaw()),
            getModulePositionStates(),
            new Pose2d(),
            targets);
    m_driveController = driveController;
    zeroModuleEncoders();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    m_visionSim = visionSim;

    // Setup Viz
    m_fieldViz = fieldViz;
    SmartDashboard.putData("Swerve Viz", m_viz);
    m_vizModuleRoots = new MechanismRoot2d[m_modules.length];
    m_vizModuleCurrentState = new MechanismLigament2d[m_modules.length];
    m_vizModuleTargetState = new MechanismLigament2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      m_vizModuleRoots[i] =
          m_viz.getRoot(
              "Swerve Module ID: " + i,
              50 + m_modules[i].getPosition().getX() * 100,
              50 + m_modules[i].getPosition().getY() * 100);
      m_vizModuleCurrentState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Current State", 0, 0, 10, new Color8Bit(Color.kRed)));
      m_vizModuleTargetState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Target State", 0, 0, 5, new Color8Bit(Color.kGreen)));
    }
  }

  /**
   * Returns an array of QuixSwerveModules that define the individual module configurations. Called
   * on construction.
   */
  protected abstract QuixSwerveModule[] createModules();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (var module : m_modules) {
      module.m_driveMotor.checkFaultsAndReconfigureIfNecessary();
      module.m_steeringMotor.checkFaultsAndReconfigureIfNecessary();
      // TODO: Handle reconfiguration if necessary.
    }

    // Update localization only when enabled.
    if (DriverStation.isEnabled() || true) {
      final double yaw =
          QuixStatusSignal.getLatencyCompensatedValue(
              m_imu.continuousYawSignal(), m_imu.yawRateSignal());
      final var odometryMeasurement =
          new SwerveDriveOdometryMeasurement(new Rotation2d(yaw), getModulePositionStates());

      // Combining measurements from all cameras.
      final ArrayList<PipelineVisionPacket> visionMeasurements = new ArrayList<>();
      for (var camera : m_cameras) {
        visionMeasurements.add(camera.getLatestMeasurement());
      }

      m_localizer.update(odometryMeasurement, visionMeasurements);
    }

    // Plot
    updateModuleViz();
    m_fieldViz.getObject("Odometry").setPose(m_localizer.getOdometryPose());
    m_fieldViz.getObject("Localizer Raw").setPose(m_localizer.getRawPose());
    m_fieldViz.getObject("Localizer").setPose(m_localizer.getPose());

    SmartDashboard.putNumber("IMU: Roll", m_imu.getRoll());
    SmartDashboard.putNumber("IMU: Pitch", m_imu.getPitch());
    SmartDashboard.putNumber("IMU: Yaw", m_imu.getContinuousYaw());
    SmartDashboard.putNumber("IMU: Roll rate", m_imu.getRollRate());
    SmartDashboard.putNumber("IMU: Pitch rate", m_imu.getPitchRate());
    SmartDashboard.putNumber("IMU: Yaw rate", m_imu.getYawRate());
  }

  /** Sets the IMU and localizer to the given pose. */
  public void resetPose(final Pose2d pose) {
    m_imu.setContinuousYaw(pose.getRotation().getRadians());
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()), getModulePositionStates(), pose);
  }

  /**
   * Drive with open loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveOpenLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, false);
  }

  /**
   * Drive with closed loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveClosedLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, true);
  }

  /**
   * Drive with the given translation and rotation velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether the provided velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   * @param isClosedLoop Whether to use closed-loop module velocities
   */
  private void drive(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub,
      final boolean isClosedLoop) {
    final ChassisSpeeds desiredChassisSpeeds =
        correctForDynamics(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVel, yVel, thetaVel, new Rotation2d(m_imu.getContinuousYaw()))
                : new ChassisSpeeds(xVel, yVel, thetaVel));

    final int antiScrubMode = 604;
    switch (antiScrubMode) {
      case 604:
        {
          m_prevSetpoint =
              m_setpointGenerator.getFeasibleSetpoint(
                  m_prevSetpoint, desiredChassisSpeeds, allowedScrub);
          setDesiredModuleStates(m_prevSetpoint.moduleStates, isClosedLoop);
          break;
        }
      case 254:
        {
          var setpoint254 =
              m_setpointGenerator254.generateSetpoint(
                  m_kinematicLimits,
                  m_prevSetpoint254,
                  desiredChassisSpeeds,
                  TimedRobot.kDefaultPeriod);

          // Unwrap states
          var curStates = getModuleStates();
          var newStates254 = new SwerveModuleState[setpoint254.moduleStates.length];
          for (int i = 0; i < setpoint254.moduleStates.length; i++) {
            newStates254[i] = new SwerveModuleState();
            newStates254[i].speedMetersPerSecond = setpoint254.moduleStates[i].speedMetersPerSecond;
            newStates254[i].angle =
                new Rotation2d(
                    MathUtils.placeInScope(
                        setpoint254.moduleStates[i].angle.getRadians(),
                        curStates[i].angle.getRadians()));
          }
          setpoint254.moduleStates = newStates254;
          m_prevSetpoint254 = setpoint254;
          setDesiredModuleStates(m_prevSetpoint254.moduleStates, isClosedLoop);
          break;
        }
      default:
        {
          var newStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);

          // Unwrap states
          var curStates = getModuleStates();
          for (int i = 0; i < newStates.length; i++) {
            newStates[i] =
                QuixSwerveModuleSetpointGenerator.optimizeModule(newStates[i], curStates[i]);
          }
          m_prevStates = newStates;
          setDesiredModuleStates(m_prevStates, isClosedLoop);
          break;
        }
    }
  }

  /**
   * Approximation to correct for for swerve second order dynamics issue. Inspired by 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(final ChassisSpeeds originalSpeeds) {
    final Pose2d robotPoseVel =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * TimedRobot.kDefaultPeriod,
            originalSpeeds.vyMetersPerSecond * TimedRobot.kDefaultPeriod,
            Rotation2d.fromRadians(
                originalSpeeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod));
    final Twist2d twistVel = MathUtils.log(robotPoseVel);
    return new ChassisSpeeds(
        twistVel.dx / TimedRobot.kDefaultPeriod,
        twistVel.dy / TimedRobot.kDefaultPeriod,
        twistVel.dtheta / TimedRobot.kDefaultPeriod);
  }

  /**
   * Drives the swerve to the target pose.
   *
   * @param targetPose The target pose in field-frame
   * @param xVelocityRef Field-relative x-velocity feed-forward
   * @param yVelocityRef Field-relative y-velocity feed-forward
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   * @return the error between the current pose and the target pose
   */
  public Pose2d driveToPose(
      final Pose2d targetPose,
      final double xVelocityRef,
      final double yVelocityRef,
      final double thetaVelocityRef,
      final double allowedScrub) {
    final Pose2d currentPose = m_localizer.getPose();
    final Pose2d poseError =
        new Pose2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY(),
            targetPose.getRotation().minus(currentPose.getRotation()));

    m_fieldViz.getObject("Target Pose").setPose(targetPose);
    SmartDashboard.putNumber("Chassis x actual", currentPose.getX());
    SmartDashboard.putNumber("Chassis x target", targetPose.getX());
    SmartDashboard.putNumber("Chassis x error", poseError.getX());
    SmartDashboard.putNumber("Chassis y actual", currentPose.getY());
    SmartDashboard.putNumber("Chassis y target", targetPose.getY());
    SmartDashboard.putNumber("Chassis y error", poseError.getY());
    SmartDashboard.putNumber(
        "Chassis theta actual (degrees)", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "Chassis theta target (degrees)", targetPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Chassis theta error (degrees)", poseError.getRotation().getDegrees());
    final ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(
            currentPose, targetPose, xVelocityRef, yVelocityRef, thetaVelocityRef);
    driveClosedLoop(
        fieldRelativeChassisSpeeds.vxMetersPerSecond,
        fieldRelativeChassisSpeeds.vyMetersPerSecond,
        fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
        /*fieldRelative=*/ false,
        allowedScrub);
    return poseError;
  }

  /**
   * Turns the swerve in-place to the target angle.
   *
   * @param angle The target angle in field-frame
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void turnToAngle(
      final Rotation2d angle, final double thetaVelocityRef, final double allowedScrub) {
    final Pose2d targetPose = new Pose2d(getPose().getTranslation(), angle);
    final ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(m_localizer.getPose(), targetPose, 0.0, 0.0, thetaVelocityRef);
    driveOpenLoop(
        0.0,
        0.0,
        fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
        /*fieldRelative=*/ false,
        allowedScrub);
  }

  /** Drive open loop with zero velocity. */
  public void stop() {
    driveOpenLoop(0.0, 0.0, 0.0, /*fieldRelative=*/ false, Double.POSITIVE_INFINITY);
  }

  /** Turns modules to form an X pointing at the origin with zero velocity. */
  public void stopWithX() {
    final var moduleStates = getModuleStates();
    final SwerveModuleState[] desiredStates = new SwerveModuleState[moduleStates.length];
    for (int i = 0; i < moduleStates.length; i++) {
      final double angle =
          Math.atan2(m_modules[i].getPosition().getY(), m_modules[i].getPosition().getX());
      desiredStates[i] =
          new SwerveModuleState(
              0.0,
              new Rotation2d(MathUtils.placeInScope(angle, moduleStates[i].angle.getRadians())));
    }
    setDesiredModuleStates(desiredStates, false);
  }

  private ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public FieldSpeeds getFieldSpeeds() {
    final Rotation2d rotation = getPose().getRotation();
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    final double vxField =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    final double vyField =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();
    return new FieldSpeeds(vxField, vyField, chassisSpeeds.omegaRadiansPerSecond);
  }

  private Translation2d[] getModulePositions() {
    final Translation2d[] positions = new Translation2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  private SwerveModuleState[] getLastCommandedModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getLastCommandedState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositionStates() {
    final SwerveModulePosition[] states = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getPositionState(true);
    }
    return states;
  }

  public Pose2d getPose() {
    return m_localizer.getPose();
  }

  /** Returns the velocity (m/s for translation & rad/s for rotation) in the robot frame. */
  public Transform2d getVelocity() {
    final var chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    return new Transform2d(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        new Rotation2d(chassisSpeeds.omegaRadiansPerSecond));
  }

  /** Returns the velocity (m/s for translation & rad/s for rotation) in the field frame. */
  public Transform2d getFieldRelativeVelocity() {
    final Transform2d vel = getVelocity();
    final double c = getPose().getRotation().getCos();
    final double s = getPose().getRotation().getSin();
    return new Transform2d(
        vel.getX() * c - vel.getY() * s, vel.getX() * s + vel.getY() * c, vel.getRotation());
  }

  public void setContinuousYaw(double yaw) {
    m_imu.setContinuousYaw(yaw);
    final Pose2d curPose = getPose();
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()),
        getModulePositionStates(),
        new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(yaw)));
  }

  private void zeroModuleEncoders() {
    for (var module : m_modules) {
      module.zeroToAbsPosition();
    }
  }

  private void setDesiredModuleStates(
      final SwerveModuleState[] desiredStates, final boolean isClosedLoop) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i], isClosedLoop);
    }
  }

  private void updateModuleViz() {
    final double kSpeedVizScalar = 5.0;
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].logMotorStates();

      var currentState = m_modules[i].getState();
      m_vizModuleCurrentState[i].setAngle(currentState.angle);
      m_vizModuleCurrentState[i].setLength(kSpeedVizScalar * currentState.speedMetersPerSecond);

      var targetState = m_modules[i].getLastCommandedState();
      m_vizModuleTargetState[i].setAngle(targetState.angle);
      m_vizModuleTargetState[i].setLength(kSpeedVizScalar * targetState.speedMetersPerSecond);

      SmartDashboard.putNumber("Module Speed " + i, currentState.speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Module Absolute Angle (deg) " + i,
          Math.toDegrees(m_modules[i].getAbsoluteSensorAngle()));
      SmartDashboard.putNumber(
          "Module Angle (deg) " + i, Math.toDegrees(m_modules[i].getSteeringAngle()));
    }
    m_swerveStatePublishser.set(getModuleStates());
    m_swerveStateTargetPublishser.set(getLastCommandedModuleStates());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private Pose2d m_simPose = new Pose2d();

  /** Sets the simulated pose to the given pose. */
  public void resetSimPose(final Pose2d pose) {
    m_simPose = pose;
    m_visionSim.resetSimPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Update vision sim with sim pose.
    m_visionSim.updatePose(m_simPose);

    // TODO: Sim jointly as one system.
    for (var module : m_modules) {
      module.updateSimPeriodic();
    }

    // Update pose by integrating ChassisSpeeds.
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    m_simPose =
        m_simPose.transformBy(
            new Transform2d(
                new Translation2d(
                    chassisSpeeds.vxMetersPerSecond * TimedRobot.kDefaultPeriod,
                    chassisSpeeds.vyMetersPerSecond * TimedRobot.kDefaultPeriod),
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod)));

    // Update IMU based on sim pose.
    m_imu.setSimContinuousYaw(m_simPose.getRotation().getRadians());
  }
  // --- END STUFF FOR SIMULATION ---
}
