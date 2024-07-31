// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.planning.QuixTrapezoidProfile;
import frc.quixlib.viz.Link2d;
import frc.quixlib.wpilib.LoggedDigitalInput;
import frc.robot.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {
  public final LoggedDigitalInput m_beamBreak =
      new LoggedDigitalInput(Constants.Launcher.beamBreakPort);

  // This motor controls the upper launch wheels
  private final QuixTalonFX m_upperLaunchMotor =
      new QuixTalonFX(
          Constants.Launcher.upperMotorID,
          Constants.Launcher.upperMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Launcher.upperMotorInvert)
              .setPIDConfig(
                  Constants.Launcher.launcherVelocityPIDSlot,
                  Constants.Launcher.launcherVelocityPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(120.0));

  // This motor controls the lower launch wheels
  private final QuixTalonFX m_lowerLaunchMotor =
      new QuixTalonFX(
          Constants.Launcher.lowerMotorID,
          Constants.Launcher.lowerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Launcher.lowerMotorInvert)
              .setPIDConfig(
                  Constants.Launcher.launcherVelocityPIDSlot,
                  Constants.Launcher.launcherVelocityPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(120.0));

  // This motor controls the feed rollers
  private final QuixTalonFX m_feedRollerMotor =
      new QuixTalonFX(
          Constants.Launcher.feedMotorID,
          Constants.Launcher.feedMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Launcher.feedMotorInvert)
              .setBrakeMode()
              .setPIDConfig(
                  Constants.Launcher.feedVelocityPIDSlot, Constants.Launcher.feedVelocityPIDConfig)
              .setPIDConfig(
                  Constants.Launcher.feedPositionPIDSlot, Constants.Launcher.feedPositionPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(60.0));

  // This motor controls the redirect roller
  private final QuixTalonFX m_redirectRollerMotor =
      new QuixTalonFX(
          Constants.Launcher.redirectMotorID,
          Constants.Launcher.redirectMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Launcher.redirectMotorInvert)
              .setBrakeMode()
              .setPIDConfig(
                  Constants.Launcher.redirectVelocityPIDSlot,
                  Constants.Launcher.redirectVelocityPIDConfig)
              .setPIDConfig(
                  Constants.Launcher.redirectPositionPIDSlot,
                  Constants.Launcher.redirectPositionPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(60.0));

  // This motor controls the angle of the arm (launcher)
  private final QuixTalonFX m_armAngleMotor =
      new QuixTalonFX(
          Constants.Launcher.armMotorID,
          Constants.Launcher.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Launcher.armMotorInvert)
              .setBrakeMode()
              .setPIDConfig(
                  Constants.Launcher.armPositionPIDSlot, Constants.Launcher.armPositionPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBootPositionOffset(Constants.Launcher.startingAngle)
              .setReverseSoftLimit(Constants.Launcher.minAngle)
              .setForwardSoftLimit(Constants.Launcher.maxAngle));

  private QuixTrapezoidProfile m_armProfile;
  private final Timer m_armTimer = new Timer();
  private State m_armState = new State(m_armAngleMotor.getSensorPosition(), 0.0);

  private boolean m_beamBreakLastState = false;
  private Double m_beamBreakFeedPosition = null;
  private Double m_beamBreakRedirectPosition = null;

  public LauncherSubsystem(
      Link2d launcherArmViz,
      Link2d launcherTopWheelViz,
      Link2d launcherBottomWheelViz,
      Link2d launcherFeedRollerViz,
      Link2d launcherRedirectRollerViz) {

    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Launcher.armTrapConstraints,
            new State(Constants.Launcher.startingAngle, 0.0),
            m_armState);
    m_armTimer.start();

    // Setup viz.
    m_launcherArmViz = launcherArmViz;
    m_launcherTopWheelViz = launcherTopWheelViz;
    m_launcherBottomWheelViz = launcherBottomWheelViz;
    m_launcherFeedRollerViz = launcherFeedRollerViz;
    m_launcherRedirectRollerViz = launcherRedirectRollerViz;
  }

  public boolean hasPiece() {
    return m_beamBreak.get();
  }

  public boolean readyForIntake() {
    return !hasPiece()
        && isAtAngle(Constants.Launcher.intakeAngle, Constants.Launcher.intakeAngleTolerance);
  }

  public double getArmAngle() {
    return m_armAngleMotor.getSensorPosition();
  }

  public boolean isAtLaunchVelocity(double launchVelocity, double tolerance) {
    return Math.abs(launchVelocity - m_upperLaunchMotor.getSensorVelocity()) <= tolerance
        && Math.abs(launchVelocity - m_lowerLaunchMotor.getSensorVelocity()) <= tolerance;
  }

  public boolean isAtAutoStartVelocity() {
    return m_upperLaunchMotor.getSensorVelocity() > Constants.Launcher.autoLaunchStartVelocity
        && m_lowerLaunchMotor.getSensorVelocity() > Constants.Launcher.autoLaunchStartVelocity;
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_armAngleMotor.getSensorPosition()) <= tolerance;
  }

  public void setArmAngle(double targetArmAngle) {
    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Launcher.armTrapConstraints,
            new State(
                MathUtils.clamp(
                    targetArmAngle, Constants.Launcher.minAngle, Constants.Launcher.maxAngle),
                0.0),
            m_armState);
    m_armTimer.reset();
  }

  public void setArmAngleSlow(double targetArmAngle) {
    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Launcher.armSlowTrapConstraints,
            new State(
                MathUtils.clamp(
                    targetArmAngle, Constants.Launcher.minAngle, Constants.Launcher.maxAngle),
                0.0),
            m_armState);
    m_armTimer.reset();
  }

  public void setFeedVelocity(double velocity) {
    final double feedffVolts = Constants.Launcher.feedRollerFeedforward.calculate(velocity);
    if (velocity == 0.0) {
      m_feedRollerMotor.setPercentOutput(0.0);
    } else {
      m_feedRollerMotor.setVelocitySetpoint(
          Constants.Launcher.feedVelocityPIDSlot, velocity, feedffVolts);
    }
  }

  public void setFeedPower(double power) {
    m_feedRollerMotor.setPercentOutput(power);
  }

  public void setRedirectPower(double power) {
    m_redirectRollerMotor.setPercentOutput(power);
  }

  public void stopFeed() {
    m_feedRollerMotor.setPercentOutput(0.0);
  }

  public void setRedirectVelocity(double velocity) {
    final double redirectffVolts = Constants.Launcher.redirectRollerFeedforward.calculate(velocity);
    if (velocity == 0.0) {
      m_redirectRollerMotor.setPercentOutput(0.0);
    } else {
      m_redirectRollerMotor.setVelocitySetpoint(
          Constants.Launcher.redirectVelocityPIDSlot, velocity, redirectffVolts);
    }
  }

  public void moveFeedAndRedirectToPositionOffset(double rads) {
    if (m_beamBreakFeedPosition != null && m_beamBreakRedirectPosition != null) {
      m_feedRollerMotor.setPositionSetpoint(
          Constants.Launcher.feedPositionPIDSlot, m_beamBreakFeedPosition + rads);
      m_redirectRollerMotor.setPositionSetpoint(
          Constants.Launcher.redirectPositionPIDSlot, m_beamBreakRedirectPosition + rads);
    }
  }

  /** Velocity in rad/s */
  public void setLaunchVelocity(double velocity) {
    final double ffVolts = Constants.Launcher.launcherFeedforward.calculate(velocity);

    if (velocity == 0.0) {
      m_upperLaunchMotor.setPercentOutput(0.0);
      m_lowerLaunchMotor.setPercentOutput(0.0);
    } else {
      m_upperLaunchMotor.setVelocitySetpoint(
          Constants.Launcher.launcherVelocityPIDSlot, velocity, ffVolts);
      m_lowerLaunchMotor.setVelocitySetpoint(
          Constants.Launcher.launcherVelocityPIDSlot, velocity, ffVolts);
    }
  }

  public double setLinearLaunchVelocity(double metersPerSecond) {
    // Linear approximation of launch velocity.
    final double radsPerSec =
        (metersPerSecond / Constants.Launcher.defaultShotVelocity)
            * Constants.Launcher.launchVelocity;
    setLaunchVelocity(radsPerSec);
    return radsPerSec;
  }

  public void disabledInit() {
    m_armAngleMotor.setBrakeMode(true);
  }

  public void disabledExit() {
    m_armAngleMotor.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_upperLaunchMotor.updateInputs();
    m_lowerLaunchMotor.updateInputs();
    m_feedRollerMotor.updateInputs();
    m_redirectRollerMotor.updateInputs();
    m_armAngleMotor.updateInputs();
    m_beamBreak.updateInputs();

    if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_armState = new State(m_armAngleMotor.getSensorPosition(), 0.0);
      setArmAngle(m_armAngleMotor.getSensorPosition());
    }

    // Track the position where the beam break is broken.
    if (m_beamBreak.get()) {
      if (!m_beamBreakLastState) {
        m_beamBreakFeedPosition = m_feedRollerMotor.getSensorPosition();
        m_beamBreakRedirectPosition = m_redirectRollerMotor.getSensorPosition();
      }
      m_beamBreakLastState = true;
    } else {
      m_beamBreakFeedPosition = null;
      m_beamBreakRedirectPosition = null;
      m_beamBreakLastState = false;
    }

    LoggerHelper.recordCurrentCommand("Launcher", this);
    Logger.recordOutput("Launcher/Beam Break", m_beamBreak.get());

    m_armState = m_armProfile.calculate(m_armTimer.get());
    m_armAngleMotor.setPositionSetpoint(
        Constants.Launcher.armPositionPIDSlot,
        m_armState.position,
        // Arm angle is defined as positive when the launcher is pointed up, but the CG is on the
        // other side with some offset, so we need to negate the angle and voltage for FF.
        -Constants.Launcher.armFeedForward.calculate(
            -m_armState.position + Constants.Launcher.cgOffset, -m_armState.velocity));

    Logger.recordOutput(
        "Launcher/Current Arm Angle (deg)",
        Units.radiansToDegrees(m_armAngleMotor.getSensorPosition()));
    Logger.recordOutput(
        "Launcher/Target Arm Angle (deg)", Units.radiansToDegrees(m_armState.position));
    Logger.recordOutput(
        "Launcher/Arm Angle Error (deg)",
        Units.radiansToDegrees(m_armState.position - m_armAngleMotor.getSensorPosition()));

    Logger.recordOutput(
        "Launcher/Current Redirect Roller Velocity (rad per sec)",
        m_redirectRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Launcher/Current Feed Roller Velocity (rad per sec)",
        m_feedRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Launcher/Current Top Launch Wheel Velocity (rad per sec)",
        m_upperLaunchMotor.getSensorVelocity());
    Logger.recordOutput(
        "Launcher/Current Lower Launch Wheel Velocity (rad per sec)",
        m_lowerLaunchMotor.getSensorVelocity());

    Logger.recordOutput(
        "Launcher/Current Feed Roller Current (A)", m_feedRollerMotor.getStatorCurrent());
    Logger.recordOutput(
        "Launcher/Current Top Launch Wheel Current (A)", m_upperLaunchMotor.getStatorCurrent());
    Logger.recordOutput(
        "Launcher/Current Lower Launch Wheel Current (A)", m_lowerLaunchMotor.getStatorCurrent());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Note that the arm simulated backwards because the sim requires zero angle to be gravity acting
  // down on the arm, but gravity acts "up" on the arm from the perspective of the launch angle.
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Launcher.armMotorRatio.reduction(),
          Constants.Launcher.simArmMOI,
          Constants.Launcher.simArmCGLength,
          -Constants.Launcher.maxAngle, // Arm is simulated backwards
          -Constants.Launcher.minAngle, // Arm is simulated backwards
          true, // Simulate gravity
          Constants.Launcher.startingAngle);

  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_topWheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Launcher.simWheelMOI,
              Constants.Launcher.upperMotorRatio.reduction()),
          m_simMotor);

  private static final FlywheelSim m_bottomWheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Launcher.simWheelMOI,
              Constants.Launcher.lowerMotorRatio.reduction()),
          m_simMotor);

  private static final FlywheelSim m_feedRollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Launcher.simRollerMOI,
              Constants.Launcher.feedMotorRatio.reduction()),
          m_simMotor);

  private static final FlywheelSim m_redirectRollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Launcher.simRollerMOI,
              Constants.Launcher.redirectMotorRatio.reduction()),
          m_simMotor);

  // Visualization
  private final Link2d m_launcherArmViz;
  private final Link2d m_launcherTopWheelViz;
  private final Link2d m_launcherBottomWheelViz;
  private final Link2d m_launcherFeedRollerViz;
  private final Link2d m_launcherRedirectRollerViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(-m_armAngleMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(LoggedRobot.defaultPeriodSecs);
    // Arm is simulated backwards because gravity acting on a horizontal arm needs to be at
    // zero degrees
    m_armAngleMotor.setSimSensorPositionAndVelocity(
        -m_armSim.getAngleRads() - Constants.Launcher.startingAngle,
        m_armSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Launcher.armMotorRatio);

    m_topWheelSim.setInput(
        m_upperLaunchMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_topWheelSim.update(LoggedRobot.defaultPeriodSecs);
    m_upperLaunchMotor.setSimSensorVelocity(
        m_topWheelSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Launcher.upperMotorRatio);

    m_bottomWheelSim.setInput(
        m_lowerLaunchMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_bottomWheelSim.update(LoggedRobot.defaultPeriodSecs);
    m_lowerLaunchMotor.setSimSensorVelocity(
        m_bottomWheelSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Launcher.lowerMotorRatio);

    m_feedRollerSim.setInput(
        m_feedRollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_feedRollerSim.update(LoggedRobot.defaultPeriodSecs);
    m_feedRollerMotor.setSimSensorVelocity(
        m_feedRollerSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Launcher.feedMotorRatio);

    m_redirectRollerSim.setInput(
        m_redirectRollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_redirectRollerSim.update(LoggedRobot.defaultPeriodSecs);
    m_redirectRollerMotor.setSimSensorVelocity(
        m_redirectRollerSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Launcher.redirectMotorRatio);

    m_launcherArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.launcherArmPivotX,
            0.0,
            // TODO: Figure out how to do this without hardcoding
            Rotation2d.fromRadians(
                m_armSim.getAngleRads() - Constants.Viz.elevatorAngle.getRadians())));
    m_launcherTopWheelViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.launcherWheelX,
            Constants.Viz.launcherTopWheelY,
            Rotation2d.fromRadians(
                m_launcherTopWheelViz.getRelativeTransform().getRotation().getRadians()
                    + m_topWheelSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
    m_launcherBottomWheelViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.launcherWheelX,
            Constants.Viz.launcherBottomWheelY,
            Rotation2d.fromRadians(
                m_launcherBottomWheelViz.getRelativeTransform().getRotation().getRadians()
                    + m_bottomWheelSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
    m_launcherFeedRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.launcherFeedRollerX,
            Constants.Viz.launcherFeedRollerY,
            Rotation2d.fromRadians(
                m_launcherFeedRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_feedRollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
    m_launcherRedirectRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.launcherRedirectRollerX,
            Constants.Viz.launcherRedirectRollerY,
            Rotation2d.fromRadians(
                m_launcherRedirectRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_redirectRollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}
