// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public final DigitalInput m_beamBreak = new DigitalInput(Constants.Intake.beamBreakPort);

  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Intake.rollerMotorID,
          Constants.Intake.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.rollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(Constants.Intake.rollerVelocitySlot, Constants.Intake.rollerPIDConfig));

  private final QuixTalonFX m_deployMotor =
      new QuixTalonFX(
          Constants.Intake.deployMotorID,
          Constants.Intake.deployMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.deployMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0)
              .setMotionMagicConfig(
                  Constants.Intake.deployMaxVelocity,
                  Constants.Intake.deployMaxAcceleration,
                  Constants.Intake.deployMaxJerk)
              .setPIDConfig(Constants.Intake.deployPositionSlot, Constants.Intake.deployPIDConfig)
              .setBootPositionOffset(Constants.Intake.startingAngle)
              .setReverseSoftLimit(Constants.Intake.minAngle)
              .setForwardSoftLimit(Constants.Intake.maxAngle));

  private double m_targetAngle = Constants.Intake.startingAngle;
  private Timer m_lastPieceTimer = new Timer();

  public IntakeSubsystem(Link2d intakeArmViz, Link2d intakeRollerViz) {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_intakeArmViz = intakeArmViz;
    m_intakeRollerViz = intakeRollerViz;
  }

  public boolean hasPiece() {
    return m_beamBreak.get();
  }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
  }

  public double getAngle() {
    return m_deployMotor.getSensorPosition();
  }

  public void setAngle(double targetAngle) {
    m_targetAngle = targetAngle;
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_deployMotor.getSensorPosition()) <= tolerance;
  }

  public void setRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_rollerMotor.setPercentOutput(0.0);
    } else {
      m_rollerMotor.setVelocitySetpoint(
          Constants.Intake.rollerVelocitySlot,
          velocity,
          Constants.Intake.rollerFeedforward.calculate(velocity));
    }
  }

  public void disabledInit() {
    m_deployMotor.setBrakeMode(true);
  }

  public void disabledExit() {
    m_deployMotor.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasPiece()) {
      m_lastPieceTimer.reset();
    }

    SmartDashboard.putBoolean("Intake: Beam Break", m_beamBreak.get());

    m_deployMotor.setMotionMagicPositionSetpoint(
        Constants.Intake.deployPositionSlot, m_targetAngle);

    SmartDashboard.putNumber(
        "Intake: Current Angle (deg)", Units.radiansToDegrees(m_deployMotor.getSensorPosition()));
    SmartDashboard.putNumber(
        "Intake: Target Angle (deg)",
        Units.radiansToDegrees(m_deployMotor.getClosedLoopReference()));
    SmartDashboard.putNumber(
        "Intake: Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_deployMotor.getSensorVelocity()));
    SmartDashboard.putNumber(
        "Intake: Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_deployMotor.getClosedLoopReferenceSlope()));
    SmartDashboard.putNumber(
        "Intake: Current Roller Velocity (rad per sec)", m_rollerMotor.getSensorVelocity());

    m_rollerMotor.logMotorState();
    m_deployMotor.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Intake.deployMotorRatio.reduction(),
          Constants.Intake.simArmMOI,
          Constants.Intake.simArmCGLength,
          Constants.Intake.minAngle,
          Constants.Intake.maxAngle,
          true, // Simulate gravity
          Constants.Intake.startingAngle);

  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Intake.rollerMotorRatio.reduction(),
          Constants.Intake.simRollerMOI);

  // Visualization
  private final Link2d m_intakeArmViz;
  private final Link2d m_intakeRollerViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_deployMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_deployMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - Constants.Intake.startingAngle,
        // m_armSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);

    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);

    // Update arm viz.
    m_intakeArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.intakePivotX,
            Constants.Viz.intakePivotY,
            Rotation2d.fromRadians(m_armSim.getAngleRads())));
    m_intakeRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.intakeArmLength,
            0.0,
            Rotation2d.fromRadians(
                m_intakeRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_rollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}
