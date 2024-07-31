// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Elevator.motorID,
          Constants.Elevator.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0)
              .setInverted(Constants.Elevator.motorInvert)
              .setPIDConfig(Constants.Elevator.motorPositionSlot, Constants.Elevator.motorPIDConfig)
              .setMotionMagicConfig(
                  Constants.Elevator.maxVelocity,
                  Constants.Elevator.maxAcceleration,
                  Constants.Elevator.maxJerk)
              .setReverseSoftLimit(Constants.Elevator.minHeight)
              .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private double m_targetHeight = Constants.Elevator.minHeight;

  public ElevatorSubsystem(Link2d elevatorCarriageViz) {
    // Setup viz.
    m_elevatorCarriageViz = elevatorCarriageViz;
  }

  public boolean readyForIntake() {
    return isAtHeight(Constants.Elevator.stowHeight, Constants.Elevator.stowTolerance);
  }

  public double getHeight() {
    return m_motor.getSensorPosition();
  }

  public void setHeight(double targetHeight) {
    m_targetHeight = targetHeight;
  }

  public boolean isAtHeight(double height, double tolerance) {
    return Math.abs(height - m_motor.getSensorPosition()) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.updateInputs();

    m_motor.setDynamicMotionMagicPositionSetpoint(
        Constants.Elevator.motorPositionSlot,
        m_targetHeight,
        Constants.Elevator.maxVelocity,
        Constants.Elevator.maxAcceleration,
        Constants.Elevator.maxJerk);

    LoggerHelper.recordCurrentCommand("Elevator", this);
    Logger.recordOutput(
        "Elevator/Current Height (in)", Units.metersToInches(m_motor.getSensorPosition()));
    Logger.recordOutput(
        "Elevator/Target Height (in)", Units.metersToInches(m_motor.getClosedLoopReference()));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Elevator.motorRatio.reduction(),
          Constants.Elevator.simCarriageMass,
          Constants.Elevator.sprocketPitchDiameter * 0.5,
          Constants.Elevator.minHeight,
          Constants.Elevator.maxHeight,
          true,
          0);

  // Visualization
  private final Link2d m_elevatorCarriageViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevatorSim.setInput(m_motor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(LoggedRobot.defaultPeriodSecs);
    m_motor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        // m_elevatorSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        LoggedRobot.defaultPeriodSecs,
        Constants.Elevator.motorRatio);

    // Update carriage viz.
    m_elevatorCarriageViz.setRelativeTransform(
        new Transform2d(m_elevatorSim.getPositionMeters(), 0.0, new Rotation2d()));
  }
  // --- END STUFF FOR SIMULATION ---
}
