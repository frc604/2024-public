// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Climber.motorID,
          Constants.Climber.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(100.0)
              .setInverted(Constants.Climber.motorInvert)
              .setPIDConfig(
                  Constants.Climber.unloadedPositionSlot, Constants.Climber.unloadedPIDConfig)
              .setPIDConfig(
                  Constants.Climber.loadedPositionSlot, Constants.Climber.loadedPIDConfig));

  public ClimberSubsystem() {}

  public void zeroSensorPosition() {
    m_motor.zeroSensorPosition();
  }

  public void biasDown() {
    m_motor.setCurrentOutput(-5.0, 0.2);
  }

  public void setPositionUnloaded(double targetPosition) {
    m_motor.setDynamicMotionMagicPositionSetpoint(
        Constants.Climber.unloadedPositionSlot,
        targetPosition,
        Constants.Climber.maxVelocity,
        Constants.Climber.maxAcceleration,
        Constants.Climber.maxJerk);
  }

  public void setPositionLoaded(double targetPosition) {
    m_motor.setDynamicMotionMagicPositionSetpoint(
        Constants.Climber.loadedPositionSlot,
        targetPosition,
        Constants.Climber.slowMaxVelocity,
        Constants.Climber.slowMaxAcceleration,
        Constants.Climber.slowMaxJerk);
  }

  public boolean isAtPosition(double height, double tolerance) {
    return Math.abs(height - m_motor.getSensorPosition()) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.updateInputs();

    LoggerHelper.recordCurrentCommand("Climber", this);
    Logger.recordOutput("Climber/Current Pos (rads)", m_motor.getSensorPosition());
    Logger.recordOutput("Climber/Target Pos (rads)", m_motor.getClosedLoopReference());
  }
}
