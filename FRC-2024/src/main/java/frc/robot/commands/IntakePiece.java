// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class IntakePiece extends Command {
  private final IntakeSubsystem m_intake;
  private final LauncherSubsystem m_launcher;
  private final ElevatorSubsystem m_elevator;

  public IntakePiece(
      IntakeSubsystem intakeSubsystem,
      LauncherSubsystem launcherSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    m_intake = intakeSubsystem;
    m_launcher = launcherSubsystem;
    m_elevator = elevatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, launcherSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
    m_launcher.setFeedVelocity(Constants.Launcher.intakeFeedVelocity);
    m_launcher.setRedirectVelocity(Constants.Launcher.intakeFeedVelocity);
    m_intake.setAngle(Constants.Intake.intakeDeployAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_launcher.isAtAngle(
            Constants.Launcher.intakeAngle, Constants.Launcher.intakeAngleTolerance)
        && m_elevator.isAtHeight(Constants.Elevator.stowHeight, Constants.Elevator.stowTolerance)) {
      m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
    } else {
      m_intake.setRollerVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setAngle(Constants.Intake.intakeStowAngle);
    m_intake.setRollerVelocity(0);
    // m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
    m_launcher.setFeedPower(0.0);
    m_launcher.setRedirectPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasPiece() || m_launcher.hasPiece();
  }
}
