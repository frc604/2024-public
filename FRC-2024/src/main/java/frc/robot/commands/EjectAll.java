// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class EjectAll extends Command {
  private final IntakeSubsystem m_intake;
  private final LauncherSubsystem m_launcher;
  private final ElevatorSubsystem m_elevator;

  public EjectAll(
      IntakeSubsystem intakeSubsystem, LauncherSubsystem launcher, ElevatorSubsystem elevator) {
    m_intake = intakeSubsystem;
    m_launcher = launcher;
    m_elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, launcher, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setRollerVelocity(-50);
    m_launcher.setArmAngle(Constants.Launcher.scoreAmpArmAngle);
    m_elevator.setHeight(Constants.Elevator.scoreAmpHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_launcher.isAtAngle(
        Constants.Launcher.scoreAmpArmAngle, Constants.Launcher.scoreAmpArmAngleTolerance)) {
      m_launcher.setFeedVelocity(-Constants.Launcher.scoreAmpFeedVelocity);
      m_launcher.setRedirectVelocity(-Constants.Launcher.scoreAmpFeedVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
    m_intake.setRollerVelocity(0);
    m_launcher.setFeedVelocity(0);
    m_launcher.setRedirectVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
