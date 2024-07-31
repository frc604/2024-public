// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class EjectIntake extends Command {
  private final IntakeSubsystem m_intake;
  private final LauncherSubsystem m_launcher;

  public EjectIntake(IntakeSubsystem intake, LauncherSubsystem launcher) {
    m_intake = intake;
    m_launcher = launcher;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setRollerVelocity(-50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setFeedVelocity(-Constants.Launcher.scoreAmpFeedVelocity);
    m_launcher.setRedirectVelocity(-Constants.Launcher.scoreAmpFeedVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
