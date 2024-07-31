// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;
import java.util.function.BooleanSupplier;

public class ReceivePieceFromIntake extends Command {
  private final LauncherSubsystem m_launcher;
  private final BooleanSupplier m_intakeHasPiece;

  public ReceivePieceFromIntake(LauncherSubsystem launcher, BooleanSupplier intakeHasPiece) {
    m_launcher = launcher;
    m_intakeHasPiece = intakeHasPiece;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setLaunchVelocity(0.0);
    if (!m_launcher.hasPiece()) {
      // if (!m_launcher.hasPiece() && m_intakeHasPiece.getAsBoolean()) {
      m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
      m_launcher.setFeedVelocity(Constants.Launcher.intakeFeedVelocity);
      m_launcher.setRedirectVelocity(Constants.Launcher.intakeFeedVelocity);
    } else {
      m_launcher.setArmAngle(Constants.Launcher.podiumLaunchAngle);
      // m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
      m_launcher.setFeedPower(0.0);
      m_launcher.setRedirectPower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
