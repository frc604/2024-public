// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class StowIntakeAndConditionallyFeedLauncher extends Command {
  private final IntakeSubsystem m_intake;
  private final BooleanSupplier m_elevatorReadyForIntake;
  private final BooleanSupplier m_launcherReadyForIntake;
  private final BooleanSupplier m_launcherHasPiece;

  public StowIntakeAndConditionallyFeedLauncher(
      IntakeSubsystem intake,
      BooleanSupplier elevatorReadyForIntake,
      BooleanSupplier launcherReadyForIntake,
      BooleanSupplier launcherHasPiece) {
    m_intake = intake;
    m_elevatorReadyForIntake = elevatorReadyForIntake;
    m_launcherReadyForIntake = launcherReadyForIntake;
    m_launcherHasPiece = launcherHasPiece;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setAngle(Constants.Intake.intakeStowAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.recentlyHadPiece() && m_launcherHasPiece.getAsBoolean()) {
      // If the intake ever has a piece when the launcher does, eject immediately.
      m_intake.setRollerVelocity(-Constants.Intake.intakeRollerVelocity);
    } else if (m_intake.hasPiece()
        && m_elevatorReadyForIntake.getAsBoolean()
        && m_launcherReadyForIntake.getAsBoolean()) {
      m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
    } else {
      m_intake.setRollerVelocity(0);
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
