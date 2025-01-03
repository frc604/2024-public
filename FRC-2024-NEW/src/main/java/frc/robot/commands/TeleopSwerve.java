// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuixSwerveTeleopControl;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private final SwerveSubsystem m_swerve;
  private final XboxController m_xboxController;
  private final QuixSwerveTeleopControl m_teleopControl;

  public TeleopSwerve(SwerveSubsystem swerve, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    m_swerve = swerve;
    m_xboxController = xboxController;
    m_teleopControl =
        new QuixSwerveTeleopControl(
            Constants.Swerve.linearSlewRate,
            Constants.Swerve.angularSlewRate,
            Constants.Swerve.stickDeadband,
            Constants.Swerve.maxDriveSpeed,
            Constants.Swerve.maxAngularVelocity,
            true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Map joystick axes to velocities.
    // Joystick +x/+y don't correspond to field +x/+y, so map them accordingly.
    final var alliance = DriverStation.getAlliance();
    final var driveVelocities =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? m_teleopControl.getDriveVelocitiesFromJoysticks(
                -m_xboxController.getLeftY(),
                -m_xboxController.getLeftX(),
                -m_xboxController.getRightX())
            : m_teleopControl.getDriveVelocitiesFromJoysticks(
                m_xboxController.getLeftY(),
                m_xboxController.getLeftX(),
                -m_xboxController.getRightX());
    m_swerve.driveOpenLoop(
        driveVelocities.xVel,
        driveVelocities.yVel,
        driveVelocities.thetaVel,
        true,
        Constants.Swerve.teleopScrubLimit);
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
