// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TuneShot extends Command {
  private final LauncherSubsystem m_launcher;
  private final LoggedDashboardNumber m_targetAngle =
      new LoggedDashboardNumber("TuneShot: TargetAngle (deg)");

  private final Timer m_readyToShootTimer = new Timer();
  private boolean m_hasDecidedToShoot = false;

  public TuneShot(LauncherSubsystem launcher) {
    m_launcher = launcher;

    m_readyToShootTimer.start();
    m_targetAngle.set(Math.toDegrees(Constants.Launcher.podiumLaunchAngle));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcher.setArmAngle(Math.toRadians(m_targetAngle.get()));
    m_launcher.setLaunchVelocity(Constants.Launcher.launchVelocity);
    m_readyToShootTimer.reset();
    m_hasDecidedToShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_launcher.isAtAngle(
            Math.toRadians(m_targetAngle.get()), Constants.Launcher.launchAngleTolerance)
        || !m_launcher.isAtLaunchVelocity(
            Constants.Launcher.launchVelocity, Constants.Launcher.launchVelocityTolerance)) {
      m_readyToShootTimer.reset();
    }

    if (m_hasDecidedToShoot || m_readyToShootTimer.hasElapsed(0.5)) {
      m_hasDecidedToShoot = true;
      m_launcher.setFeedVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
      m_launcher.setRedirectVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
    } else {
      m_launcher.setFeedVelocity(0.0);
      m_launcher.setRedirectVelocity(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
    m_launcher.setFeedVelocity(0);
    m_launcher.setRedirectVelocity(0);
    m_launcher.setLaunchVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: use a sensor
    return false;
  }
}
