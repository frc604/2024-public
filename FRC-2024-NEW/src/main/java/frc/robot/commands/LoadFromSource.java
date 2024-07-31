package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LoadFromSource extends Command {
  private final LauncherSubsystem m_launcher;
  private final ElevatorSubsystem m_elevator;
  private boolean m_beamBreakTriggeredOnce = false;

  public LoadFromSource(LauncherSubsystem launcher, ElevatorSubsystem elevator) {
    m_launcher = launcher;
    m_elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcher.setArmAngle(Constants.Launcher.intakeFromSourceAngle);
    m_launcher.setLaunchVelocity(Constants.Launcher.intakeFromSourceLaunchVelocity);
    m_launcher.setFeedPower(Constants.Launcher.intakeFromSourceFeedPower);
    m_launcher.setRedirectPower(Constants.Launcher.intakeFromSourceFeedPower);
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_beamBreakTriggeredOnce = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_launcher.hasPiece()) {
      m_beamBreakTriggeredOnce = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
    // m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
    m_launcher.setFeedPower(0.0);
    m_launcher.setRedirectPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beamBreakTriggeredOnce && !m_launcher.hasPiece();
  }
}
