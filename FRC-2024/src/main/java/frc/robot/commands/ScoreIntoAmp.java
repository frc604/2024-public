package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ScoreIntoAmp extends Command {
  private final LauncherSubsystem m_launcher;
  private final ElevatorSubsystem m_elevator;
  private Timer m_ejectTimer = new Timer();

  public ScoreIntoAmp(LauncherSubsystem launcher, ElevatorSubsystem elevator) {
    m_launcher = launcher;
    m_elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setHeight(Constants.Elevator.scoreAmpHeight);
    m_launcher.setArmAngle(Constants.Launcher.scoreAmpArmAngle);
    m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);

    m_ejectTimer.start();
    m_ejectTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_launcher.isAtAngle(
            Constants.Launcher.scoreAmpArmAngle, Constants.Launcher.scoreAmpArmAngleTolerance)
        && m_elevator.isAtHeight(
            Constants.Elevator.scoreAmpHeight, Constants.Elevator.scoreAmpTolerance)) {
      m_launcher.setFeedVelocity(-Constants.Launcher.scoreAmpFeedVelocity);
      m_launcher.setRedirectVelocity(Constants.Launcher.scoreAmpFeedVelocity);
    } else {
      m_ejectTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
    m_launcher.setFeedVelocity(0);
    m_launcher.setRedirectVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ejectTimer.get() > 0.35 && !m_launcher.hasPiece();
  }
}
