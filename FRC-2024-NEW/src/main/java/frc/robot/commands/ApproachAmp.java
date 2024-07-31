package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.planning.SimpleSwerveTrajectory;
import frc.quixlib.planning.SwerveTrajectoryState;
import frc.robot.Constants;
import frc.robot.Fiducials;
import frc.robot.subsystems.SwerveSubsystem;

public class ApproachAmp extends Command {
  private final SwerveSubsystem m_swerve;
  private Pose2d m_targetPose;
  private SimpleSwerveTrajectory m_trajectory;
  private Timer m_trajectoryTimer = new Timer();

  public ApproachAmp(SwerveSubsystem swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetPose =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue
            ? Fiducials.aprilTagFiducials[5].getPose().toPose2d()
            : Fiducials.aprilTagFiducials[4].getPose().toPose2d();
    m_trajectory =
        new SimpleSwerveTrajectory(
            m_swerve.getPose(),
            m_swerve.getFieldSpeeds(),
            m_targetPose.transformBy(Constants.FieldPoses.ampApproachOffsetTransform),
            new Constraints(
                Constants.Swerve.maxDriveSpeed * 0.25, Constants.Swerve.maxDriveAcceleration),
            new Constraints(
                Constants.Swerve.maxAngularVelocity * 0.25,
                Constants.Swerve.maxAngularAcceleration));
    m_trajectoryTimer.start();
    m_trajectoryTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double t = m_trajectoryTimer.get();
    SwerveTrajectoryState m_trajectoryState = m_trajectory.getState(t);
    m_swerve.driveToPose(
        m_trajectoryState.pose,
        m_trajectoryState.vx,
        m_trajectoryState.vy,
        m_trajectoryState.vTheta,
        Constants.Swerve.teleopScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_trajectoryTimer.get() > m_trajectory.totalTime();
  }
}
