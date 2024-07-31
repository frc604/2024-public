// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.math.MathUtils;
import frc.quixlib.swerve.QuixSwerveTeleopControl;
import frc.robot.Constants;
import frc.robot.Fiducials;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.ShotInfo;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class AutoAim extends Command {
  private final SwerveSubsystem m_swerve;
  private final XboxController m_xboxController;
  private final QuixSwerveTeleopControl m_teleopControl;
  private final LauncherSubsystem m_launcher;
  private final ElevatorSubsystem m_elevator;

  private final TrapezoidProfile m_xVelocityTrap;
  private State m_xVelocityState = new State();
  private State m_xTargetVelocityState = new State();
  private final TrapezoidProfile m_yVelocityTrap;
  private State m_yVelocityState = new State();
  private State m_yTargetVelocityState = new State();
  private final Timer m_timer = new Timer();

  private final Timer m_readyToShootTimer = new Timer();
  private boolean m_hasDecidedToShoot = false;

  private final TrapezoidProfile m_yawTrap =
      new TrapezoidProfile(new Constraints(2.0 * Math.PI, 4.0 * Math.PI));
  private State m_yawState = new State();
  private final PIDController m_yawController = new PIDController(2.5, 0.0, 0.01);

  public AutoAim(
      SwerveSubsystem swerve,
      XboxController xboxController,
      LauncherSubsystem launcher,
      ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, launcher, elevator);

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
    m_launcher = launcher;
    m_elevator = elevator;

    m_xVelocityTrap = new TrapezoidProfile(new Constraints(10.0, 80.0)); // Max acceleration / jerk
    m_yVelocityTrap = new TrapezoidProfile(new Constraints(10.0, 80.0)); // Max acceleration / jerk
    m_timer.start();

    m_readyToShootTimer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();

    final var fieldVel = m_swerve.getFieldRelativeVelocity();
    m_xVelocityState.position = fieldVel.getX(); // Position is actually velocity
    m_xVelocityState.velocity = 0.0;
    m_yVelocityState.position = fieldVel.getY(); // Position is actually velocity
    m_yVelocityState.velocity = 0.0;

    m_yawState.position = m_swerve.getPose().getRotation().getRadians();
    m_yawState.velocity = fieldVel.getRotation().getRadians();

    m_readyToShootTimer.reset();
    m_hasDecidedToShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double dt = m_timer.get();
    m_timer.reset();

    final boolean isRightTriggerPressed = m_xboxController.getRightTriggerAxis() > 0.2;
    final boolean isDpadSidePressed =
        m_xboxController.getPOV() == 90 || m_xboxController.getPOV() == 270;
    final boolean shootButtonPressed = isRightTriggerPressed || isDpadSidePressed;

    final double elevatorTargetHeight =
        isDpadSidePressed ? Constants.Elevator.maxHeight : Constants.Elevator.stowHeight;
    m_elevator.setHeight(elevatorTargetHeight);

    final var alliance = DriverStation.getAlliance();
    final var targetDriveVelocities =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? m_teleopControl.getDriveVelocitiesFromJoysticks(
                -m_xboxController.getLeftY(),
                -m_xboxController.getLeftX(),
                -m_xboxController.getRightX())
            : m_teleopControl.getDriveVelocitiesFromJoysticks(
                m_xboxController.getLeftY(),
                m_xboxController.getLeftX(),
                -m_xboxController.getRightX());

    final double kBuffer = 2.0; // m
    final boolean isSpeakerShot =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? m_swerve.getPose().getX() < Constants.FieldPoses.blueWingLine + kBuffer
            : m_swerve.getPose().getX()
                > 2.0 * Constants.FieldPoses.midline - Constants.FieldPoses.blueWingLine - kBuffer;

    final var fieldVel = m_swerve.getFieldRelativeVelocity();
    if (!shootButtonPressed) {
      // Constantly reset profiled velocity state when not ready to shoot.
      m_xVelocityState.position = fieldVel.getX(); // Position is actually velocity
      m_xVelocityState.velocity = 0.0;
      m_yVelocityState.position = fieldVel.getY(); // Position is actually velocity
      m_yVelocityState.velocity = 0.0;
    }

    // Velocity profile the current velocity to the target velocity.
    // Remember, State position and velocity are actually velocity and acceleration respectively.
    final double kDriveScalar = 0.8;
    m_xTargetVelocityState.position = targetDriveVelocities.xVel * kDriveScalar;
    m_xVelocityState = m_xVelocityTrap.calculate(dt, m_xVelocityState, m_xTargetVelocityState);
    m_yTargetVelocityState.position = targetDriveVelocities.yVel * kDriveScalar;
    m_yVelocityState = m_yVelocityTrap.calculate(dt, m_yVelocityState, m_yTargetVelocityState);

    final Pose3d tagPose =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? Fiducials.aprilTagFiducials[6].getPose()
            : Fiducials.aprilTagFiducials[3].getPose();
    final Pose3d ampPose =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? Fiducials.aprilTagFiducials[5].getPose()
            : Fiducials.aprilTagFiducials[4].getPose();
    final Translation3d goalPosition =
        isSpeakerShot
            ? tagPose.plus(Constants.FieldPoses.speakerCenterTagToGoalOffset).getTranslation()
            : ampPose.plus(Constants.FieldPoses.ampTagToFeedShotOffset).getTranslation();

    final Pose2d curPose = m_swerve.getPose();

    final double startTimestampUs = RobotController.getFPGATime();
    final ShotInfo shotInfo =
        isSpeakerShot
            ? ShotCalculator.computeSpeakerShotInfo(
                elevatorTargetHeight * Math.sin(Math.toRadians(75)),
                goalPosition,
                curPose,
                new Transform2d(
                    new Translation2d(m_xVelocityState.position, m_yVelocityState.position)
                        .rotateBy(curPose.getRotation().unaryMinus()),
                    m_swerve.getVelocity().getRotation()),
                new Transform2d(
                    new Translation2d(m_xVelocityState.velocity, m_yVelocityState.velocity)
                        .rotateBy(curPose.getRotation().unaryMinus()),
                    new Rotation2d()),
                new Transform2d(
                    new Translation2d(m_xVelocityState.position, m_yVelocityState.position),
                    new Rotation2d()))
            : ShotCalculator.computeFeedShotInfo(
                goalPosition,
                curPose,
                new Transform2d(
                    new Translation2d(m_xVelocityState.position, m_yVelocityState.position)
                        .rotateBy(curPose.getRotation().unaryMinus()),
                    m_swerve.getVelocity().getRotation()),
                new Transform2d(
                    new Translation2d(m_xVelocityState.velocity, m_yVelocityState.velocity)
                        .rotateBy(curPose.getRotation().unaryMinus()),
                    new Rotation2d()),
                new Transform2d(
                    new Translation2d(m_xVelocityState.position, m_yVelocityState.position),
                    new Rotation2d()));
    final double endTimestampUs = RobotController.getFPGATime();
    Logger.recordOutput("AutoAim/ShotCalculatorMs", (endTimestampUs - startTimestampUs) / 1000.0);

    // Yaw contoller
    m_yawState =
        m_yawTrap.calculate(
            LoggedRobot.defaultPeriodSecs,
            m_yawState,
            new State(
                MathUtils.placeInScope(shotInfo.yaw, m_yawState.position), shotInfo.yawVelocity));
    final double currentTheta =
        MathUtils.placeInScope(curPose.getRotation().getRadians(), m_yawState.position);
    final double kYawFFScalar = 0.75; // Raw FF is overly aggressive for some reason
    final double thetaVel =
        m_yawController.calculate(currentTheta, m_yawState.position)
            + m_yawState.velocity * kYawFFScalar;
    m_swerve.driveClosedLoop(
        shootButtonPressed
            ? m_xVelocityState.position // Position is actually velocity
            : targetDriveVelocities.xVel,
        shootButtonPressed
            ? m_yVelocityState.position // Position is actually velocity
            : targetDriveVelocities.yVel,
        thetaVel,
        true,
        Constants.Swerve.teleopScrubLimit);

    Logger.recordOutput("AutoAim/Theta (degrees)", Math.toDegrees(currentTheta));
    Logger.recordOutput("AutoAim/Theta target (degrees)", Math.toDegrees(shotInfo.yaw));
    Logger.recordOutput(
        "AutoAim/Theta trapezoid target (degrees)", Math.toDegrees(m_yawState.position));
    Logger.recordOutput(
        "AutoAim/Theta error (degrees)", Math.toDegrees(shotInfo.yaw - currentTheta));
    Logger.recordOutput("AutoAim/Shot Velocity (meters per second)", shotInfo.shotVelocity);

    final double radsPerSec = m_launcher.setLinearLaunchVelocity(shotInfo.shotVelocity);
    m_launcher.setArmAngle(
        m_launcher.hasPiece() ? shotInfo.elevation : Constants.Launcher.intakeAngle);

    // Dynamic yaw tolerance.
    final Translation2d goalLeftEdge =
        new Translation2d(
            goalPosition.getX(), goalPosition.getY() + 0.5 * Constants.FieldPoses.speakerGoalWidth);
    final Translation2d goalRightEdge =
        new Translation2d(
            goalPosition.getX(), goalPosition.getY() - 0.5 * Constants.FieldPoses.speakerGoalWidth);
    final double goalLeftAngle =
        Math.atan2(goalLeftEdge.getY() - curPose.getY(), goalLeftEdge.getX() - curPose.getX());
    final double goalRightAngle =
        Math.atan2(goalRightEdge.getY() - curPose.getY(), goalRightEdge.getX() - curPose.getX());
    final double goalWidthAngle =
        Math.abs(MathUtils.constrainAngleNegPiToPi(goalLeftAngle - goalRightAngle));
    final double kWidthScalar = 0.5;
    final boolean yawOK =
        Math.abs(m_swerve.getPose().getRotation().minus(new Rotation2d(shotInfo.yaw)).getRadians())
            < (isSpeakerShot ? goalWidthAngle * kWidthScalar : Math.toRadians(5.0));

    // Dynamic elevation tolerance. Ranges from +/- 0.5-3 degrees from 8-0 m away from the goal.
    final Translation2d goalPos2d = new Translation2d(goalPosition.getX(), goalPosition.getY());
    final Translation2d goalVector = goalPos2d.minus(curPose.getTranslation());
    final double goalDistance = goalVector.getNorm();
    final double elevationAngleToleranceDeg =
        MathUtils.clamp(3.0 - (2.5 / 8.0) * goalDistance, 1.0, 3.0);
    final boolean elevationOK =
        m_launcher.isAtAngle(shotInfo.elevation, Math.toRadians(elevationAngleToleranceDeg));

    // Dynamic velocity tolerance. Ranges from +/- 10-50 rad/s from 8-0 m away from the goal.
    final double launchVelocityTolerance =
        isSpeakerShot ? MathUtils.clamp(50.0 - (40.0 / 8.0) * goalDistance, 10.0, 50.0) : 30.0;
    final boolean launchOK = m_launcher.isAtLaunchVelocity(radsPerSec, launchVelocityTolerance);
    final boolean elevatorOK =
        m_elevator.isAtHeight(elevatorTargetHeight, Units.inchesToMeters(1.0));

    if (!shotInfo.feasibleShot || !yawOK || !elevationOK || !launchOK || !elevatorOK) {
      m_readyToShootTimer.reset();
    }

    if (m_hasDecidedToShoot || (shootButtonPressed && m_readyToShootTimer.hasElapsed(0.1))) {
      m_hasDecidedToShoot = true;
      m_launcher.setFeedVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
      m_launcher.setRedirectVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
    } else {
      m_launcher.setFeedVelocity(
          m_launcher.hasPiece() ? 0.0 : Constants.Launcher.intakeFeedVelocity);
      m_launcher.setRedirectVelocity(
          m_launcher.hasPiece() ? 0.0 : Constants.Launcher.intakeFeedVelocity);
    }

    Logger.recordOutput("AutoAim/Shot Feasible", shotInfo.feasibleShot);
    Logger.recordOutput("AutoAim/Yaw OK", yawOK);
    Logger.recordOutput("AutoAim/Elevation OK", elevationOK);
    Logger.recordOutput("AutoAim/Launch OK", launchOK);
    Logger.recordOutput("AutoAim/Elevator OK", elevatorOK);
    Logger.recordOutput("AutoAim/Distance To Goal (ft)", Units.metersToFeet(goalDistance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setLaunchVelocity(0.0);
    m_launcher.setFeedVelocity(0.0);
    m_launcher.setRedirectVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
