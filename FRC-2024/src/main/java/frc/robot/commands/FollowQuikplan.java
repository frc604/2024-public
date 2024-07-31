// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikPlanAction;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikplanTrajectoryState;
import frc.robot.Constants;
import frc.robot.Fiducials;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.ShotInfo;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;

public class FollowQuikplan extends Command {
  private final boolean kDebugPrint = false;

  private final QuikPlanSwervePartialTrajectoryReader m_reader;
  private final SwerveSubsystem m_swerve;
  private final LauncherSubsystem m_launcher;
  private final IntakeSubsystem m_intake;

  private final Timer m_timer = new Timer();

  private ArrayList<Double> m_times = new ArrayList<>();
  private ArrayList<Double> m_xErrors = new ArrayList<>();
  private ArrayList<Double> m_yErrors = new ArrayList<>();
  private ArrayList<Double> m_thetaErrors = new ArrayList<>();

  private final Timer m_startDelayTimer = new Timer();
  private boolean m_driveReady = false;
  private boolean m_shootReady = false;
  private boolean m_shootEnabled = true;
  private boolean m_deployIntake = true;

  private final EndMode m_endMode;

  public enum EndMode {
    IMMEDIATE,
    WAIT_FOR_PIECE,
    FINISH,
  }

  public FollowQuikplan(
      QuikPlanSwervePartialTrajectoryReader reader,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      LauncherSubsystem launcher) {
    this(reader, swerve, intake, launcher, EndMode.IMMEDIATE);
  }

  public FollowQuikplan(
      QuikPlanSwervePartialTrajectoryReader reader,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      LauncherSubsystem launcher,
      EndMode endMode) {
    m_reader = reader;
    m_swerve = swerve;
    m_launcher = launcher;
    m_intake = intake;
    m_endMode = endMode;

    addRequirements(swerve, intake, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_swerve.resetPose(m_reader.getInitialPose());
    m_timer.start();
    m_timer.reset();
    m_startDelayTimer.start();
    m_startDelayTimer.reset();
    m_driveReady = false;
    m_shootReady = false;
    m_shootEnabled = true;
    m_deployIntake = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setLaunchVelocity(Constants.Launcher.launchVelocity);

    if (!m_shootReady && !m_launcher.isAtAutoStartVelocity()) {
      m_startDelayTimer.reset();
    } else {
      m_shootReady = true;
    }

    if (m_startDelayTimer.get() < 0.05) {
      m_timer.reset();
    } else {
      m_driveReady = true;
    }

    final double curTime = m_timer.get();
    final QuikplanTrajectoryState targetState = m_reader.getState(curTime);
    final var actionEntry = m_reader.getAction(curTime);
    final QuikPlanAction action = actionEntry == null ? null : actionEntry.getValue();

    if (actionEntry != null) {
      switch (action.actionType) {
        case 1:
          m_shootEnabled = true;
          break;
        case 2:
          m_shootEnabled = false;
          break;
        case 3:
          m_deployIntake = true;
          break;
        case 4:
          m_deployIntake = false;
          break;
        case 5:
          m_deployIntake = false;
          m_shootEnabled = false;
          break;
        case 6:
          m_deployIntake = true;
          m_shootEnabled = false;
        default:
          break;
      }
    }

    if (m_driveReady) {
      final Pose2d poseError =
          m_swerve.driveToPose(
              targetState.pose,
              targetState.xVel,
              targetState.yVel,
              targetState.thetaVel,
              Constants.Swerve.autoScrubLimit);
      if (kDebugPrint) {
        m_times.add(m_timer.get());
        m_xErrors.add(poseError.getX());
        m_yErrors.add(poseError.getY());
        m_thetaErrors.add(poseError.getRotation().getRadians());
      }
    }

    m_intake.setAngle(
        m_deployIntake ? Constants.Intake.intakeDeployAngle : Constants.Intake.intakeStowAngle);

    final var alliance = DriverStation.getAlliance();
    final Pose3d tagPose =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? Fiducials.aprilTagFiducials[6].getPose()
            : Fiducials.aprilTagFiducials[3].getPose();
    final Translation3d goalPosition =
        tagPose.plus(Constants.FieldPoses.speakerCenterTagToGoalOffset).getTranslation();
    final ShotInfo shotInfo =
        ShotCalculator.computeSpeakerShotInfo(
            0.0,
            goalPosition,
            m_swerve.getPose(),
            new Transform2d(
                new Translation2d(targetState.xVel, targetState.yVel)
                    .rotateBy(m_swerve.getPose().getRotation().unaryMinus()),
                m_swerve.getVelocity().getRotation()),
            new Transform2d(),
            // TODO: Get acceleration from quikplan
            // new Transform2d(
            //     new Translation2d(m_xVelocityState.velocity, m_yVelocityState.velocity)
            //         .rotateBy(m_swerve.getPose().getRotation().unaryMinus()),
            //     new Rotation2d()),
            m_swerve.getFieldRelativeVelocity());

    final boolean elevationOK =
        m_launcher.isAtAngle(shotInfo.elevation, Constants.Launcher.launchAngleTolerance);
    final boolean inWing =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? m_swerve.getPose().getX() < Constants.FieldPoses.blueWingLine
            : m_swerve.getPose().getX()
                > 2.0 * Constants.FieldPoses.midline - Constants.FieldPoses.blueWingLine;

    if (m_launcher.hasPiece()) {
      m_intake.setRollerVelocity(0.0);
      m_launcher.setArmAngle(shotInfo.elevation);
      if (elevationOK && m_shootReady && m_shootEnabled && inWing) {
        m_launcher.setFeedVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
        m_launcher.setRedirectVelocity(Constants.Launcher.scoreSpeakerFeedVelocity);
      } else {
        m_launcher.setFeedPower(0.0);
        m_launcher.setRedirectPower(0.0);
      }
    } else {
      // Feed from intake
      m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
      m_launcher.setArmAngle(Constants.Launcher.intakeAngle);
      m_launcher.setFeedVelocity(Constants.Launcher.intakeFeedVelocity);
      m_launcher.setRedirectVelocity(Constants.Launcher.intakeFeedVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_swerve.stop();
    if (kDebugPrint) {
      for (int i = 0; i < m_times.size(); i++) {
        System.out.println(
            m_times.get(i)
                + ","
                + m_xErrors.get(i)
                + ","
                + m_yErrors.get(i)
                + ","
                + m_thetaErrors.get(i));
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (m_endMode) {
      case WAIT_FOR_PIECE:
        {
          // Wait up to kEndTimeExtension for a note to be detected in the intake.
          final double kEndTimeExtension = 1.0; // s
          return m_timer.hasElapsed(m_reader.getTotalTime() + kEndTimeExtension)
              || (m_timer.hasElapsed(m_reader.getTotalTime()) && m_intake.recentlyHadPiece());
        }
      case FINISH:
        {
          final double kEndTimeExtension = 0.5; // s
          return m_timer.hasElapsed(m_reader.getTotalTime() + kEndTimeExtension);
        }
      case IMMEDIATE:
      default:
        {
          return m_timer.hasElapsed(m_reader.getTotalTime());
        }
    }
  }
}
