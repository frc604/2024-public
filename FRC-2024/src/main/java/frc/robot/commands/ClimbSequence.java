// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ClimbSequence extends Command {
  private final ElevatorSubsystem m_elevator;
  private final LauncherSubsystem m_launcher;
  private final IntakeSubsystem m_intake;
  private final ClimberSubsystem m_climber;
  private final XboxController m_xboxController;

  private boolean m_backReleased = false;
  private boolean m_startReleased = false;

  private ClimbState m_climbState = ClimbState.CLIMBER_UP;

  private double m_intakeTargetAngle = Constants.Intake.intakeStowAngle;
  private double m_elevatorTargetHeight = Constants.Elevator.stowHeight;
  private double m_launcherTargetAngle = Constants.Launcher.maxAngle;
  private double m_climberTargetPos = Constants.Climber.minPos;

  private final Timer m_pulseTimer = new Timer();

  enum ClimbState {
    RELEASE,
    INTAKE_DOWN,
    CLIMBER_UP,
    CLIMBER_PARTIAL_DOWN,
    ELEVATOR_UP,
    LAUNCHER_ARM_UP,
    CLIMBER_DOWN,
    SCORE,
  }

  public ClimbSequence(
      ElevatorSubsystem elevator,
      LauncherSubsystem launcher,
      IntakeSubsystem intake,
      ClimberSubsystem climber,
      XboxController xboxController) {
    m_elevator = elevator;
    m_launcher = launcher;
    m_intake = intake;
    m_climber = climber;
    m_xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, launcher, intake, climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbState = ClimbState.INTAKE_DOWN;
    m_intake.setRollerVelocity(0.0);
    m_climber.zeroSensorPosition();

    m_backReleased = false;
    m_startReleased = false;

    m_pulseTimer.start();
  }

  private boolean allAtTargetPos() {
    return m_intake.isAtAngle(m_intakeTargetAngle, Math.toRadians(5.0))
        && m_elevator.isAtHeight(m_elevatorTargetHeight, Units.inchesToMeters(1.0))
        && m_launcher.isAtAngle(m_launcherTargetAngle, Math.toRadians(50.0))
        && m_climber.isAtPosition(m_climberTargetPos, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Debounce buttons
    final boolean backPressed = m_xboxController.getBackButton();
    final boolean startPressed = m_xboxController.getStartButton();
    if (!backPressed) {
      m_backReleased = true;
    }
    if (!startPressed) {
      m_startReleased = true;
    }

    boolean goBack;
    if (m_backReleased && backPressed) {
      goBack = true;
      m_backReleased = false;
    } else {
      goBack = false;
    }

    boolean goNext;
    if (m_startReleased && startPressed) {
      goNext = true;
      m_startReleased = false;
    } else {
      goNext = false;
    }

    boolean climberLoaded = false;

    if (goNext) {
      m_pulseTimer.reset();
    }
    final double kPulseFrequency = 0.75; // Hz
    final double kReverseDutyCycle = 0.2 / 1.33;
    final boolean reversePulse =
        (m_pulseTimer.get() * kPulseFrequency) % 1 > (1.0 - kReverseDutyCycle);

    switch (m_climbState) {
      case INTAKE_DOWN:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.stowHeight;
        m_launcherTargetAngle = Constants.Launcher.maxAngle;
        m_launcher.setLaunchVelocity(Constants.Launcher.intakeFromSourceLaunchVelocity);
        m_launcher.setFeedVelocity(0.0);
        m_launcher.setRedirectVelocity(0.0);
        m_climberTargetPos = Constants.Climber.minPos;
        climberLoaded = false;

        if (goBack) {
          m_climbState = ClimbState.RELEASE;
        } else if (goNext && allAtTargetPos()) {
          m_climbState = ClimbState.CLIMBER_UP;
        }
        break;
      case CLIMBER_UP:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.stowHeight;
        m_launcherTargetAngle = Constants.Launcher.maxAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.setFeedVelocity(0.0);
        m_launcher.setRedirectVelocity(0.0);
        m_climberTargetPos = Constants.Climber.extendPos;
        climberLoaded = false;

        if (goBack) {
          m_climbState = ClimbState.INTAKE_DOWN;
        } else if (goNext && allAtTargetPos()) {
          m_climbState = ClimbState.CLIMBER_PARTIAL_DOWN;
        }
        break;
      case CLIMBER_PARTIAL_DOWN:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.stowHeight;
        m_launcherTargetAngle = Constants.Launcher.maxAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
        m_climberTargetPos = Constants.Climber.partialGrabPos;
        climberLoaded = true;

        if (goBack) {
          m_climbState = ClimbState.CLIMBER_UP;
        } else if (goNext && allAtTargetPos()) {
          m_climbState = ClimbState.ELEVATOR_UP;
        }
        break;
      case ELEVATOR_UP:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.maxHeight;
        m_launcherTargetAngle = Constants.Launcher.maxAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.setFeedVelocity(0.0);
        m_launcher.setRedirectVelocity(0.0);
        m_climberTargetPos = Constants.Climber.partialGrabPos;
        climberLoaded = false;

        if (goBack) {
          m_climbState = ClimbState.CLIMBER_PARTIAL_DOWN;
        } else if (allAtTargetPos()) {
          // Auto advance to launcher arm up
          m_climbState = ClimbState.LAUNCHER_ARM_UP;
        }
        break;
      case LAUNCHER_ARM_UP:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.maxHeight;
        m_launcherTargetAngle = Constants.Launcher.trapAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.setFeedVelocity(0.0);
        m_launcher.setRedirectVelocity(0.0);
        m_climberTargetPos = Constants.Climber.partialGrabPos;
        climberLoaded = false;

        if (goBack) {
          m_climbState = ClimbState.ELEVATOR_UP;
        } else if (goNext && allAtTargetPos()) {
          m_climbState = ClimbState.CLIMBER_DOWN;
        }
        break;
      case CLIMBER_DOWN:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.maxHeight;
        m_launcherTargetAngle = Constants.Launcher.trapAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.setFeedVelocity(0.0);
        m_launcher.setRedirectVelocity(0.0);
        m_climberTargetPos = Constants.Climber.minPos;
        climberLoaded = true;

        if (goBack) {
          m_climbState = ClimbState.LAUNCHER_ARM_UP;
        } else if (goNext && allAtTargetPos()) {
          m_climbState = ClimbState.SCORE;
        }
        break;
      case SCORE:
        m_intakeTargetAngle = Constants.Intake.intakeDeployAngle;
        m_elevatorTargetHeight = Constants.Elevator.maxHeight;
        m_launcherTargetAngle = Constants.Launcher.trapAngle;
        m_launcher.setLaunchVelocity(0.0);
        m_launcher.setFeedPower(reversePulse ? 0.3 : -1.0);
        m_launcher.setRedirectPower(reversePulse ? -0.3 : 1.0);
        m_climberTargetPos = Constants.Climber.minPos;
        climberLoaded = true;

        if (goBack) {
          m_climbState = ClimbState.CLIMBER_DOWN;
        }
        break;
      default:
        m_intakeTargetAngle = Constants.Intake.intakeStowAngle;
        m_elevatorTargetHeight = Constants.Elevator.stowHeight;
        m_launcherTargetAngle = Constants.Launcher.maxAngle;
        m_climberTargetPos = Constants.Climber.minPos;
        climberLoaded = false;
        break;
    }

    m_intake.setAngle(m_intakeTargetAngle);
    m_elevator.setHeight(m_elevatorTargetHeight);
    m_launcher.setArmAngleSlow(m_launcherTargetAngle);
    if (climberLoaded) {
      m_climber.setPositionLoaded(m_climberTargetPos);
    } else {
      m_climber.setPositionUnloaded(m_climberTargetPos);
    }

    SmartDashboard.putString("ClimbSequence: State", m_climbState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climbState == ClimbState.RELEASE;
  }
}
