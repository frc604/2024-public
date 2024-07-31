// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.devices.QuixCANBus;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.vision.PhotonVisionCamera;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.EjectAll;
import frc.robot.commands.EjectIntake;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.LoadFromSource;
import frc.robot.commands.ReceivePieceFromIntake;
import frc.robot.commands.ScoreFromPodium;
import frc.robot.commands.ScoreFromSubwoofer;
import frc.robot.commands.ScoreIntoAmp;
import frc.robot.commands.StowIntakeAndConditionallyFeedLauncher;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.Auto1456;
import frc.robot.commands.autos.Auto1456red;
import frc.robot.commands.autos.Auto1546;
import frc.robot.commands.autos.Auto1546red;
import frc.robot.commands.autos.Auto231456;
import frc.robot.commands.autos.Auto24531;
import frc.robot.commands.autos.Auto24531red;
import frc.robot.commands.autos.Auto25431;
import frc.robot.commands.autos.Auto25431red;
import frc.robot.commands.autos.Auto678;
import frc.robot.commands.autos.Auto768drop;
import frc.robot.commands.autos.Auto786drop;
import frc.robot.commands.autos.Auto786fast;
import frc.robot.commands.autos.Auto876;
import frc.robot.commands.autos.Auto876drop;
import frc.robot.commands.autos.Auto876fast;
import frc.robot.commands.autos.AutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Controllers
  public final XboxController driverXbox = new XboxController(0);

  // ABXY Buttons
  private final JoystickButton buttonA =
      new JoystickButton(driverXbox, XboxController.Button.kA.value);
  private final JoystickButton buttonB =
      new JoystickButton(driverXbox, XboxController.Button.kB.value);
  private final JoystickButton buttonX =
      new JoystickButton(driverXbox, XboxController.Button.kX.value);
  private final JoystickButton buttonY =
      new JoystickButton(driverXbox, XboxController.Button.kY.value);

  // Bumpers
  private final JoystickButton bumperLeft =
      new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton bumperRight =
      new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);

  // Triggers
  private final Trigger leftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.2);
  private final Trigger rightTrigger = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.2);

  // D-pad Buttons
  private final Trigger dPadUp = new Trigger(() -> driverXbox.getPOV() == 0);
  private final Trigger dPadRight = new Trigger(() -> driverXbox.getPOV() == 90);
  private final Trigger dPadDown = new Trigger(() -> driverXbox.getPOV() == 180);
  private final Trigger dPadLeft = new Trigger(() -> driverXbox.getPOV() == 270);

  // Menu Buttons
  private final JoystickButton buttonStart =
      new JoystickButton(driverXbox, XboxController.Button.kStart.value);
  private final JoystickButton buttonBack =
      new JoystickButton(driverXbox, XboxController.Button.kBack.value);

  // CANbuses
  private final QuixCANBus rioBus = new QuixCANBus();
  private final QuixCANBus canivoreBus = new QuixCANBus(Constants.kCanivoreName);

  // Sensors
  private final QuixPigeon2 imu =
      new QuixPigeon2(
          Constants.IMU.pigeonID,
          QuixPigeon2.makeDefaultConfig().setGyroTrimZ(Constants.IMU.gyroTrimZ));
  private final ArrayList<QuixVisionCamera> cameras =
      new ArrayList<>(
          Arrays.asList(
              new PhotonVisionCamera(
                  "frontleft",
                  Constants.Cameras.LeftCam.robotToCameraT,
                  Constants.Cameras.LeftCam.fovWidth,
                  Constants.Cameras.LeftCam.fovHeight,
                  Constants.Cameras.LeftCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "frontright",
                  Constants.Cameras.RightCam.robotToCameraT,
                  Constants.Cameras.RightCam.fovWidth,
                  Constants.Cameras.RightCam.fovHeight,
                  Constants.Cameras.RightCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "backleft",
                  Constants.Cameras.BackLeft.robotToCameraT,
                  Constants.Cameras.BackLeft.fovWidth,
                  Constants.Cameras.BackLeft.fovHeight,
                  Constants.Cameras.BackLeft.pipelineConfigs),
              new PhotonVisionCamera(
                  "backright",
                  Constants.Cameras.BackRight.robotToCameraT,
                  Constants.Cameras.BackRight.fovWidth,
                  Constants.Cameras.BackRight.fovHeight,
                  Constants.Cameras.BackRight.pipelineConfigs)));
  private final ArrayList<QuixVisionCamera> localizationCameras =
      new ArrayList<QuixVisionCamera>(Arrays.asList(cameras.get(0), cameras.get(1)));

  // Simulation & viz
  private final QuixVisionSim visionSim =
      new QuixVisionSim(cameras, Fiducials.aprilTagFiducials, Fiducials.centerNotes);
  private final Field2d fieldViz = visionSim.getSimField();
  private final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(54.0), Units.inchesToMeters(50.0), 100.0);

  private final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));
  // Intake viz
  private final Link2d intakeArmViz =
      robotViz.addLink(
          new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
  private final Link2d intakeRollerViz =
      intakeArmViz.addLink(
          new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));
  // Elevator viz
  private final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              10.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              5,
              Color.kLightGreen));
  // Launcher viz
  private final Link2d launcherArmViz =
      elevatorCarriageViz.addLink(
          new Link2d(robotViz, "Launcher Arm", Constants.Viz.launcherArmLength, 10, Color.kRed));
  private final Link2d lancherTopWheelViz =
      launcherArmViz.addLink(
          new Link2d(robotViz, "Launcher Top Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));
  private final Link2d lancherBottomWheelViz =
      launcherArmViz.addLink(
          new Link2d(
              robotViz, "Launcher Bottom Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));
  private final Link2d launcherFeedRollerViz =
      launcherArmViz.addLink(
          new Link2d(
              robotViz, "Launcher Feed Roller", Units.inchesToMeters(1.0), 10, Color.kCoral));
  private final Link2d launcherRedirectRollerViz =
      launcherArmViz.addLink(
          new Link2d(
              robotViz, "Launcher Redirect Roller", Units.inchesToMeters(1.0), 10, Color.kCoral));

  // Subsystems
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(imu, localizationCameras, visionSim, fieldViz);
  private final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorCarriageViz);
  private final LauncherSubsystem launcher =
      new LauncherSubsystem(
          launcherArmViz,
          lancherTopWheelViz,
          lancherBottomWheelViz,
          launcherFeedRollerViz,
          launcherRedirectRollerViz);
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // Misc.
  private final LoggedDashboardChooser<AutoCommand> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private AutoCommand lastSelectedAuto = null;

  public RobotContainer() {
    // Add autos
    autoChooser.addDefaultOption("24531", new Auto24531(swerve, intake, launcher));
    autoChooser.addOption("25431", new Auto25431(swerve, intake, launcher));
    autoChooser.addOption("24531red", new Auto24531red(swerve, intake, launcher));
    autoChooser.addOption("25431red", new Auto25431red(swerve, intake, launcher));
    autoChooser.addOption("876", new Auto876(swerve, intake, launcher));
    autoChooser.addOption("876drop", new Auto876drop(swerve, intake, launcher));
    autoChooser.addOption("87drop", new Auto876drop(swerve, intake, launcher, true));
    autoChooser.addOption("876fast", new Auto876fast(swerve, intake, launcher));
    autoChooser.addOption("786drop", new Auto786drop(swerve, intake, launcher));
    autoChooser.addOption("78drop", new Auto786drop(swerve, intake, launcher, true));
    autoChooser.addOption("786fast", new Auto786fast(swerve, intake, launcher));
    autoChooser.addOption("768drop", new Auto768drop(swerve, intake, launcher));
    autoChooser.addOption("76drop", new Auto768drop(swerve, intake, launcher, true));
    autoChooser.addOption("678", new Auto678(swerve, intake, launcher));
    autoChooser.addOption("1456", new Auto1456(swerve, intake, launcher));
    autoChooser.addOption("1546", new Auto1546(swerve, intake, launcher));
    autoChooser.addOption("1456red", new Auto1456red(swerve, intake, launcher));
    autoChooser.addOption("1546red", new Auto1546red(swerve, intake, launcher));
    autoChooser.addOption("231456", new Auto231456(swerve, intake, launcher));

    // Default commands
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driverXbox));
    intake.setDefaultCommand(
        new StowIntakeAndConditionallyFeedLauncher(
            intake, elevator::readyForIntake, launcher::readyForIntake, launcher::hasPiece));
    launcher.setDefaultCommand(new ReceivePieceFromIntake(launcher, intake::hasPiece));
    elevator.setDefaultCommand(
        new InstantCommand(
            () -> {
              elevator.setHeight(Constants.Elevator.stowHeight);
            },
            elevator));
    climber.setDefaultCommand(new InstantCommand(climber::biasDown, climber));

    configureBindings();
  }

  private void configureBindings() {
    // Gyro reset
    buttonY.onTrue(
        new InstantCommand(
            () -> {
              final var alliance = DriverStation.getAlliance();
              swerve.setContinuousYaw(
                  alliance.isPresent() && alliance.get() == Alliance.Blue ? 0.0 : Math.PI);
            },
            swerve));

    leftTrigger.whileTrue(new IntakePiece(intake, launcher, elevator));
    buttonA.whileTrue(new EjectIntake(intake, launcher));
    bumperRight.whileTrue(new AutoAim(swerve, driverXbox, launcher, elevator));
    buttonX.whileTrue(new EjectAll(intake, launcher, elevator));

    // what to move NoteVisionTest to?
    // bumperRight.whileTrue(new NoteVisionTest(cameras.get(2), cameras.get(3)));
    bumperLeft.whileTrue(new LoadFromSource(launcher, elevator));

    // leftTrigger.whileTrue(
    //     new SequentialCommandGroup(
    //         new ApproachAmp(swerve), new AutoScoreIntoAmpCommand(launcher, elevator, swerve)));
    buttonB.whileTrue(new ScoreIntoAmp(launcher, elevator));

    // Climb
    buttonStart.onTrue(
        new ClimbSequence(elevator, launcher, intake, climber, driverXbox)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    // buttonBack.onTrue(new RevertClimbState(elevator, launcher));

    // Emergency buttons
    dPadUp.whileTrue(new ScoreFromPodium(launcher));
    dPadDown.whileTrue(new ScoreFromSubwoofer(launcher));
  }

  public Command getAutonomousCommand() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      return null;
    }
    return selectedAuto.getCommand();
  }

  public void disabledInit() {
    intake.disabledInit();
    launcher.disabledInit();
  }

  public void disabledExit() {
    intake.disabledExit();
    launcher.disabledExit();
  }

  /** Runs when the robot is disabled. */
  public void disabledPeriodic() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      // Clear poses
      fieldViz.getObject("traj").setPoses();
      LoggerHelper.recordPose2dList("AutoTraj", new ArrayList<Pose2d>());
    } else if (lastSelectedAuto != selectedAuto) {
      selectedAuto.loadAndUpdateViz(fieldViz);
      swerve.resetPose(selectedAuto.getInitialPose());
      if (Robot.isSimulation()) {
        swerve.resetSimPose(selectedAuto.getInitialPose());
      }
    }
    lastSelectedAuto = selectedAuto;
  }

  public void teleopPeriodic() {
    driverXbox.setRumble(
        RumbleType.kBothRumble,
        (leftTrigger.getAsBoolean() || bumperLeft.getAsBoolean())
                && (intake.hasPiece() || launcher.hasPiece())
            ? 1
            : 0);
  }

  public void robotPeriodic() {
    // Update canbus inputs
    rioBus.updateInputs();
    canivoreBus.updateInputs();

    // 3d viz
    final Pose3d intakeArm =
        Constants.Viz3d.intakePivotBase.transformBy(
            new Transform3d(0, 0, 0, new Rotation3d(0, intake.getAngle() - 0.5 * Math.PI, 0)));
    final Pose3d elevatorCarriage =
        Constants.Viz3d.elevatorBase.transformBy(
            new Transform3d(0, 0, elevator.getHeight(), new Rotation3d()));
    final Pose3d launcherArm =
        elevatorCarriage
            .transformBy(Constants.Viz3d.elevatorCarriageToLauncherArmPivot)
            .transformBy(
                new Transform3d(
                    0,
                    0,
                    0,
                    new Rotation3d(
                        0,
                        -launcher.getArmAngle() - Constants.Viz3d.elevatorBase.getRotation().getY(),
                        0)));
    Logger.recordOutput(
        "mechanismPoses",
        new Pose3d[] {intakeArm, elevatorCarriage, launcherArm, Constants.Viz3d.climberPivot});
  }
}
