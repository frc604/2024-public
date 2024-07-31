// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.commands.FollowQuikplan;
import frc.robot.commands.FollowQuikplan.EndMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class Auto786drop implements AutoCommand {
  // Starting trajectory
  private final QuikPlanSwervePartialTrajectoryReader m_7 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-7.csv");

  // Recovery trajectories
  private final QuikPlanSwervePartialTrajectoryReader m_78 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-78.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_86 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-86.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_60 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-60.csv");

  // Pickup to score
  private final QuikPlanSwervePartialTrajectoryReader m_8s =
      new QuikPlanSwervePartialTrajectoryReader("876drop-8s.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_7s =
      new QuikPlanSwervePartialTrajectoryReader("876drop-7s.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_6s =
      new QuikPlanSwervePartialTrajectoryReader("876drop-6s.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_0s =
      new QuikPlanSwervePartialTrajectoryReader("876drop-0s.csv");

  // Score to pickup
  private final QuikPlanSwervePartialTrajectoryReader m_s8 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-s8.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_s6 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-s6.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_s0 =
      new QuikPlanSwervePartialTrajectoryReader("876drop-s0.csv");

  private final Command m_command;

  public Auto786drop(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher) {
    this(swerve, intake, launcher, false);
  }

  public Auto786drop(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher,
      final boolean skipThirdPickup) {
    m_command =
        new SequentialCommandGroup(
            new FollowQuikplan(m_7, swerve, intake, launcher, true),
            new ConditionalCommand(
                // Picked up 7
                new SequentialCommandGroup(
                    new FollowQuikplan(m_7s, swerve, intake, launcher, true),
                    new FollowQuikplan(m_s8, swerve, intake, launcher, true),
                    new ConditionalCommand(
                        // Picked up 8
                        new SequentialCommandGroup(
                            new FollowQuikplan(m_8s, swerve, intake, launcher, true),
                            new FollowQuikplan(m_s0, swerve, intake, launcher, true),
                            new FollowQuikplan(m_0s, swerve, intake, launcher, true)),
                        // Missed 8
                        new SequentialCommandGroup(
                            new FollowQuikplan(m_86, swerve, intake, launcher, true),
                            create6s0sCommand(swerve, intake, launcher)),
                        () -> intake.recentlyHadPiece() || launcher.hasPiece() || skipThirdPickup)),
                // Missed 7
                new SequentialCommandGroup(
                    new FollowQuikplan(m_78, swerve, intake, launcher, true),
                    new ConditionalCommand(
                        // Picked up 8
                        new SequentialCommandGroup(
                            new FollowQuikplan(m_8s, swerve, intake, launcher, true),
                            new ConditionalCommand(
                                // Skip 6 and go for 0
                                new SequentialCommandGroup(
                                    new FollowQuikplan(m_s0, swerve, intake, launcher, true),
                                    new FollowQuikplan(m_0s, swerve, intake, launcher, true)),
                                // Go for 6 and 0
                                new SequentialCommandGroup(
                                    new FollowQuikplan(m_s6, swerve, intake, launcher, true),
                                    create6s0sCommand(swerve, intake, launcher)),
                                () -> skipThirdPickup)),
                        // Missed 8
                        new SequentialCommandGroup(
                            new FollowQuikplan(m_86, swerve, intake, launcher, true),
                            create6s0sCommand(swerve, intake, launcher)),
                        () -> intake.recentlyHadPiece() || launcher.hasPiece())),
                () -> intake.recentlyHadPiece() || launcher.hasPiece()));
  }

  private Command create6s0sCommand(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher) {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            // Picked up 6
            new SequentialCommandGroup(
                new FollowQuikplan(m_6s, swerve, intake, launcher, true),
                new FollowQuikplan(m_s0, swerve, intake, launcher, true)),
            // Missed 6
            new FollowQuikplan(m_60, swerve, intake, launcher, true),
            () -> intake.recentlyHadPiece() || launcher.hasPiece()),
        new FollowQuikplan(m_0s, swerve, intake, launcher, EndMode.FINISH, true));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_7.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(
        Arrays.asList(m_7, m_78, m_86, m_60, m_8s, m_7s, m_6s, m_0s, m_s8, m_s6, m_s0));
  }
}
