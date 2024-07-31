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

public class Auto231456 implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_2s3s1s =
      new QuikPlanSwervePartialTrajectoryReader("231456-2s3s1s.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_14 =
      new QuikPlanSwervePartialTrajectoryReader("231456-14.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_45 =
      new QuikPlanSwervePartialTrajectoryReader("231456-45.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_4s5 =
      new QuikPlanSwervePartialTrajectoryReader("231456-4s5.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_5s6 =
      new QuikPlanSwervePartialTrajectoryReader("231456-5s6.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_56 =
      new QuikPlanSwervePartialTrajectoryReader("231456-56.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_6s =
      new QuikPlanSwervePartialTrajectoryReader("231456-6s.csv");
  private final Command m_command;

  public Auto231456(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher) {
    m_command =
        new SequentialCommandGroup(
            new FollowQuikplan(m_2s3s1s, swerve, intake, launcher, EndMode.IMMEDIATE),
            new FollowQuikplan(m_14, swerve, intake, launcher),
            new ConditionalCommand(
                new FollowQuikplan(m_4s5, swerve, intake, launcher),
                new FollowQuikplan(m_45, swerve, intake, launcher),
                () -> intake.recentlyHadPiece() || launcher.hasPiece()),
            new ConditionalCommand(
                new FollowQuikplan(m_5s6, swerve, intake, launcher),
                new FollowQuikplan(m_56, swerve, intake, launcher),
                () -> intake.recentlyHadPiece() || launcher.hasPiece()),
            new FollowQuikplan(m_6s, swerve, intake, launcher, EndMode.FINISH));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_2s3s1s.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_2s3s1s, m_14, m_45, m_4s5, m_5s6, m_56, m_6s));
  }
}
