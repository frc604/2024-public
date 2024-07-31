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

public class Auto876 implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_8 =
      new QuikPlanSwervePartialTrajectoryReader("876-8.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_8s7 =
      new QuikPlanSwervePartialTrajectoryReader("876-8s7.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_87 =
      new QuikPlanSwervePartialTrajectoryReader("876-87.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_7s6 =
      new QuikPlanSwervePartialTrajectoryReader("876-7s6.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_76 =
      new QuikPlanSwervePartialTrajectoryReader("876-76.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_6s =
      new QuikPlanSwervePartialTrajectoryReader("876-6s.csv");

  private final Command m_command;

  public Auto876(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher) {
    m_command =
        new SequentialCommandGroup(
            new FollowQuikplan(m_8, swerve, intake, launcher),
            new ConditionalCommand(
                new FollowQuikplan(m_8s7, swerve, intake, launcher),
                new FollowQuikplan(m_87, swerve, intake, launcher),
                () -> intake.recentlyHadPiece() || launcher.hasPiece()),
            new ConditionalCommand(
                new FollowQuikplan(m_7s6, swerve, intake, launcher),
                new FollowQuikplan(m_76, swerve, intake, launcher),
                () -> intake.recentlyHadPiece() || launcher.hasPiece()),
            new FollowQuikplan(m_6s, swerve, intake, launcher, EndMode.FINISH));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_8.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_8, m_8s7, m_87, m_7s6, m_76, m_6s));
  }
}
