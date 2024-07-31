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

public class Auto678 implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_6 =
      new QuikPlanSwervePartialTrajectoryReader("678-6.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_6s7 =
      new QuikPlanSwervePartialTrajectoryReader("678-6s7.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_67 =
      new QuikPlanSwervePartialTrajectoryReader("678-67.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_7s8 =
      new QuikPlanSwervePartialTrajectoryReader("678-7s8.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_78 =
      new QuikPlanSwervePartialTrajectoryReader("678-78.csv");
  private final QuikPlanSwervePartialTrajectoryReader m_8s =
      new QuikPlanSwervePartialTrajectoryReader("678-8s.csv");

  private final Command m_command;

  public Auto678(
      final SwerveSubsystem swerve,
      final IntakeSubsystem intake,
      final LauncherSubsystem launcher) {
    m_command =
        new SequentialCommandGroup(
            new FollowQuikplan(m_6, swerve, intake, launcher),
            new ConditionalCommand(
                new FollowQuikplan(m_6s7, swerve, intake, launcher),
                new FollowQuikplan(m_67, swerve, intake, launcher),
                () -> true),
            new ConditionalCommand(
                new FollowQuikplan(m_7s8, swerve, intake, launcher),
                new FollowQuikplan(m_78, swerve, intake, launcher),
                () -> true),
            new FollowQuikplan(m_8s, swerve, intake, launcher, EndMode.FINISH));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_6.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_6, m_6s7, m_67, m_7s8, m_78, m_8s));
  }
}
