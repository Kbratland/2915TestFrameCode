package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class rotateToTarget extends CommandBase {

  DriveSubsystem m_robotDrive;

  public rotateToTarget(DriveSubsystem m_robotDrive) {
    this.m_robotDrive = m_robotDrive;
  }

  public void initialize() {}

  public void execute() {}

  public boolean isFinished() {
    return false;
  }
}
