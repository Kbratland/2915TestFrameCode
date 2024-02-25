package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RingStoreSubsystem;



public class spinFire extends Command {
  private double DB_mode = .5;
  private long start;
  private long duration = 3000;
  private double fLSpeed = -1 * DB_mode;
  private double fRSpeed = DB_mode;
  private double rLSpeed = -1 * DB_mode;
  private double rRSpeed = DB_mode;
  private ShooterSubsystem m_ShooterSubsystem;
  private RingStoreSubsystem m_RingStoreSubsystem;
  private double speed = 0.5;
  public spinFire(ShooterSubsystem m_ShooterSubsystem) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;

  }

  public void initialize() {
    start = System.currentTimeMillis();
  }

  public void execute() {
    m_ShooterSubsystem.Launch(speed);
    System.out.println("attempting to shoot");
    if((
      m_ShooterSubsystem.getSpeed(1) > 1000 &&
      m_ShooterSubsystem.getSpeed(2) > 1000 &&
      m_ShooterSubsystem.getSpeed(3) > 1000 &&
      m_ShooterSubsystem.getSpeed(4) > 1000) ||
      System.currentTimeMillis() - start > 2000
    )
    {
      System.out.println("shooting");
      m_RingStoreSubsystem.GastroIntestinalPush(0.5);
    }
  }

  public boolean isFinished() {
    long e = 0;
    while(e <= duration)
    {
      e = System.currentTimeMillis() - start;
    }
    m_ShooterSubsystem.EndLaunch();
    m_RingStoreSubsystem.Flush();
    return(true);
  }
}
