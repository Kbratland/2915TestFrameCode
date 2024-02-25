package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.eyeSpySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RingStoreSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RingCheckSubsystem;

public class educatedSpinFire extends Command {
    private eyeSpySubsystem m_EyeSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private RingStoreSubsystem m_RingStoreSubsystem;
    private RingCheckSubsystem m_CheckSubsystem;

    private double DB_mode = .5;
    private long start;
    private long duration = 3000;
    private double speed;
    
    

    public educatedSpinFire(eyeSpySubsystem m_EyeSubsystem, ShooterSubsystem m_ShooterSubsystem, RingStoreSubsystem m_RingStoreSubsystem, RingCheckSubsystem m_CheckSubsystem){
        this.m_EyeSubsystem = m_EyeSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_RingStoreSubsystem = m_RingStoreSubsystem;
        this.m_CheckSubsystem = m_CheckSubsystem;
    }

    public void initialize() {
        start = System.currentTimeMillis();
      }
    
      public void execute() {
        if(m_EyeSubsystem.getTargetArea() >= 0.5){
          
        }
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
        long t = 0;
        long shotTime = 0;
        long postShotDuration = 500;
        boolean shotInitiated = false;
        while(e <= duration && t <= postShotDuration)
        {
            e = System.currentTimeMillis() - start;
            if(!m_CheckSubsystem.isShooterSwitchClosed() && !shotInitiated)
            {
                shotInitiated = true;
                shotTime = System.currentTimeMillis();
            }
            if(shotInitiated)
            {
                t = System.currentTimeMillis() - shotTime;
            }
        }
        m_ShooterSubsystem.EndLaunch();
        m_RingStoreSubsystem.Flush();
        return(true);
      }
}
