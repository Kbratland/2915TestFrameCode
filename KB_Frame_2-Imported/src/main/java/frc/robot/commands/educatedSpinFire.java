package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RingCheckSubsystem;
import frc.robot.subsystems.RingStoreSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.eyeSpySubsystem;

public class educatedSpinFire extends Command {
    private final eyeSpySubsystem m_EyeSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final RingStoreSubsystem m_RingStoreSubsystem;
    private final RingCheckSubsystem m_CheckSubsystem;
    private final DriveSubsystem m_DriveSubsystem;
    Joystick m_driverController = new Joystick(0);
    @SuppressWarnings("unused")
    private final double DB_mode = .5;
    private long start;
    private final long duration = 3000;
    public double xSpeed = 0;
    public double ySpeed = 0;
    public double rotSpeed = 0;

    public educatedSpinFire(eyeSpySubsystem m_EyeSubsystem, ShooterSubsystem m_ShooterSubsystem,
            RingStoreSubsystem m_RingStoreSubsystem, RingCheckSubsystem m_CheckSubsystem,
            DriveSubsystem m_DriveSubsystem) {
        this.m_EyeSubsystem = m_EyeSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_RingStoreSubsystem = m_RingStoreSubsystem;
        this.m_CheckSubsystem = m_CheckSubsystem;
        this.m_DriveSubsystem = m_DriveSubsystem;
    }

    @Override
    public void initialize() {
        start = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        System.out.println(m_EyeSubsystem.getTargetYaw() + " is the target's Yaw");
        System.out.println(m_EyeSubsystem.getTargetPitch() + " is the target's Pitch");
        System.out.println(m_EyeSubsystem.getTargetArea() + " is the target's Area");
        if (m_EyeSubsystem.targetSpotted()) {
            if (m_EyeSubsystem.getTargetArea() >= 0.3) {
                ySpeed = -.15;
            } else if (m_EyeSubsystem.getTargetArea() <= 0.8) {
                ySpeed = 0.15;
            } else if (m_EyeSubsystem.getTargetArea() <= 0.3 && m_EyeSubsystem.getTargetArea() >= 0.8) {
                ySpeed = 0;
                if (m_EyeSubsystem.getTargetYaw() >= 0.2) {
                    rotSpeed = -0.15;
                } else if (m_EyeSubsystem.getTargetYaw() <= -0.2) {
                    rotSpeed = 0.15;
                } else {
                    rotSpeed = 0;
                    m_ShooterSubsystem.Launch(0.85);
                    System.out.println("attempting to shoot");
                    if ((m_ShooterSubsystem.getSpeed(1) > 1000 && m_ShooterSubsystem.getSpeed(2) > 1000
                            && m_ShooterSubsystem.getSpeed(3) > 1000 && m_ShooterSubsystem.getSpeed(4) > 1000)
                            || System.currentTimeMillis() - start > 2000) {
                        System.out.println("shooting");
                        m_RingStoreSubsystem.GastroIntestinalPush(0.5);
                    }
                }
            }
            m_DriveSubsystem.drive(m_driverController.getRawAxis(1), ySpeed, rotSpeed, true);
        }
    }

    @Override
    public boolean isFinished() {
        long e = 0;
        long t = 0;
        long shotTime = 0;
        long postShotDuration = 500;
        boolean shotInitiated = false;
        while (e <= duration && t <= postShotDuration) {
            e = System.currentTimeMillis() - start;
            if (!m_CheckSubsystem.isShooterSwitchClosed() && !shotInitiated) {
                shotInitiated = true;
                shotTime = System.currentTimeMillis();
            }
            if (shotInitiated) {
                t = System.currentTimeMillis() - shotTime;
            }
        }
        m_ShooterSubsystem.EndLaunch();
        m_RingStoreSubsystem.Flush();
        return (true);
    }
}