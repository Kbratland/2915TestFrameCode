package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RingStoreSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class spinFire extends Command {

    private long start;
    private final long duration = 3000;
    private final ShooterSubsystem m_ShooterSubsystem;
    private RingStoreSubsystem m_RingStoreSubsystem;
    private final double speed;

    public spinFire(ShooterSubsystem m_ShooterSubsystem) {
        this.speed = 0.85;
        this.m_ShooterSubsystem = m_ShooterSubsystem;

    }

    @Override
    public void initialize() {
        start = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.Launch(speed);
        System.out.println("attempting to shoot");
        if ((m_ShooterSubsystem.getSpeed(1) > 1000
                && m_ShooterSubsystem.getSpeed(2) > 1000
                && m_ShooterSubsystem.getSpeed(3) > 1000
                && m_ShooterSubsystem.getSpeed(4) > 1000)
                || System.currentTimeMillis() - start > 2000) {
            System.out.println("shooting");
            m_RingStoreSubsystem.GastroIntestinalPush(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        long e = 0;
        while (e <= duration) {
            e = System.currentTimeMillis() - start;
        }
        m_ShooterSubsystem.EndLaunch();
        m_RingStoreSubsystem.Flush();
        return (true);
    }
}
