package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RingCheckSubsystem;
import frc.robot.subsystems.RingStoreSubsystem;

public class Intake extends Command {

    IntakeSubsystem m_IntakeSubsystem;
    RingCheckSubsystem m_RingCheckSubsystem;
    RingStoreSubsystem m_RingStoreSubsystem;

    private long start;
    private final long duration = 7000;

    public Intake(IntakeSubsystem m_IntakeSubsystem, RingCheckSubsystem m_RingCheckSubsystem,
            RingStoreSubsystem m_RingStoreSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.m_RingCheckSubsystem = m_RingCheckSubsystem;
        this.m_RingStoreSubsystem = m_RingStoreSubsystem;

    }

    @Override
    public void initialize() {
        start = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        m_IntakeSubsystem.IntakeSPIIIIIIIIIIIIIIN(0.45);
        m_RingStoreSubsystem.GastroIntestinalPush(0.32);
    }

    @Override
    public boolean isFinished() {
        long e = 0;
        while (m_RingCheckSubsystem.isShooterSwitchClosed() && e <= duration) {
            e = System.currentTimeMillis() - start;
        }
        m_IntakeSubsystem.IntakeStop();
        m_RingStoreSubsystem.Flush();
        return true;
    }
}
