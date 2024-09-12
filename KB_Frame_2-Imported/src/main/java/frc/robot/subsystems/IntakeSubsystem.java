package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {

    // private RingStoreSubsystem m_RingStoreSubsystem = new RingStoreSubsystem();
    private final TalonSRX UpperSlurper = new TalonSRX(DriveConstants.kUpperIntakeCanId);
    private final TalonSRX LowerSlurper = new TalonSRX(DriveConstants.kLowerIntakeCanId);

    public void IntakeSPIIIIIIIIIIIIIIN(double speed) {
        // System.out.println("SLURP");
        UpperSlurper.set(ControlMode.PercentOutput, (3 * speed));
        LowerSlurper.set(ControlMode.PercentOutput, (speed));
        // m_RingStoreSubsystem.GastroIntestinalPush(0.3);
    }

    public void IntakeStop() {
        UpperSlurper.set(ControlMode.PercentOutput, 0);
        LowerSlurper.set(ControlMode.PercentOutput, 0);
        // m_RingStoreSubsystem.Flush();
    }
}
