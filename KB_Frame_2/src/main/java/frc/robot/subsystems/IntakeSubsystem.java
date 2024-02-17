package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonSRX UpperSlurper = new TalonSRX(DriveConstants.kUpperIntakeCanId);
    private final TalonSRX LowerSlurper = new TalonSRX(DriveConstants.kLowerIntakeCanId);
    public void IntakeSPIIIIIIIIIIIIIIN(double speed)
    {
        UpperSlurper.set(ControlMode.PercentOutput, speed);
        LowerSlurper.set(ControlMode.PercentOutput, speed);
    }
}