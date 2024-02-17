package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonSRX UpperSlurper = new TalonSRX(DriveConstants.kUpperIntakeCanId);
    private final TalonSRX LowerSlurper = new TalonSRX(DriveConstants.kLowerIntakeCanId)
}
