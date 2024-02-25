package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberSubsystem extends SubsystemBase
{
    private final TalonSRX rightClimber = new TalonSRX(DriveConstants.kRightClimberCanId);
    private final TalonSRX leftClimber = new TalonSRX(DriveConstants.kLeftClimberCanId);
    public void UppiesSpeed(double strength)
    {
        rightClimber.set(ControlMode.PercentOutput, strength);
        leftClimber.set(ControlMode.PercentOutput, strength);
    }
    public void NoUppies()
    {
        rightClimber.set(ControlMode.PercentOutput, 0);
        leftClimber.set(ControlMode.PercentOutput, 0);
    }
    public void ClimbBrakesON()
    {
        rightClimber.setNeutralMode(NeutralMode.Brake);
        leftClimber.setNeutralMode(NeutralMode.Brake);
    }
}
