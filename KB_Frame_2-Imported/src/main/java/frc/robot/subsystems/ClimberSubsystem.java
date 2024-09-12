package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonSRX ClimberLeft = new TalonSRX(DriveConstants.kLeftClimber);
    private final TalonSRX ClimberRight = new TalonSRX(DriveConstants.kRightClimber);

    public void Uppies() {

        ClimberLeft.set(ControlMode.PercentOutput, 0.5);
        ClimberRight.set(ControlMode.PercentOutput, 0.5);
    }

    public void Downies() {
        ClimberLeft.set(ControlMode.PercentOutput, -0.5);
        ClimberRight.set(ControlMode.PercentOutput, -0.5);
    }

    public void narr() {
        ClimberLeft.set(ControlMode.PercentOutput, 0);
        ClimberRight.set(ControlMode.PercentOutput, 0);
    }
}
