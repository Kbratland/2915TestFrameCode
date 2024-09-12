package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class RingCheckSubsystem extends SubsystemBase {

    DigitalInput shooterSwitch = null;

    public RingCheckSubsystem() {
        shooterSwitch = new DigitalInput(DriveConstants.ringSwitch);
    }

    public boolean isShooterSwitchClosed() {
        return shooterSwitch.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
