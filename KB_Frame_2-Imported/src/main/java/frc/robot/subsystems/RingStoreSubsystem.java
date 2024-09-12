package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class RingStoreSubsystem extends SubsystemBase {

    private final CANSparkMax LeftStore = new CANSparkMax(DriveConstants.kLeftStoreCanId, MotorType.kBrushed);
    private final CANSparkMax RightStore = new CANSparkMax(DriveConstants.kRightStoreCanId, MotorType.kBrushed);

    public void GastroIntestinalPush(double thirst) {
        LeftStore.set(thirst);
        RightStore.set(-thirst);
    }

    public void Flush() {
        // System.out.println("Done Storing");
        LeftStore.set(0);
        RightStore.set(0);
    }
}
