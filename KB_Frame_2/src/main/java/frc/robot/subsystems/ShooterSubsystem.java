package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;


public class ShooterSubsystem extends SubsystemBase
{
    private final CANSparkMax FLShoot = new CANSparkMax(DriveConstants.kFrontLeftShooterCanId, MotorType.kBrushless);
    private final CANSparkMax FRShoot = new CANSparkMax(DriveConstants.kFrontRightShooterCanId, MotorType.kBrushless);
    private final CANSparkMax RLShoot = new CANSparkMax(DriveConstants.kRearLeftShooterCanId, MotorType.kBrushless);
    private final CANSparkMax RRShoot = new CANSparkMax(DriveConstants.kRearRightShooterCanId, MotorType.kBrushless);

    private RelativeEncoder FLEncoder;
    private RelativeEncoder FREncoder;
    private RelativeEncoder RLEncoder;
    private RelativeEncoder RREncoder;

    public void Launch(double fLSpeed, double fRSpeed, double rLSpeed, double rRSpeed)
    {
        FLEncoder = FLShoot.getEncoder();
        FREncoder = FRShoot.getEncoder();
        RLEncoder = RLShoot.getEncoder();
        RREncoder = RRShoot.getEncoder();
        FLShoot.set(fLSpeed);
        FRShoot.set(fRSpeed);
        RLShoot.set(rLSpeed);
        RRShoot.set(rRSpeed);
    }
    public void EndLaunch()
    {
        FLShoot.set(0);
        FRShoot.set(0);
        RLShoot.set(0);
        RRShoot.set(0);
    }
    public double getSpeed(int motorNum)
    {
        double encoderOut = 0;
        if(motorNum == 1)
        {
            encoderOut = FLEncoder.getVelocity();
        }
        else if(motorNum == 2)
        {
            encoderOut = FREncoder.getVelocity();
        }
        else if(motorNum == 3)
        {
            encoderOut = RLEncoder.getVelocity();
        }
        else if(motorNum == 4)
        {
            encoderOut = RREncoder.getVelocity();
        }
        if(encoderOut == 0)
        {
            System.out.println("No arg for motorNum, outputing 0 for observed speed");
        }
        return encoderOut;
        
    }
}
