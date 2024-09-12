package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax FLShoot = new CANSparkMax(DriveConstants.kFrontLeftShooterCanId, MotorType.kBrushless);
    private final CANSparkMax FRShoot = new CANSparkMax(DriveConstants.kFrontRightShooterCanId, MotorType.kBrushless);
    private final CANSparkMax RLShoot = new CANSparkMax(DriveConstants.kRearLeftShooterCanId, MotorType.kBrushless);
    private final CANSparkMax RRShoot = new CANSparkMax(DriveConstants.kRearRightShooterCanId, MotorType.kBrushless);

    private RelativeEncoder FLEncoder;
    private RelativeEncoder FREncoder;
    private RelativeEncoder RLEncoder;
    private RelativeEncoder RREncoder;

    PIDController pider = new PIDController(
            DriveConstants.kTurnP,
            DriveConstants.kTurnI,
            DriveConstants.kTurnD);

    public void Launch(double speed) {
        double fLSpeed = -speed;
        double fRSpeed = speed;
        double rLSpeed = -speed / 1.2;
        double rRSpeed = speed / 1.2;
        pider.enableContinuousInput(0, 1);

        FLEncoder = FLShoot.getEncoder();
        FREncoder = FRShoot.getEncoder();
        RLEncoder = RLShoot.getEncoder();
        RREncoder = RRShoot.getEncoder();
        FLShoot.set(fLSpeed);
        FRShoot.set(fRSpeed);
        RLShoot.set(rLSpeed);
        RRShoot.set(rRSpeed);
    }

    public void zucc() {
        FLEncoder = FLShoot.getEncoder();
        FREncoder = FRShoot.getEncoder();
        RLEncoder = RLShoot.getEncoder();
        RREncoder = RRShoot.getEncoder();
        FLShoot.set(-0.3);
        FRShoot.set(-0.3);
        RLShoot.set(-0.3);
        RRShoot.set(-0.3);
    }

    public void EndLaunch() {
        FLShoot.set(0);
        FRShoot.set(0);
        RLShoot.set(0);
        RRShoot.set(0);
    }

    public double getSpeed(int motorNum) {
        double encoderOut = 0;
        switch (motorNum) {
            case 1 ->
                encoderOut = FLEncoder.getVelocity();
            case 2 ->
                encoderOut = FREncoder.getVelocity();
            case 3 ->
                encoderOut = RLEncoder.getVelocity();
            case 4 ->
                encoderOut = RREncoder.getVelocity();
            default -> {
            }
        }
        if (encoderOut == 0) {
            System.out.println("No arg for motorNum, outputing 0 for observed speed");
        }
        return encoderOut;

    }

    public CANSparkMax getRRShoot() {
        return RRShoot;
    }
}
