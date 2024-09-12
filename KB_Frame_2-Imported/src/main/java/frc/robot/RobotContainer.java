// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RingStoreSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // public double speedLimit = 0.15;
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final RingStoreSubsystem m_RingStoreSubsystem = new RingStoreSubsystem();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
    // private final varChange025 vChange025 = new varChange025();
    // private final varChange05 vChange05 = new varChange05();
    // private final varChange10 vChange10 = new varChange10();
    // private final varChange075 vChange075 = new varChange075();
    // private final spinFire m_shoot = new spinFire(m_shooter);
    // The driver's controller
    Joystick m_driverController = new Joystick(0);
    JoystickButton driveButton1 = new JoystickButton(m_driverController, 1);
    JoystickButton driveButton2 = new JoystickButton(m_driverController, 2);
    // JoystickButton driveButton9 = new JoystickButton(m_driverController, 9);

    Joystick m_twangController = new Joystick(1);
    JoystickButton twangButton1 = new JoystickButton(m_twangController, 1);
    JoystickButton twangButton2 = new JoystickButton(m_twangController, 2);
    JoystickButton twangButton9 = new JoystickButton(m_twangController, 9);
    JoystickButton twangButton6 = new JoystickButton(m_twangController, 6);
    JoystickButton twangButton3 = new JoystickButton(m_twangController, 3);
    JoystickButton twangButton4 = new JoystickButton(m_twangController, 4);
    JoystickButton twangButton5 = new JoystickButton(m_twangController, 5);

    // JoystickButton button1 = new JoystickButton(m_driverController, 1);
    // JoystickButton button2 = new JoystickButton(m_driverController, 2);
    // JoystickButton button3 = new JoystickButton(m_driverController, 3);
    // JoystickButton button4 = new JoystickButton(m_driverController, 4);
    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                MathUtil.applyDeadband(
                                        m_driverController.getRawAxis(1),
                                        0.05),
                                MathUtil.applyDeadband(
                                        m_driverController.getRawAxis(0),
                                        0.05),
                                MathUtil.applyDeadband(
                                        m_driverController.getRawAxis(2) * 0.75,
                                        0.01),
                                true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or
     * one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    // I FIXED THE CODE FOR YOU ALREADY, DON'T TOUCH IT!!!!!!!
    private void configureButtonBindings() {

        driveButton1.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        // driveButton9.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(),
        // m_robotDrive));
        driveButton2.onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        driveButton2.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // driveButton8.onFalse(new RunCommand(() -> m_robotDrive.zeroHeading(),
        // m_robotDrive));

        twangButton9.whileTrue(new RunCommand(() -> m_shooter.Launch(0.125), m_shooter));
        twangButton2.whileTrue(new RunCommand(() -> m_shooter.Launch(-0.25), m_shooter));
        twangButton1.onTrue(new RunCommand(() -> m_IntakeSubsystem.IntakeSPIIIIIIIIIIIIIIN(0.45),
                m_IntakeSubsystem));
        twangButton1.onFalse(new RunCommand(() -> m_IntakeSubsystem.IntakeStop(), m_IntakeSubsystem));
        twangButton2.onFalse(new RunCommand(() -> m_shooter.EndLaunch(), m_shooter));
        twangButton4.onTrue(new RunCommand(() -> m_ClimberSubsystem.Uppies(), m_ClimberSubsystem));
        twangButton4.onFalse(new RunCommand(() -> m_ClimberSubsystem.narr(), m_ClimberSubsystem));
        twangButton5.onTrue(new RunCommand(() -> m_ClimberSubsystem.Downies(), m_ClimberSubsystem));
        twangButton5.onFalse(new RunCommand(() -> m_ClimberSubsystem.narr(), m_ClimberSubsystem));
        twangButton3.onTrue(new RunCommand(() -> m_RingStoreSubsystem.GastroIntestinalPush(0.75),
                m_RingStoreSubsystem));
        twangButton3.onFalse(new RunCommand(() -> m_RingStoreSubsystem.Flush(), m_RingStoreSubsystem));
        twangButton6.onTrue(new RunCommand(() -> m_shooter.Launch(1), m_shooter));
        twangButton6.onFalse(new RunCommand(() -> m_shooter.EndLaunch(), m_shooter));
        twangButton9.onFalse(new RunCommand(() -> m_shooter.EndLaunch(), m_shooter));
    }

    // public class ComplexAuto extends SequentialCommandGroup {
    // /**
    // * Creates a new ComplexAuto.
    // *
    // * @param drive The drive subsystem this command will run on
    // * @param hatch The hatch subsystem this command will run on
    // */
    // public ComplexAuto(DriveSubsystem drive) {
    // addCommands(
    // new setX(m_robotDrive.setX())
    // );
    // }
    // }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 3)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 3, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController,
                0,
                0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
        // Run path following command, then stop at the end.
        return Commands
                .runOnce(() -> m_shooter.Launch(1))
                .andThen(swerveControllerCommand);
    }
}
