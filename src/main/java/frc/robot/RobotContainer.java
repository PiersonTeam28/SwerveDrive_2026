
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.commands.SubsystemCommands;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Feeder.Speed;
import frc.util.SwerveTelemetry;

public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.SLOW; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    //private final AutoFactory autoFactory;
   // private final AutoChooser autoChooser = new AutoChooser();
    
    private DoubleSupplier input = () -> 0.1;
    
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");
    
     private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        drivetrain,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        arm,
        limelight
    );
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        drivetrain,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        arm,
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX()
    );

    public RobotContainer() {
       // autoFactory = drivetrain.createAutoFactory();
        
        //autoChooser.addRoutine("simple", autoRoutines::simplePathAuto);
        //SmartDashboard.putData("Auto Chooser", autoChooser);


        configureBindings();
        autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        //configureManualDriveBindings();

        limelight.setDefaultCommand(updateVisionCommand());


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention. (This is referring to the robot's coordinate system, not the joystick's coordinate system.)
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //-----------------------------Subsystem Bindings------------------//


        joystick1.y().onTrue(arm.upCommand()).onFalse(arm.stopCommand());
        joystick1.a().onTrue(arm.downCommand()).onFalse(arm.stopCommand());

        joystick1.povUp().onTrue(hood.positionCommand(0.7));
        joystick1.povDown().onTrue(hood.positionCommand(0.2));

        joystick1.povLeft().onTrue(hanger.latchCommand(0));
        joystick1.povRight().onTrue(hanger.latchCommand(1));



        joystick1.b().onTrue(feeder.indexCommand()).onFalse(feeder.stopFeeder());

       // hood.setDefaultCommand(hood.positionCommand(() -> joystick1.getRightY()));

        joystick1.rightBumper().onTrue(subsystemCommands.shootManually()).onFalse(subsystemCommands.stopShooter().andThen(feeder.stopFeeder())); 

        joystick.leftBumper().onTrue(shooter.stopCommand());

        //joystick1.b().onTrue(intake.testIntake(input)).onFalse(intake.testIntake(() -> 0));

        joystick1.x().onTrue(intake.intakeCommand()).onFalse(intake.stopIntake());

        //arm.setDefaultCommand(arm.pivotCommand(() -> joystick1.getLeftY()));

        hanger.setDefaultCommand(hanger.hangCommand(() -> joystick1.getRightY()));

    }

    // private void configureManualDriveBindings() {
    //     final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
    //         swerve, 
    //         () -> -joystick.getLeftY(), 
    //         () -> -joystick.getLeftX(), 
    //         () -> -joystick.getRightX()
    //     );
    //     swerve.setDefaultCommand(manualDriveCommand);
    //     joystick.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
    //     joystick.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
    //     joystick.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
    //     joystick.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
    //     joystick.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    // }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = drivetrain.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                drivetrain.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                
                );
            });
        }).ignoringDisable(true);
    }

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         drivetrain.applyRequest(() -> idle)
    //     );
    // }

    public Command getAutonomousCommand() {
        return autoRoutines.getAutoCommand();
    }
}
