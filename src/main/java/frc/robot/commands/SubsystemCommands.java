package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;



import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class SubsystemCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Arm arm;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        CommandSwerveDrivetrain drivetrain,
        Intake intake,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Arm arm,
        
        
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.arm = arm;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        CommandSwerveDrivetrain drivetrain,
        Intake intake,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Arm arm
    ) {
        this(
            drivetrain,
            intake,
            feeder,
            shooter,
            hood,
            hanger,
            arm,
            () -> 0,
            () -> 0
        );
    }


    public Command aimAndShoot2() {
        final AimAndDriveCommand2 aimAndDriveCommand = new AimAndDriveCommand2(drivetrain, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> drivetrain.getState().Pose);
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed1())
        );
    }



    public Command stopShooter() {
        return Commands.runOnce(() -> shooter.stop());
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed1())
            .handleInterrupt(() -> shooter.stop());
    }

    public Command shootAuto(double rpm) {
        //LimelightHelpers.
        return shooter.spinUpCommand(rpm).andThen(Commands.waitSeconds(1), feed1());
    }

    private Command feed1() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            feeder.indexCommand()).handleInterrupt(() -> feeder.stop());
    }

}
