package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.Shooter;

import frc.util.ShotCalculator.*;
import frc.util.ProjectileSimulator.*;
import frc.util.ShotCalculator;



// So this is aim and drive but with our swerve instead of the other one idk 

public class AimAndDriveCommand2 extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

   // private final Swerve swerve;
    private final CommandSwerveDrivetrain drivetrain;
   // private final Shooter shooter;
    private final DriveInputSmoother inputSmoother;

    //GeneratedLUT lut = new GeneratedLUT(null, null, 0, 0, 0, 0);
    GeneratedLUT lut;
    Config config = new Config();
    private ShotCalculator calc = new ShotCalculator(config);




    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand2(        
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.drivetrain = drivetrain;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
            addRequirements(drivetrain);
        

        //lut stuff
        for(var entry : lut.entries()) {
            if(entry.reachable()){
                calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
        //---//
    }

    public AimAndDriveCommand2(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = drivetrain.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(drivetrain.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(drivetrain.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        drivetrain.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub())
        );
    }

    //method for calc
    // public void shotCalc(){
    // ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
    //     drivetrain.getPose(), drivetrain.getFieldVelocity(), drivetrain.getRobotVelocity(),
    //     hubCenter, hubForwardVector, visionConfidence
    // );
    // ShotCalculator.LaunchParameters result = calc.calculate(inputs);
    // if (result.isValid() && result.confidence() > 50) {
    //     shooter.setRPM(result.rpm());
    //     drivetrain.setHeading(result.driveAngle());
    // }
    // }
    

    @Override
    public boolean isFinished() {
        return false;
    }
}
