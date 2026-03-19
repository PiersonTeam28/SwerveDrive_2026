package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;

import frc.robot.Constants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.ProjectileSimulator;
import frc.util.ShotCalculator;
import frc.util.ProjectileSimulator.*;
import frc.util.ShotCalculator.*;

public class AimBot extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    private final ProjectileSimulator pSim = new ProjectileSimulator(new ProjectileSimulator.SimParameters(Constants.BALL_MASS, Constants.BALL_DIAMETER, Constants.DRAG_COEFF, Constants.MAGNUS_COEFF, Constants.AIR_DENSITY, Constants.EXIT_HEIGHT, Constants.WHEEL_DIAMETER, Constants.TARGET_HEIGHT, Constants.SLIP_FACTOR, Constants.LAUNCH_ANGLE, Constants.SIM_TIMESTEP, Constants.MIN_RPM, Constants.MAX_RPM, Constants.SEARCH_ITERS, Constants.MAX_SIM_TIME));
    private GeneratedLUT lut;
    private Config config = new Config();
    private ShotCalculator calc = new ShotCalculator();

    


    private final Shooter shooter;
    private final Hood hood;
    private final Limelight limelight;
    private final CommandSwerveDrivetrain swerve;
    private final Supplier<Pose2d> robotPoseSupplier;

    public AimBot(Shooter shooter, Hood hood, CommandSwerveDrivetrain swerve, Limelight limelight, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(shooter, hood, swerve);

        lut = pSim.generateLUT();

        for (LUTEntry entry : lut.entries()) {
        if (entry.reachable()) {
            System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                entry.distanceM(), entry.rpm(), entry.tof());
        }
    }

        for(var entry : lut.entries()) {
            if(entry.reachable()){
                calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    //method for calc
    // Shot Inputs are Pose, field Velocity, Robot velocity, hubCenter, hubForward Vector, vision confidence
    // public void shotCalc(){
    // ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
    //     swerve.getState().Pose, swerve.getState().Speeds.fromRobotRelativeSpeeds(swerve.getState().Speeds, swerve.getState().Pose.getRotation()), swerve.getState().Speeds,
    //     hubCenter, hubForwardVector, visionConfidence
    // );
    // ShotCalculator.LaunchParameters result = calc.calculate(inputs);
    // if (result.isValid() && result.confidence() > 50) {
    //     shooter.setRPM(result.rpm());
    //     swerve.setHeading(result.driveAngle());
    // }
    // }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}