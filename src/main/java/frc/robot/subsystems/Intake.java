package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;


//SPARK MAX IMPORTS
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.encoder.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.config.*;
import com.revrobotics.spark.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.4);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final AngularVelocity kMaxPivotVelocity = Constants.kPivotRPM.div(kPivotReduction); //change from Kraken to vex 775 pro motor, GB Ratio 4:1
    private static final Angle kPositionTolerance = Degrees.of(5);

    private static final double kAllowedError = 0.5;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private static final double kS = 0;
    private static final double kV = 0;
    private static final double kA = 0.01;

    private static final double SLOWL = 1;

    private static final double kMaxPV = kMaxPivotVelocity.in(RPM)* SLOWL; // to double?
    private static final double kMaxPA = kMaxPivotVelocity.in(RotationsPerSecond) * SLOWL; // to double?

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private final SparkMax pivot;
    private final SparkClosedLoopController pivotController;
    //private final AbsoluteEncoder pivotEncoder;
    private final AbsoluteEncoderConfig pivotEncoderConfig;
    //private final MAXMotionConfig maxMotionConfig;







    private boolean isHomed = false;

    private boolean invertPivot = false;

    public Intake() {                               // PIVOT MOTOR IS A SPARKMAX MOTOR
        
        pivot = new SparkMax(Constants.kIntakePivot, MotorType.kBrushed); //SPARKMAX 29
       
        
        pivotEncoderConfig = new AbsoluteEncoderConfig();


       // maxMotionConfig = new MAXMotionConfig();

        
        pivotMotor = new TalonFX(Constants.kIntakePivot, Constants.kCANivoreCANBus); 
        rollerMotor = new TalonFX(Constants.kIntakeRollers, Constants.kRoboRioCANBus);
        
        configurePivotMotor();
        configureRollerMotor();
       
        configureSparkMaxPivot();

        pivotController = pivot.getClosedLoopController();
       
        SmartDashboard.putData(this);
    }

    private void configureSparkMaxPivot() {
        final SparkMaxConfig pivotConfig = new SparkMaxConfig();
        
        pivotConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
            .inverted(invertPivot);
        
        pivotConfig.absoluteEncoder.apply(pivotEncoderConfig);
        
        pivotConfig.closedLoop.maxMotion
            .cruiseVelocity(kMaxPV, ClosedLoopSlot.kSlot0)
            .maxAcceleration(kMaxPA, ClosedLoopSlot.kSlot0)
            .allowedProfileError(kAllowedError, ClosedLoopSlot.kSlot0);

        pivotConfig.closedLoop
            .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

        // pivotConfig.closedLoop.feedForward
        //     .kA(kA, ClosedLoopSlot.kSlot0)
        //     .kS(kS, ClosedLoopSlot.kSlot0)
        //     .kV(kV, ClosedLoopSlot.kSlot0);


        
       
       
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

    }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
        
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }


    // Checks if the pivot is within the position tolerance of the target position TALON
    // Changed to SparkMAX
    private boolean isPositionWithinTolerance() {
        //final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle currentPos = Degrees.of(pivot.getAbsoluteEncoder().getPosition());
        //pivot.getAbsoluteEncoder().getPosition();
       // pivotController.getMAXMotionSetpointPosition();
        //final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        final Angle targetPos = Degrees.of(pivotController.getMAXMotionSetpointPosition());

    
        return currentPos.isNear(targetPos, kPositionTolerance);
    }

    // set pivot TALON percent output
    // private void setPivotPercentOutput(double percentOutput) {
    //     pivotMotor.setControl(
    //         pivotVoltageRequest
    //             .withOutput(Volts.of(percentOutput * 12.0))
    //     );
    // }

    private void setPivotPercentOutput(double percentOutput) {
        pivot.setVoltage(Volts.of(percentOutput * 12.0));
        //pivot.set(percentOutput);
    }

    // set pivot TALON to position
    // public void set(Position position) {
    //     pivotMotor.setControl(
    //         pivotMotionMagicRequest
    //             .withPosition(position.angle())
    //     );
    // }

    public void set(double setpoint){
        pivotController.setSetpoint(setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

   
    public void testSetPivotPercentOutput(double percentOutput) {
        pivot.setVoltage(Volts.of(percentOutput * 12.0));
    }


    // set speed for roller TALON
    public void set(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    public Command stopIntake() {
        return runOnce(() -> set(Speed.STOP));
    }

    // actual intake command 
    public Command intakeCommand() {
        return startEnd(
            () -> {
                //set(Position.INTAKE);
                set(Position.INTAKE.angle().in(Degrees));
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)
        );
    }

    public Command testIntake(DoubleSupplier speedSupplier) {
        return run(() -> {
            testSetPivotPercentOutput(speedSupplier.getAsDouble());
        });
    }


    // agitiate command 
    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE.angle().in(Degrees))),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE.angle().in(Degrees))),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE.angle().in(Degrees));
                set(Speed.STOP);
            });
    }


    // homing command 
    // public Command homingCommand() {
    //     return Commands.sequence(
    //         runOnce(() -> setPivotPercentOutput(0.1)),
    //         Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
    //         runOnce(() -> {
    //             pivotMotor.setPosition(Position.HOMED.angle());
    //             isHomed = true;
    //             set(Position.STOWED.angle().in(Degrees));
    //         })
    //     )
    //     .unless(() -> isHomed)
    //     .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivot.getOutputCurrent() > 6),
            runOnce(() -> {
                //pivot.setPosition(Position.HOMED.angle());
                pivotController.setSetpoint(Position.HOMED.angle().in(Degrees), SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
                isHomed = true;
                set(Position.STOWED.angle().in(Degrees));
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);

        // builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);

        builder.addDoubleProperty("Encoder Position", () -> pivot.getAbsoluteEncoder().getPosition(), null);

            builder.addDoubleProperty("Angle (degrees)", () -> Degrees.of(pivot.getAbsoluteEncoder().getPosition()).in(Degrees), null);
            builder.addDoubleProperty("Target Angle (degrees)", () -> pivotController.getMAXMotionSetpointPosition(), null);

            builder.addDoubleProperty("kP", null, value -> {
                kP = value;
            });

             builder.addDoubleProperty("kI", null, value -> {
                kI = value;

            });

              builder.addDoubleProperty("kD", null, value -> {
                kD = value;
            });

        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);




        //builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);

        builder.addDoubleProperty("Pivot Supply Current", () -> pivot.getOutputCurrent(), null);


        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
