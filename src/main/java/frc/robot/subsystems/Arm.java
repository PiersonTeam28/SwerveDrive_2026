package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
 


import java.security.spec.EncodedKeySpec;
import java.util.function.DoubleSupplier;


//SPARK MAX IMPORTS
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.encoder.*;
import com.revrobotics.encoder.config.*;
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

public class Arm extends SubsystemBase {
    
    // probably no need for these angles, just starting up and going down to intake at start of match. Intake could move a bit to move balls around if they get stuck? so the agitate command
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

    
    private final SparkMax pivot;
    private final SparkClosedLoopController pivotController;
    //private final AbsoluteEncoder pivotEncoder;
   // private final AbsoluteEncoderConfig pivotEncoderConfigAbs;
   // private final EncoderConfig pivotEncoderConfig;
    //private final MAXMotionConfig maxMotionConfig;







    private boolean isHomed = false;

    private boolean invertPivot = false;

    public Arm() {                               // PIVOT MOTOR IS A SPARKMAX MOTOR
        
        pivot = new SparkMax(Constants.kIntakePivot, MotorType.kBrushless); //SPARKMAX 29
        
       
        
        //pivotEncoderConfig = new EncoderConfig();


       // maxMotionConfig = new MAXMotionConfig();

        
        //pivotMotor = new TalonFX(Constants.kIntakePivot, Constants.kCANivoreCANBus); 
       
        
       
       
        configureSparkMaxPivot();

        pivotController = pivot.getClosedLoopController();
       
        SmartDashboard.putData(this);
    }

    private void configureSparkMaxPivot() {
        final SparkMaxConfig pivotConfig = new SparkMaxConfig();

        
        
        pivotConfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .inverted(invertPivot);

        
        
       // pivotConfig.absoluteEncoder.apply(pivotEncoderConfig);



        // pivotConfig.closedLoop.maxMotion
        //     .cruiseVelocity(kMaxPV, ClosedLoopSlot.kSlot0)
        //     .maxAcceleration(kMaxPA, ClosedLoopSlot.kSlot0)
        //     .allowedProfileError(kAllowedError, ClosedLoopSlot.kSlot0);

        // pivotConfig.closedLoop
        //     .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

        // pivotConfig.closedLoop.feedForward
        //     .kA(kA, ClosedLoopSlot.kSlot0)
        //     .kS(kS, ClosedLoopSlot.kSlot0)
        //     .kV(kV, ClosedLoopSlot.kSlot0);


        
       
       
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

    }

   


    // Checks if the pivot is within the position tolerance of the target position TALON
    // Changed to SparkMAX
    // private boolean isPositionWithinTolerance() {
    //     //final Angle currentPosition = pivotMotor.getPosition().getValue();
    //     final Angle currentPos = Degrees.of(pivot.getAbsoluteEncoder().getPosition());
    //     //pivot.getAbsoluteEncoder().getPosition();
    //    // pivotController.getMAXMotionSetpointPosition();
    //     //final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    //     final Angle targetPos = Degrees.of(pivotController.getMAXMotionSetpointPosition());

    
    //     return currentPos.isNear(targetPos, kPositionTolerance);
    // }

   

    private void setPivotPercentOutput(double percentOutput) {
        pivot.setVoltage(Volts.of(percentOutput * 12.0));
        //pivot.set(percentOutput);
    }

   

    // public void set(double setpoint){
    //     pivotController.setSetpoint(setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    // }

   
    public void testSetPivotPercentOutput(double percentOutput) {
        pivot.set(percentOutput);
    }

    public Command pivotCommand(DoubleSupplier speedSupplier) {
        return run(() -> testSetPivotPercentOutput(speedSupplier.getAsDouble()));
    }


   



    

   
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);

        // builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);

       // builder.addDoubleProperty("Alt Encoder Position", () -> pivot.getAlternateEncoder().getPosition(), null);
        //builder.addDoubleProperty("Primary Encoder Position", () -> pivot.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Absolute Encoder Position", () -> pivot.getAbsoluteEncoder().getPosition(), null);



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

       
        builder.addDoubleProperty("Pivot Supply Current", () -> pivot.getOutputCurrent(), null);


    }
}
