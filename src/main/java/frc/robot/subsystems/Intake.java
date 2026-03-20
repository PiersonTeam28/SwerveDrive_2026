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

import java.security.spec.EncodedKeySpec;
import java.util.function.DoubleSupplier;



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
        INTAKE(0.5);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }
    // probably no need for these angles, just starting up and going down to intake at start of match. Intake could move a bit to move balls around if they get stuck? so the agitate command
   
    private static final double kPivotReduction = 50.0;
    
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final AngularVelocity kMaxPivotVelocity = Constants.kPivotRPM.div(kPivotReduction); //change from Kraken to vex 775 pro motor, GB Ratio 4:1
    private static final Angle kPositionTolerance = Degrees.of(5);

    

    private static final double SLOWL = 1;

   

    private final TalonFX rollerMotor;
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);


    public Intake() {                              
                
        rollerMotor = new TalonFX(Constants.kIntakeRollers, Constants.kRoboRioCANBus);
        
        configureRollerMotor();
              
        SmartDashboard.putData(this);
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
                    .withStatorCurrentLimit(Amps.of(Constants.INTAKE_STATOR))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.INTAKE_SUPPLY))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
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
               // set(Position.INTAKE.angle().in(Degrees));
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)
        );
    }

   
   


  

   
   
    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);


    //     builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
    //     builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    // }
}
