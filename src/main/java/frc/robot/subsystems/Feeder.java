package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.spark.config.SparkParameters.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private double agitSpeed = 0.05;
    
    public enum Speed {
        FEED(5000*0.75),
        AGIT(0.2);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final SparkMax agitatorMotorL;
    private final SparkMax agitatorMotorR;

    private final TalonFX feederMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Feeder() {
        feederMotor = new TalonFX(Constants.kFeeder, Constants.kRoboRioCANBus);

        agitatorMotorL = new SparkMax(Constants.kAgitatorL, SparkBase.MotorType.kBrushless);
        agitatorMotorR = new SparkMax(Constants.kAgitatorR, SparkBase.MotorType.kBrushless);


        final SparkMaxConfig agitatorConfig = new SparkMaxConfig();

        agitatorConfig.smartCurrentLimit(Constants.AGIT_STALL).idleMode(IdleMode.kBrake);
        
        agitatorMotorL.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        agitatorMotorR.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.FEED_STATOR))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.FEED_SUPPLY))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        feederMotor.getConfigurator().apply(config);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        feederMotor.setControl(
            velocityRequest
                .withVelocity(speed.angularVelocity())
        );
    }



    public void stop() {
        feederMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
        agitatorMotorL.set(0);
        agitatorMotorR.set(0);

    }

    public Command stopFeeder() {
        return run(this::stop);
    }

    public void setPercentOutput(double percentOutput) {
        feederMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    // positive percent output on one agitator motor and negative on the other to spin them in opposite directions
    public void setAgitator(double percentOutput) {
        agitatorMotorL.set(-percentOutput);
        agitatorMotorR.set(percentOutput);
    }
    
    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
    }

    public Command agitateCommand() { 
        return startEnd(() -> setAgitator(0.05), () -> setAgitator(0));
    }



    public Command indexCommand() {
        return startEnd(() -> {this.setAgitator(agitSpeed);
        this.set(Speed.FEED);}, () -> {
            this.setPercentOutput(0);
            this.setAgitator(0);
        });
    }

    public Command feedAndAgitateCommand() {
        return indexCommand();
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        
        
       // builder.addDoubleProperty("L Current", () -> agitatorMotorL.getOutputCurrent(), null);
       // builder.addDoubleProperty("R Current", () -> agitatorMotorR.getOutputCurrent(), null);
       // builder.addDoubleProperty("Agit Speed", () -> agitSpeed, value -> agitSpeed = value);

        builder.addDoubleProperty("RPM", () -> feederMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Stator Current", () -> feederMotor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Supply Current", () -> feederMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
