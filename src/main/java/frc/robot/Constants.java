package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;


public class Constants {

    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static final double SLOW = 0.66;

    //public static final double INTAKE = 0.5;
    

    // PWM IDS


    public static final int HOOD_L = 0;
    public static final int HOOD_R = 1;
    
    
    // DIO IDS

    // CAN IDS
    // TALON 
    public static final int FRONT_RIGHT_DRIVE = 25;
    public static final int REAR_RIGHT_DRIVE = 55;
    public static final int FRONT_LEFT_DRIVE = 10;
    public static final int REAR_LEFT_DRIVE = 40;

    public static final int FRONT_LEFT_STEER = 22;
    public static final int FRONT_RIGHT_STEER = 20;
    public static final int REAR_LEFT_STEER = 35;
    public static final int REAR_RIGHT_STEER = 50;
    
    public static final int REAR_LEFT_ENCODER = 45;
    public static final int FRONT_LEFT_ENCODER = 15;
    public static final int FRONT_RIGHT_ENCODER = 30;
    public static final int REAR_RIGHT_ENCODER = 60;

    public static final int PIGEONCAWWWWWWWWW = 13;

    //PORTS 

    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 29;
    public static final int kIntakeRollers = 5;
    public static final int kFloor = 12;
    public static final int kFeeder = 13;
    public static final int kShooterLeft = 11;
    //public static final int kShooterMiddle = 15;
    public static final int kShooterRight = 17;
    public static final int kHanger = 28;

}
