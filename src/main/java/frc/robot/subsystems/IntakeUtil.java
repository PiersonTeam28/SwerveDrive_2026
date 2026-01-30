package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeUtil extends SubsystemBase {
    private SparkMax intakeMotor;

    public IntakeUtil(){
        intakeMotor = new SparkMax(0, null);
    }
}
