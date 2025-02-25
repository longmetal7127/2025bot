package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Superstructure;

public class Take extends SubsystemBase {
    private SparkMax takeMotor = new SparkMax(Superstructure.CANIds.kTakeMotorCanId, MotorType.kBrushless);

    public Take() {

    }

    public Command runTakeMotor() {
        return startEnd(() -> {
            takeMotor.set(0.5);
        }, () -> {
            takeMotor.set(0);
        });
    }
    public Command runTakeMotorReverse() {
        return startEnd(() -> {
            takeMotor.set(-0.5);
        }, () -> {
            takeMotor.set(0);
        });
    }


}
