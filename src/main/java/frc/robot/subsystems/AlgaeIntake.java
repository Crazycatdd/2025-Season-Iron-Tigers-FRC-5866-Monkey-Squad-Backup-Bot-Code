package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase{
    public final SparkFlex algaeIntakeMotor;
    public AlgaeIntake(){
        algaeIntakeMotor = new SparkFlex(AlgaeIntakeConstants.ALGAEINTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.signals.primaryEncoderPositionPeriodMs(5); // Faster encoder updates
        algaeIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void Intake(double speed){
        algaeIntakeMotor.set(speed * 0.5);
    }
    public void Eject(double speed){
        algaeIntakeMotor.set(-speed * 0.5);
    }
    public void stop(){
        algaeIntakeMotor.set(0);
    }
    //runs every 10 milisecondss
    @Override
    public void periodic() {
        if(RobotContainer.getRightTrigger() > 0.05){
            Eject(RobotContainer.getRightTrigger());
        }
        else if(RobotContainer.getLeftTrigger() > 0.05){
            Intake(RobotContainer.getLeftTrigger());
        }
        else{
            stop();
        }
    }
}
