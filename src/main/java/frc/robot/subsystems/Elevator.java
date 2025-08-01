package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.RobotContainer;
import frc.robot.Constants.ELevatorConstants;

public class Elevator extends SubsystemBase{
    
    private final SparkMax elevatorMotor;

    private final RelativeEncoder encoder;

    private double setpoint;
    private double maxHeight = 92.837;
    private double minHeight = -38.285;


    private PIDController pidController;

    public Elevator(){
        
        elevatorMotor  = new SparkMax(ELevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        encoder = elevatorMotor.getEncoder();
        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(0.05);

        setpoint = 0.0;
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.signals.primaryEncoderPositionPeriodMs(5); // Faster encoder updates
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void SetPos(double Pos){
        setpoint = Pos;
    }
    //U ses triggers to move up and down
    // Counter clockwise is up
    public void MoveUp(double move){
        setpoint += move;
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
    }
    public void MoveDown(double move){
        setpoint -= move;
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    @Override
    public void periodic(){
        double position = encoder.getPosition();
        SmartDashboard.putNumber("Elevator Postition: ", position);
        SmartDashboard.putBoolean("Right Trigger: ", RobotContainer.getRightBumper());
        double speed = pidController.calculate(position, setpoint);
        elevatorMotor.set(speed);
        
        if(RobotContainer.getLeftBumper()){
            MoveDown(0.5);
        }
        else if(RobotContainer.getRightBumper()){
            MoveUp(0.5);
        }

    }
}

// I hate richard thomas mitchell