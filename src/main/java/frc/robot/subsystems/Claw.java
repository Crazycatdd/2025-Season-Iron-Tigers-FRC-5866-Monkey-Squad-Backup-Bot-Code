package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants.ClawConstants;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase{
    private final SparkMax clawMotor;

    private final RelativeEncoder encoder;

    private double setpoint;
    private double max = -19.19;
    private double min = 1.5;//0.5

    private PIDController pidController;

    public Claw(){
        SparkMaxConfig config = new SparkMaxConfig();
        clawMotor = new SparkMax(ClawConstants.CLAW_MOTOR_ID, MotorType.kBrushless);
        config.signals.primaryEncoderPositionPeriodMs(5); // Faster encoder updates
        clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = clawMotor.getEncoder();
        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(0.05);
        setpoint = 0.0;
        //im so glad its working
    }
    public void SetPos(double pos){
        setpoint = pos;
    }
    public void ClawMove(double move){
        setpoint += move;
        if(setpoint > min){
            setpoint = min;
        }
        else if(setpoint < max){
            setpoint = max;
        }
    }
    @Override
    public void periodic() {
        double position = encoder.getPosition();
        SmartDashboard.putNumber("Claw Position: ", position);
        double speed = pidController.calculate(position, setpoint);
        clawMotor.set(speed * 0.25);
        if(RobotContainer.getRightYValue() > 0.1 || RobotContainer.getRightYValue() < -0.1){
            ClawMove(RobotContainer.getRightYValue());
        }
    }
}
//maassimo is a great guy