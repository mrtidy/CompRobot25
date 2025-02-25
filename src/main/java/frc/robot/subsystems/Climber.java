package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The climber currently consists of a single, brushed motor connected to a SparkMax.  There
 * isn't any encoder or endstops attached at the moment.
 * 
 * The behavior is initially setup up for pushing the top button on the pad makes the motor
 * climb and the down button attempts to go downward.  We'll put the motor in brake motor
 * when the robot is enabled hopefully to make the drop slower after a partial climb.
 */
public class Climber extends SubsystemBase {

    // these rates are defined here to make them easy to find and adjust; these are used
    // to set the motor run rate for climbing up or down
    private static final double UP_SPEED = 0.5d;
    private static final double DOWN_SPEED = 0.4d;
    
    private final SparkMax climbMotor;

    public Climber() {
        climbMotor =  new SparkMax(Constants.DeviceConstants.kClimberControllerId, MotorType.kBrushed);

        // we do want to configure braking and current limits
        var config = new SparkMaxConfig();
        config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up() {
        // if we add limit switches, we'll check them here before running motor
        climbMotor.set(UP_SPEED);
    }
    
    public void down() {
        climbMotor.set(-DOWN_SPEED);
    }

    public void stop() {
        climbMotor.stopMotor();
    }
}
