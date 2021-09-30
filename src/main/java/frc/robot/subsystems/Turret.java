package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Turret extends SubsystemBase {
    
    // TODO remove sim vars
    private static int simPosition;
    private static boolean movingUp;
    private final int maxPos;
    private final int minPos;

    public Turret () {

        simPosition = 50;
        movingUp = true;
        maxPos = 100;
        minPos = 0;

        // Instantiate electronics (actuators, sensors)

    }

    /**
     * Called periodically during teleop
     */
    public void teleopPeriodic () {
        // TODO maybe move this to a dedicated controller class?

        // "Sine" wave simulator
        if (!Robot.limelight.hasTarget()) {
            if (simPosition == maxPos) {
                movingUp = false;
            } else if (simPosition == minPos) {
                movingUp = true;
            }

            if (movingUp) {
                simPosition++;
            } else {
                simPosition--;
            }


        }

        System.out.println("Simulated position: " + simPosition);
        SmartDashboard.putNumber("'Aim' position", simPosition);

    }

}
