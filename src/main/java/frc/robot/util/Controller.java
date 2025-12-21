package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Controller {
    private final XboxController controller;
    private final CommandXboxController commandController;
    private double speedMult = Constants.DriveConstants.kMinDriveMultiplier;

    public Controller(int port) {
        controller = new XboxController(port);
        commandController = new CommandXboxController(port);
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }

    // -------------------------------
    // SWERVE CONTROL
    // -------------------------------
    
    private boolean fieldRelative = true;

    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public void upshift(){
        speedMult += Constants.DriveConstants.kGearIncrement;
        if (speedMult > Constants.DriveConstants.kMaxDriveMultiplier){
            speedMult = Constants.DriveConstants.kMaxDriveMultiplier;
        }
    }

    public void downshift(){
        speedMult -= Constants.DriveConstants.kGearIncrement;
        if (speedMult < 0){
            speedMult = 0;
        }
    }

    /** Get forward/backward, with deadband applied */
    public double getDriveX() {
        return applyDeadband(-controller.getLeftY()) * speedMult;
    }

    /** Get strafe, with deadband applied */
    public double getDriveY() {
        return applyDeadband(-controller.getLeftX()) * speedMult;
    }

    /** Get rotation, with deadband applied */
    public double getRotation() {
        return applyDeadband(-controller.getRightX()) * Constants.DriveConstants.kRotationMultiplier;
    }

    private double applyDeadband(double value) {
        double deadband = 0.1;
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public void bind() {
        commandController.leftStick().onChange(new InstantCommand(this::toggleFieldRelative));
        commandController.leftBumper().onTrue(new InstantCommand(this::downshift));
        commandController.rightBumper().onTrue(new InstantCommand(this::upshift));
    }
}
