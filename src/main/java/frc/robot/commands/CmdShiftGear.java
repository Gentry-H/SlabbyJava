package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubDriveTrain;

public class CmdShiftGear extends CommandBase {
    private SubDriveTrain subDriveTrain;

    public CmdShiftGear(SubDriveTrain subDriveTrain) {
        this.subDriveTrain = subDriveTrain;
        addRequirements(subDriveTrain);
    }

    @Override
    public void initialize() {
        subDriveTrain.toggleDriveTrainGear();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
