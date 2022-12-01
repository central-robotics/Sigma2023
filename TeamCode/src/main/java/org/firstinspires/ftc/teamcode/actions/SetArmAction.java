package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class SetArmAction extends Action {

    private double pos;

    public SetArmAction(HardwareManager hardware, double pos) {
        super(hardware);
        this.pos = pos;
    }

    @Override
    public void execute() {
        ArmPositionAction.targetArmPos = pos;
    }
}
