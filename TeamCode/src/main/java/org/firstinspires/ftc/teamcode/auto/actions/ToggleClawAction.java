package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class ToggleClawAction extends Action {

    HardwareManager manager;
    boolean isClosed;

    public ToggleClawAction(HardwareManager manager)
    {
        super(manager);
        this.manager = manager;
    }

    public void execute()
    {
        isClosed = !isClosed;
        if (isClosed)
            manager.accessoryMotors[1].setPower(0.1);
        else
        {
            long time = System.currentTimeMillis();
            while (System.currentTimeMillis() - time < 2000)
                manager.accessoryMotors[1].setPower(-0.15);
            manager.accessoryMotors[1].setPower(0);
        }
    }
}
