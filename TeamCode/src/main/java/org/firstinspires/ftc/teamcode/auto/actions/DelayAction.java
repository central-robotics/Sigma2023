package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class DelayAction extends Action {

    long delay;

    public DelayAction(HardwareManager manager, long delay)
    {
        super(manager);
        this.delay = delay;
    }

    public void execute()
    {
        delay += System.currentTimeMillis();
        while(System.currentTimeMillis() < delay)
        {}
    }
}
