package org.firstinspires.ftc.teamcode.auto.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class UpdateDashboardAction extends ContinuousAction {
    public LocalizationEngine localization;
    private long lastExecuted = 0;
    public UpdateDashboardAction(HardwareManager hardware, LocalizationEngine localization) {
        super(hardware);
        this.localization = localization;
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - lastExecuted > 250) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", localization.currentPosition.x);
            packet.put("y", localization.currentPosition.y);
            packet.put("t", localization.currentPosition.t);
            packet.put("status", "alive");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastExecuted = System.currentTimeMillis();
        }
    }

    @Override
    public void initialize() {
    }
}
