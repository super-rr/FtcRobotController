package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.StateMachine;

@Autonomous(name = "Auto Far", group = "Auto")
public class Far extends BaseAuto {
    @Override protected StateMachine.State getHomeState()  { return StateMachine.State.AUTO_HOME_FAR; }
    @Override protected StateMachine.State getStartState() { return StateMachine.State.AUTO_FAR; }
}