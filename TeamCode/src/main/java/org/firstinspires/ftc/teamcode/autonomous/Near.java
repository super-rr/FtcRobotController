package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.StateMachine;

@Autonomous(name = "Auto Near", group = "Auto")
public class Near extends BaseAuto {
    @Override protected StateMachine.State getHomeState()  { return StateMachine.State.AUTO_HOME_NEAR; }
    @Override protected StateMachine.State getStartState() { return StateMachine.State.AUTO_NEAR; }
}