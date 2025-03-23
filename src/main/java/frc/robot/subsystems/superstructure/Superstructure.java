package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.Wrist.WristState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;

public class Superstructure {

    public enum SuperState {
        Handoff(0, ElevatorState.Handoff, WristState.Handoff),
        Level2(1, ElevatorState.Level2, WristState.LevelNormal),
        Level3(2, ElevatorState.Level3, WristState.LevelNormal),
        Level4(3, ElevatorState.Level4, WristState.Level4);

        public final int idx;
        public final ElevatorState elevator;
        public final WristState wrist;

        private SuperState(int idx, ElevatorState elevator, WristState wrist) {
            this.idx = idx;
            this.elevator = elevator;
            this.wrist = wrist;
        }
    }
    
}
