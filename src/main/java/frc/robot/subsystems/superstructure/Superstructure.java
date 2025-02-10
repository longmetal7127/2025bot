package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;

public class Superstructure {

    public enum SuperState {
        Handoff(0, ElevatorState.Handoff, ArmState.Handoff),
        Level2(1, ElevatorState.Level2, ArmState.LevelNormal),
        Level3(2, ElevatorState.Level3, ArmState.LevelNormal),
        Level4(3, ElevatorState.Level4, ArmState.Level4);

        public final int idx;
        public final ElevatorState elevator;
        public final ArmState arm;

        private SuperState(int idx, ElevatorState elevator, ArmState arm) {
            this.idx = idx;
            this.elevator = elevator;
            this.arm = arm;
        }
    }
    
}
