package frc.handlers;

public class RobotStates {

        public enum RobotState {
            DISABLED,
            AUTONOMOUS,
            TELEOP,
            TEST
        }
    
        private RobotState currentState = RobotState.DISABLED;
    
        public void setState(RobotState newState) {
            currentState = newState;
        }
    
        public RobotState getCurrentState() {
            return currentState;
        }

        public enum ShooterState {
            IDLE,
            SPINNING_UP,
            SHOOTING
        }    
}
