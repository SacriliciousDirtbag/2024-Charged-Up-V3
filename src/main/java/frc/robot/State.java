package frc.robot;

public class State {

    // horizontal carriage state
    public enum HState {
        HOME,
        LOAD_LOW,
        LOAD_CONE,
        RAMP_CUBE,
        RAMP_CONE,
        SCORE_CUBE_LOW,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CONE_HIGH,
        EXTENDED
    };

    // vertical lift state
    public enum VState {
        HOME,
        LOAD_LOW,
        LOAD_CONE,
        RAMP_CUBE,  // load from
        RAMP_CONE,  // load from
        SCORE_CUBE_LOW,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CONE_HIGH,
    };

    // intake flipper state
    public enum FState {
        HOME,
        LOAD_LOW,
        LOAD_CONE,
        RAMP_CUBE,
        RAMP_CONE,
        SCORE_CUBE_LOW,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CONE_HIGH,
        FLIPPED
    };

    // intake spinner state
    public enum SState {
        HOME,
        STOP,
        IN,
        OUT
    }; 
}
