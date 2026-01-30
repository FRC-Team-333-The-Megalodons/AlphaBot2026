package frc.robot.util;

public class RobotStates {
    public static Mode currentMode = Mode.DISABLED;
    public static GamePieceState pieceState = GamePieceState.NONE;
    public static FieldLocation location = FieldLocation.ALLIANCE_ZONE;
    public enum Mode{
        DISABLED,
        TELEOP,
        AUTO,
        TEST
    }
    public enum GamePieceState{
        NONE,
        INTAKING,
        HAS_PIECE,
        SCORING
    }
    public enum FieldLocation{
        ALLIANCE_ZONE,
        NEUTRAL_ZONE,
        OPP_ALLINACE_ZONE
    }
}


