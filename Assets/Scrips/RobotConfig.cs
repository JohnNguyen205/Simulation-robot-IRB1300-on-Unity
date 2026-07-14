using UnityEngine;

public static class RobotConfig
{
    public static readonly float[] LowerLimits = { -180, -95, -210, -230, -130, -400 };
    public static readonly float[] UpperLimits = { 180, 155, 65, 230, 130, 400 };

    public const float REACH_MAX = 900f;
    public const float REACH_MIN = 200f;
    public const float Z_MIN = 150f;
    public const float Z_MAX = 1450f;
    public const float BASE_HEIGHT = 544f;

    public static readonly float[] HomePosition = { 0f, -30f, 30f, 0f, 60f, 0f };
    public static readonly float[] ZeroPosition = { 0f, 0f, 0f, 0f, 0f, 0f };

    public const int IK_MAX_ITERATIONS = 200;
    public const float IK_TOLERANCE_POS = 1f;
    public const float IK_TOLERANCE_ROT = 1f;
    public const float IK_MOVE_DURATION = 1.5f;

    public const float DRIVE_STIFFNESS = 100000f;
    public const float DRIVE_DAMPING = 10000f;
    public const float DRIVE_FORCE_LIMIT = 10000f;

    public static readonly string[] SliderNames = {
        "Slider", "Slider (1)", "Slider (2)",
        "Slider (3)", "Slider (4)", "Slider (5)"
    };
}