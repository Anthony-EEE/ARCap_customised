using UnityEngine;

/// <summary>
/// Configuration parameters for UR3 robot arm
/// Contains UR3-specific constants for joint limits, rest poses, and robot properties
/// </summary>
public static class UR3Config
{
    // UR3 6-DOF robot parameters
    public static readonly int DOF = 6;
    public static readonly string ROBOT_TYPE = "ur3";
    public static readonly string END_EFFECTOR_LINK = "tool0";
    
    // UR3 rest pose (home position) in radians
    // [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    public static readonly float[] REST_POSE = {0.0f, -1.5708f, 0.0f, -1.5708f, 0.0f, 0.0f};
    
    // Joint limits in radians (UR3 specifications)
    public static readonly float[] JOINT_MIN = {-6.2832f, -6.2832f, -3.1416f, -6.2832f, -6.2832f, -6.2832f};
    public static readonly float[] JOINT_MAX = {6.2832f, 6.2832f, 3.1416f, 6.2832f, 6.2832f, 6.2832f};
    
    // Joint names for Unity GameObject references
    public static readonly string[] JOINT_NAMES = {
        "shoulder_pan_joint",
        "shoulder_lift_joint", 
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
    
    // Link names for Unity GameObject references
    public static readonly string[] LINK_NAMES = {
        "base_link",
        "shoulder_link",
        "upper_arm_link", 
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0"
    };
    
    // Unity GameObject names for UR3 robot hierarchy
    // These will be the actual GameObject names in the Unity scene
    public static readonly string UR3_BASE_OBJECT = "ur3_link0_vis";
    public static readonly string UR3_END_EFFECTOR_OBJECT = "ur3_tool0_vis";
    public static readonly string UR3_ROBOT_ROOT = "ur3_vis";
    
    // Joint damping for PyBullet simulation
    public static readonly float[] JOINT_DAMPING = {1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // Coordinate frame offsets (if needed for UR3 vs Panda differences)
    public static readonly Vector3 UR3_POS_OFFSET = Vector3.zero;
    public static readonly Vector3 UR3_ROT_OFFSET = Vector3.zero;
    
    /// <summary>
    /// Check if a robot type is UR3
    /// </summary>
    /// <param name="robotType">Robot type string</param>
    /// <returns>True if robot is UR3</returns>
    public static bool IsUR3(string robotType)
    {
        return robotType.ToLower() == "ur3";
    }
    
    /// <summary>
    /// Get expected DOF for a robot type
    /// </summary>
    /// <param name="robotType">Robot type string</param>
    /// <returns>Expected degrees of freedom</returns>
    public static int GetExpectedDOF(string robotType)
    {
        return IsUR3(robotType) ? DOF : 7; // Panda has 7 DOF
    }
    
    /// <summary>
    /// Get rest pose for a robot type
    /// </summary>
    /// <param name="robotType">Robot type string</param>
    /// <returns>Rest pose joint angles</returns>
    public static float[] GetRestPose(string robotType)
    {
        if (IsUR3(robotType))
        {
            return REST_POSE;
        }
        else
        {
            // Panda rest pose (7 DOF)
            return new float[] {0.0f, -0.498f, -0.02f, -2.473f, -0.013f, 2.004f, -0.723f};
        }
    }
}