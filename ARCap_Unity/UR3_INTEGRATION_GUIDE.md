# 🤖 ARCap UR3 Integration Guide

## 📋 Overview

This guide explains how to complete the ARCap UR3 integration in Unity. The backend Python code is already implemented and working. You need to configure Unity scenes with UR3 robot objects.

## ✅ What's Already Done

- ✅ `QuestUR3ArmModule` - Full 6-DOF UR3 backend implementation  
- ✅ `UR3Config.cs` - UR3 configuration parameters
- ✅ Updated `control_joints.cs` - Variable DOF support (6 for UR3, 7 for Panda)
- ✅ Updated `MainDataRecorderGripper.cs` - UR3 object references and gripper disabling
- ✅ Removed DOF padding from Python backend
- ✅ UR3 URDF imported to `Assets/Custom/Models/ur3_arm/`

## 🔧 Unity Scene Configuration Required

### Step 1: Import UR3 URDF as ArticulationBody

1. **Install Unity URDF Importer** (if not already installed)
   - Window → Package Manager → Add package from git URL
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

2. **Import UR3 Robot**
   - Assets → Import Robot from URDF
   - Select: `Assets/Custom/Models/ur3_arm/ur3.urdf`
   - **Import Settings:**
     - ✅ Create Root Object: `ur3_vis`
     - ✅ Use ArticulationBody Components
     - ✅ Use Mesh Colliders
     - ✅ Set Mass: 1.0
     - ✅ Import Materials

3. **Verify Joint Hierarchy**
   Expected structure:
   ```
   ur3_vis (Root)
   └── base_link → ur3_link0_vis
       └── shoulder_link  
           └── upper_arm_link
               └── forearm_link
                   └── wrist_1_link
                       └── wrist_2_link
                           └── wrist_3_link → ur3_tool0_vis
   ```

### Step 2: Configure GameObject Names

**CRITICAL:** Rename GameObjects to match `UR3Config.cs` expectations:

```csharp
// Required GameObject names:
UR3_BASE_OBJECT = "ur3_link0_vis"         // Base link
UR3_END_EFFECTOR_OBJECT = "ur3_tool0_vis" // End effector (tool0)
UR3_ROBOT_ROOT = "ur3_vis"                // Root object
```

**Renaming Steps:**
1. Select imported UR3 base link → Rename to `ur3_link0_vis`
2. Select UR3 end effector (tool0) → Rename to `ur3_tool0_vis` 
3. Root object should already be named `ur3_vis`

### Step 3: Add to GripperCollection Scene

1. **Open Scene:** `Assets/Custom/Scenes/GripperCollection.unity`

2. **Add UR3 Robot:**
   - Drag `ur3_vis` prefab into scene hierarchy
   - Position at coordinate frame location (same as Panda)

3. **Configure JointController Component:**
   - Select `ur3_vis` GameObject
   - Add Component → `JointController` (control_joints.cs)
   - **Inspector Settings:**
     - `Robot Type`: "ur3"
     - `Joints`: Drag 6 UR3 joint GameObjects (shoulder_pan through wrist_3)
     - `Target Positions`: Set to UR3 rest pose: `[0, -1.5708, 0, -1.5708, 0, 0]`
     - `Local IP`: Match network config
     - `Client Port`: 12345
     - `Data Collector`: ✅ true

4. **Configure MainDataRecorderGripper:**
   - Select main data recorder GameObject
   - **Inspector Settings:**
     - `Robot Type`: "ur3"
     - `Handedness`: "L" (for left controller)
     - Keep other network settings as default

### Step 4: Network Protocol Configuration

**Python Server Command:**
```bash
cd data_processing
python data_collection_server.py --robot ur3 --handedness left --no_camera
```

**Expected Network Protocol:**
```
VR → Python: "YLHand:x,y,z,qx,qy,qz,qw,hx,hy,hz,hqx,hqy,hqz,hqw,gripper"
Python → Unity: "Y,j1,j2,j3,j4,j5,j6" (6 DOF for UR3)
```

## 🧪 Testing Steps

### Test 1: Backend Validation
```bash
cd data_processing
python test_ur3_quest_module.py
```
**Expected Output:**
- ✅ UR3 URDF loaded successfully  
- ✅ 6 DOF detected
- ✅ IK convergence > 95%
- ✅ Joint limits respected

### Test 2: Unity Scene Load
1. Open `GripperCollection.unity`
2. Press Play
3. **Check Console for:**
   - ✅ "Socket connected"
   - ✅ "Object found" 
   - ✅ No "UR3 objects not found" errors

### Test 3: Network Communication
1. **Start Python server:**
   ```bash
   python data_collection_server.py --robot ur3 --handedness left --no_camera
   ```

2. **Start Unity scene** (Play mode)

3. **Verify connection:**
   - Python console: "Initialization completed"
   - Unity console: Joint movement data received

### Test 4: VR Integration
1. **Put on Quest 3 headset**
2. **Controls:**
   - Left Controller → UR3 end-effector control
   - A Button → Start/stop recording  
   - B Button → Delete last trajectory
   - **No gripper controls** (UR3 arm only)

3. **Visual Feedback:**
   - UR3 robot follows left controller motion
   - Smooth 6-DOF tracking
   - Recording indicator changes color

### Test 5: Data Collection
1. **Record test trajectory** (A button)
2. **Check data output:**
   ```bash
   ls data/  # Should show ur3_data_timestamp.npz files
   ```
3. **Validate data format:**
   ```python
   import numpy as np
   data = np.load("data/ur3_data_timestamp.npz")
   print(data['ur3_arm_q'].shape)  # Should be (N, 6)
   ```

## 🚨 Troubleshooting

### Common Issues

**1. "UR3 objects not found" Error**
- Solution: Check GameObject naming matches `UR3Config.cs`
- Verify: `ur3_link0_vis`, `ur3_tool0_vis`, `ur3_vis` exist

**2. Joint Control Not Working**
- Check: `Joints` list in JointController has 6 objects
- Verify: All joints have ArticulationBody components
- Ensure: `robotType = "ur3"` in inspector

**3. Network Connection Failed**
- Verify: Python server running with `--robot ur3`
- Check: IP addresses match between Unity and Python
- Ensure: Firewall allows UDP ports 12345, 12346

**4. IK Convergence Issues**
- Solution: Check joint limits in UR3Config.cs
- Verify: Controller position within UR3 workspace
- Try: Restart from UR3 rest pose

**5. Visual Lag/Stuttering**
- Check: Unity runs at 90fps for VR
- Reduce: Mesh complexity if needed
- Optimize: ArticulationBody damping settings

### Debug Tools

**Python Debug:**
```bash
# Test UR3 module standalone
python test_ur3_quest_module.py

# Verbose server output
python data_collection_server.py --robot ur3 --handedness left --no_camera
```

**Unity Debug:**
- Console window for network/object errors
- Scene view to verify robot positioning
- Game view for VR visualization

## 📊 Performance Targets

- **Tracking Accuracy:** < 2cm position error
- **Update Rate:** 30Hz stable  
- **IK Success Rate:** > 95% within workspace
- **VR Latency:** < 50ms controller → robot movement

## 🎯 Success Criteria

- ✅ UR3 robot visible and animating in Unity VR
- ✅ Left controller controls UR3 end-effector smoothly
- ✅ 6-DOF joint data recorded correctly
- ✅ No gripper controls (arm-only operation)
- ✅ Recording/playback workflow functional
- ✅ Data files contain valid UR3 trajectories

## 📁 File Structure After Integration

```
ARCap_Unity/Assets/Custom/
├── Models/
│   ├── ur3_arm/           ✅ Imported URDF
│   └── franka_arm/        (Keep for Panda support)
├── Scripts/
│   ├── UR3Config.cs       ✅ New configuration
│   ├── control_joints.cs  ✅ Updated for variable DOF  
│   └── MainDataRecorderGripper.cs ✅ Updated for UR3
├── Scenes/
│   └── GripperCollection.unity ❗ Needs UR3 objects added
└── Prefabs/
    └── ur3_vis.prefab     ❗ Create from imported URDF
```

---

**🎉 Once completed, you'll have a fully functional UR3 VR data collection system!**

The migration from Panda → UR3 will be complete with proper 6-DOF arm-only trajectory tracking.