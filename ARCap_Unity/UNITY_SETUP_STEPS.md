# Unity Editor UR3 Setup - Step by Step

## Step 1: Install URDF Importer Package

1. **Open Unity Editor** with your ARCap project
2. **Window** → **Package Manager**
3. Click **"+"** button (top-left)
4. Select **"Add package from git URL"**
5. Enter: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
6. Click **"Add"**
7. Wait for package to install

## Step 2: Import UR3 Robot

1. **Assets** → **Import Robot from URDF**
2. **Browse** → Navigate to: `Assets/Custom/Models/ur3_arm/ur3.urdf`
3. **Import Settings:**
   - ✅ **Create articulation body**: Checked
   - ✅ **Create mesh colliders**: Checked  
   - ✅ **Import materials**: Checked
   - **Mass**: 1.0
   - **Root object name**: `ur3_vis`
4. Click **"Import Robot"**

## Step 3: Configure UR3 Hierarchy

After import, you should see a new `ur3_vis` GameObject in your Hierarchy. 

**CRITICAL: Rename objects to match code expectations:**

1. Expand `ur3_vis` hierarchy
2. Find and rename these objects:
   - **Base link** (usually `base_link`) → Rename to: `ur3_link0_vis`
   - **End effector** (usually `tool0` or `wrist_3_link`) → Rename to: `ur3_tool0_vis`
   - Root should already be named `ur3_vis` (keep as-is)

## Step 4: Open GripperCollection Scene

1. **File** → **Open Scene**
2. Navigate to: `Assets/Custom/Scenes/GripperCollection.unity`
3. Click **"Open"**

## Step 5: Add UR3 to Scene

1. **Drag** the `ur3_vis` prefab from Project window into Scene Hierarchy
2. **Position** the UR3 robot:
   - Same location as existing Panda robot
   - Usually at coordinate frame position
3. **Optional**: Temporarily disable Panda robot:
   - Find `panda_vis` GameObject in hierarchy
   - Uncheck the checkbox next to its name

## Step 6: Add JointController Component

1. **Select** the `ur3_vis` GameObject in hierarchy
2. **Inspector** → **Add Component**
3. Search for: `JointController` (or `control_joints`)
4. **Configure JointController:**
   - **Robot Type**: `ur3`
   - **Local IP**: `192.168.1.100` (or your network IP)
   - **Client Port**: `12345`
   - **Data Collector**: ✅ Checked
   - **Joints**: Need to populate with 6 joint objects (see next step)

## Step 7: Configure Joint List

You need to add the 6 UR3 joints to the Joints list:

1. **In JointController component**, expand **"Joints"** list
2. **Set Size**: `6`
3. **Drag these GameObjects** (in order) from ur3_vis hierarchy:
   - Element 0: `shoulder_pan_joint` object
   - Element 1: `shoulder_lift_joint` object  
   - Element 2: `elbow_joint` object
   - Element 3: `wrist_1_joint` object
   - Element 4: `wrist_2_joint` object
   - Element 5: `wrist_3_joint` object

4. **Set Target Positions** (Size: 6):
   - Element 0: `0`
   - Element 1: `-1.5708`
   - Element 2: `0`
   - Element 3: `-1.5708`
   - Element 4: `0`
   - Element 5: `0`

## Step 8: Configure MainDataRecorderGripper

1. **Find** the GameObject with `MainDataRecorderGripper` component
   - Usually named something like "Main Camera" or "OVRCameraRig"
2. **In Inspector**, find **MainDataRecorderGripper** component
3. **Set Robot Type**: `ur3`
4. **Set Handedness**: `L` (for left controller)
5. **Verify network settings** match your IP configuration

## Step 9: Test Scene Setup

1. **Click Play** button in Unity
2. **Check Console** for any errors:
   - Should see "Socket connected"
   - Should see "Object found"
   - Should NOT see "UR3 objects not found"

If you see errors, double-check GameObject names match exactly:
- `ur3_link0_vis`
- `ur3_tool0_vis` 
- `ur3_vis`

## Step 10: Test with Python Backend

1. **Keep Unity running** in Play mode
2. **Open terminal/command prompt**
3. **Navigate to project**: `cd C:\Users\11424\Documents\ARCap_customised\data_processing`
4. **Run command**: `python data_collection_server.py --robot ur3 --handedness left --no_camera`

**Expected output:**
```
🤖 Initializing UR3 arm (6-DOF, no gripper) for trajectory tracking...
✅ Successfully loaded UR3 URDF from: ../urdf_files_dataset/...
UR3 arm initialized with 6 DOF, end-effector link: 8
Initialization completed
```

## Step 11: Test VR Integration (if you have Quest 3)

1. **Put on Quest 3 headset**
2. **Use left controller** to move around
3. **Verify**: UR3 robot in Unity should follow controller movement
4. **Test buttons**:
   - **A button**: Start/stop recording (should see recording indicator)
   - **B button**: Delete trajectory
   - **No gripper controls** (UR3 is arm-only)

## Troubleshooting

**Problem**: "UR3 objects not found" error
**Solution**: Check GameObject names exactly match:
- `ur3_link0_vis`, `ur3_tool0_vis`, `ur3_vis`

**Problem**: No joint movement
**Solution**: Verify joints list has 6 ArticulationBody objects

**Problem**: Network connection failed  
**Solution**: Check IP addresses match between Unity and Python

**Problem**: URDF Importer not found
**Solution**: Reinstall package, make sure Unity 2022.3+ and URP

## Success Indicators

✅ UR3 robot visible in Unity Scene view
✅ Python server connects without errors  
✅ Joint movements smooth and responsive
✅ VR controller controls robot end-effector
✅ Recording functionality works (A/B buttons)
✅ No gripper controls (arm-only operation)

When all these work, your UR3 integration is complete!