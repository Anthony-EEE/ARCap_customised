# URDF Importer Installation Fix

## The Error
The git URL `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer` is failing.

## Solution Options

### Option 1: Try Alternative Git URL
1. **Close the current Package Manager error**
2. **Try this URL instead**: `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.urdf-importer`
3. **Or try**: `com.unity.robotics.urdf-importer`

### Option 2: Manual Package Installation
1. **Download the package manually**:
   - Go to: https://github.com/Unity-Technologies/URDF-Importer
   - Click "Code" → "Download ZIP"
   - Extract to: `Packages/com.unity.robotics.urdf-importer/`

### Option 3: Use Built-in Unity 2022+ URDF Support
Unity 2022.3+ has built-in URDF support:
1. **Assets** → **Import** → **URDF**
2. If this menu exists, you don't need the package!

### Option 4: Skip URDF Importer (Manual Setup)
We can manually configure the UR3 robot:
1. Create empty GameObjects for robot hierarchy
2. Import meshes manually from `Assets/Custom/Models/ur3_arm/meshes/`
3. Set up ArticulationBody components manually

## Recommended Next Steps
Try Option 1 first, then Option 3, then Option 4 if needed.