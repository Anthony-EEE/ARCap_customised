import open3d as o3d
import os
import glob

def convert_stl_to_obj(directory):
    stl_files = glob.glob(os.path.join(directory, '*.stl'))

    if not stl_files:
        print("No .stl files found in the directory.")
        return

    for stl_file in stl_files:
        mesh = o3d.io.read_triangle_mesh(stl_file)
        if not mesh.is_empty():
            obj_filename = os.path.splitext(stl_file)[0] + '.obj'
            success = o3d.io.write_triangle_mesh(obj_filename, mesh)
            if success:
                print(f"Converted: {stl_file} to {obj_filename}")
            else:
                print(f"Failed to convert: {stl_file}")
        else:
            print(f"Empty or invalid mesh: {stl_file}")

if __name__ == "__main__":
    directory = "tri_leap_hand"
    convert_stl_to_obj(directory)