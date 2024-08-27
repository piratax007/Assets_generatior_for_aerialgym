import os

dae_file_path = 'meshes/pine_tree.dae'

urdf_file_path = './pine.urdf'

dae_file_name = os.path.basename(dae_file_path)

urdf_content = f"""<?xml version="1.0"?>
<robot name="pine_tree_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="{dae_file_name}" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="{dae_file_name}" />
      </geometry>
    </collision>
  </link>
</robot>
"""

with open(urdf_file_path, 'w', encoding='utf-8') as urdf_file:
    urdf_file.write(urdf_content)

print(f"URDF file created: {urdf_file_path}")
