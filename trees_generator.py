import argparse
import xml.etree.ElementTree as ET
import random

def create_inertial(mass=1.0):
    inertial = ET.Element('inertial')
    ET.SubElement(inertial, 'origin', xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'inertia', ixx="1.0", ixy="0.0", ixz="0.0", iyy="1.0", iyz="0.0", izz="1.0")
    return inertial

def create_visual(name, length, radius, position, orientation):
    visual = ET.Element('visual', name=f"{name}_visual")
    geometry = ET.SubElement(visual, 'geometry')
    cylinder = ET.SubElement(geometry, 'cylinder', radius=str(radius), length=str(length))
    origin = ET.SubElement(visual, 'origin', xyz=f"{position[0]} {position[1]} {position[2]}",
                           rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    material = ET.SubElement(visual, 'material', name="branch_material")
    return visual

def create_collision(name, length, radius, position, orientation):
    collision = ET.Element('collision', name=f"{name}_collision")
    geometry = ET.SubElement(collision, 'geometry')
    cylinder = ET.SubElement(geometry, 'cylinder', radius=str(radius), length=str(length))
    origin = ET.SubElement(collision, 'origin', xyz=f"{position[0]} {position[1]} {position[2]}",
                           rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    return collision

def create_link(name, length, radius, position, orientation):
    link = ET.Element('link', name=name)
    link.append(create_inertial())
    link.append(create_visual(name, length, radius, position, orientation))
    link.append(create_collision(name, length, radius, position, orientation))
    return link

def create_joint(name, parent, child, position, orientation):
    joint = ET.Element('joint', name=name, type="fixed")
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=f"{position[0]} {position[1]} {position[2]}",
                  rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    return joint

def generate_random_tree_fixed_connection_with_altitude(robot, parent_name, parent_length, base_position, base_orientation, level, max_depth, num_branches_range, branch_length_range, branch_radius_range, altitude):
    """
    :param robot: The URDF XML tree to append to.
    :param parent_name: Name of the parent link (e.g., trunk or a branch).
    :param parent_length: Length of the parent link (needed to position branches properly).
    :param base_position: Position of the parent link.
    :param base_orientation: Orientation of the parent link.
    :param level: Current recursion level (used to limit depth).
    :param max_depth: Maximum depth of recursion.
    :param num_branches_range: Range for the random number of branches.
    :param branch_length_range: Range for the random length of branches.
    :param branch_radius_range: Range for the random radius of branches.
    :param altitude: Minimum altitude from which branches should start, relative to the base of the parent branch or trunk.
    """
    if level > max_depth:
        return

    num_branches = random.randint(num_branches_range[0], num_branches_range[1])

    parent_start_position = (base_position[0], base_position[1], base_position[2] - parent_length / 2)
    parent_end_position = (base_position[0], base_position[1], base_position[2] + parent_length / 2)

    for i in range(num_branches):
        branch_length = random.uniform(branch_length_range[0], branch_length_range[1])
        branch_radius = random.uniform(branch_radius_range[0], branch_radius_range[1])

        branch_z_position = random.uniform(parent_start_position[2], parent_end_position[2])

        branch_position = (
            parent_start_position[0],
            parent_start_position[1],
            branch_z_position
        )

        branch_orientation = (
            base_orientation[0] + random.uniform(-0.1, 0.1),
            base_orientation[1] + random.uniform(-0.1, 0.1),
            base_orientation[2] + random.uniform(-3.14, 3.14),
        )

        branch_name = f"{parent_name}_branch_{level}_{i}"

        branch_link = create_link(branch_name, branch_length, branch_radius, branch_position, branch_orientation)
        robot.append(branch_link)

        joint_name = f"joint_{parent_name}_to_{branch_name}"
        joint = create_joint(joint_name, parent_name, branch_name, branch_position, branch_orientation)
        robot.append(joint)

        generate_random_tree_fixed_connection_with_altitude(robot, branch_name, branch_length, branch_position, branch_orientation, level + 1, max_depth, num_branches_range, branch_length_range, branch_radius_range, altitude)

def generate_tree_urdf_with_altitude(trunk_length, trunk_radius, max_depth, num_branches_range, branch_length_range, branch_radius_range, altitude):
    robot = ET.Element('robot', name="tree_0.urdf")

    trunk_position = (0, 0, trunk_length / 2)
    trunk_orientation = (0, 0, 0)
    trunk_link = create_link("trunk", trunk_length, trunk_radius, trunk_position, trunk_orientation)
    robot.append(trunk_link)

    generate_random_tree_fixed_connection_with_altitude(
        robot,
        "trunk",
        trunk_length,
        trunk_position,
        trunk_orientation,
        level=1,
        max_depth=max_depth,
        num_branches_range=num_branches_range,
        branch_length_range=branch_length_range,
        branch_radius_range=branch_radius_range,
        altitude=altitude
    )

    return robot


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a random tree URDF file.")
    parser.add_argument("--trunk_length", type=float, default=5.0, help="Length of the trunk.")
    parser.add_argument("--trunk_radius", type=float, default=0.9, help="Radius of the trunk.")
    parser.add_argument("--max_depth", type=int, default=1, help="Maximum depth of the tree.")
    parser.add_argument("--num_branches_min", type=int, default=1, help="Minimum number of branches per node.")
    parser.add_argument("--num_branches_max", type=int, default=2, help="Maximum number of branches per node.")
    parser.add_argument("--branch_length_min", type=float, default=1.0, help="Minimum length of branches.")
    parser.add_argument("--branch_length_max", type=float, default=2.0, help="Maximum length of branches.")
    parser.add_argument("--branch_radius_min", type=float, default=0.5, help="Minimum radius of branches.")
    parser.add_argument("--branch_radius_max", type=float, default=0.75, help="Maximum radius of branches.")

    args = parser.parse_args()

    tree_robot = generate_tree_urdf_with_altitude(
        trunk_length=args.trunk_length,
        trunk_radius=args.trunk_radius,
        max_depth=args.max_depth,
        num_branches_range=(args.num_branches_min, args.num_branches_max),
        branch_length_range=(args.branch_length_min, args.branch_length_max),
        branch_radius_range=(args.branch_radius_min, args.branch_radius_max),
        altitude=2
    )

    urdf_str = ET.tostring(tree_robot, encoding='unicode')

    with open('random_tree_2.urdf', 'w', encoding='utf-8') as f:
        f.write(urdf_str)

    print("Random tree URDF generated and saved as 'random_tree_2.urdf'.")
