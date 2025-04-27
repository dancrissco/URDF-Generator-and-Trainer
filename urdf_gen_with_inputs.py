# urdf_gen_with_inputs.py
# Final clean version: Rule-based wheel and caster placement

def create_base_link(base_x, base_y, base_z):
    return f"""
    <link name="base_link">
        <visual>
            <geometry>
                <box size="{base_x} {base_y} {base_z}"/>
            </geometry>
            <material name="yellow">
                <color rgba="1.0 1.0 0.0 1.0"/>
            </material>
        </visual>
    </link>
    """

def create_wheel(side, wheel_radius, wheel_length, wheel_offset_y, base_x):
    wheel_offset_x = -0.3 * base_x  # Move wheels back based on 30% of base length

    if side == 'left':
        y_offset = wheel_offset_y
        rpy = "-1.5708 0 0"
        axis = "0 0 1"
    else:
        y_offset = -wheel_offset_y
        rpy = "1.5708 0 0"
        axis = "0 0 -1"

    color = 'blue' if side == 'left' else 'red'
    rgba = '0.0 0.0 1.0 1.0' if side == 'left' else '1.0 0.0 0.0 1.0'

    return f"""
    <joint name="{side}_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="{side}_wheel_link"/>
        <origin xyz="{wheel_offset_x} {y_offset} 0" rpy="{rpy}"/>
        <axis xyz="{axis}"/>
    </joint>

    <link name="{side}_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="{wheel_radius}" length="{wheel_length}"/>
            </geometry>
            <material name="{color}">
                <color rgba="{rgba}"/>
            </material>
        </visual>
    </link>
    """

def create_caster(base_x, base_z, caster_radius):
    caster_offset_x = 0.3 * base_x  # Same 30% backward shift
    caster_z = -(base_z / 2) - caster_radius + (caster_radius / 2)  # Corrected height for caster

    return f"""
    <joint name="caster_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_link"/>
        <origin xyz="{caster_offset_x} 0 {caster_z}" rpy="0 0 0"/>
    </joint>

    <link name="caster_link">
        <visual>
            <geometry>
                <sphere radius="{caster_radius}"/>
            </geometry>
            <material name="gray">
                <color rgba="0.6 0.6 0.6 1.0"/>
            </material>
        </visual>
    </link>
    """

def generate_urdf(base_x, base_y, base_z, wheel_radius, wheel_length, wheel_offset_y, include_caster, caster_radius):
    urdf = '<?xml version="1.0"?>\n'
    urdf += '<robot name="simple_diff_drive_generated" xmlns:xacro="http://ros.org/wiki/xacro">\n'
    
    urdf += create_base_link(base_x, base_y, base_z)
    urdf += create_wheel('left', wheel_radius, wheel_length, wheel_offset_y, base_x)
    urdf += create_wheel('right', wheel_radius, wheel_length, wheel_offset_y, base_x)
    
    if include_caster:
        urdf += create_caster(base_x, base_z, caster_radius)
    
    urdf += '</robot>\n'
    return urdf

def save_urdf(filename, urdf_content):
    with open(filename, 'w') as f:
        f.write(urdf_content)
    print(f"✅ URDF saved as {filename}")

def main():
    print("=== Simple Diff Drive URDF Generator ===")
    base_x = float(input("Enter base link X size (default 0.3): ") or 0.3)
    base_y = float(input("Enter base link Y size (default 0.2): ") or 0.2)
    base_z = float(input("Enter base link Z size (default 0.05): ") or 0.05)

    wheel_radius = float(input("Enter wheel radius (default 0.05): ") or 0.05)
    wheel_length = float(input("Enter wheel width (default 0.04): ") or 0.04)
    wheel_offset_y = float(input("Enter wheel Y offset from center (default 0.15): ") or 0.15)

    include_caster_input = input("Add caster wheel? (yes/no) (default yes): ").lower() or "yes"
    include_caster = True if include_caster_input in ['yes', 'y'] else False

    caster_radius = float(input("Enter caster radius (default 0.03): ") or 0.03)

    filename = input("Enter output URDF filename (default generated_robot.urdf): ") or "generated_robot.urdf"

    urdf_content = generate_urdf(
        base_x, base_y, base_z,
        wheel_radius, wheel_length, wheel_offset_y,
        include_caster, caster_radius
    )

    save_urdf(filename, urdf_content)

if __name__ == "__main__":
    main()
