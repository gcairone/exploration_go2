import sys

# Parametri dei muri
WALL_SIZE_X = 0.5
WALL_SIZE_Y = 0.5
WALL_SIZE_Z = 1.5
WALL_HEIGHT = WALL_SIZE_Z / 2.0  # metà per posizionarlo sul terreno

def ascii_to_sdf(ascii_map, output_file="model.sdf"):
    rows = ascii_map.strip().splitlines()
    rows = [r.rstrip() for r in rows]

    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="my_custom_world">
    <static>true</static>
"""
    sdf_footer = """  </model>
</sdf>
"""

    links = []
    wall_id = 0

    # La mappa ASCII ha (0,0) in alto a sinistra. 
    # Per Gazebo mettiamo y crescente verso l'alto, x verso destra.
    for row_idx, row in enumerate(rows):
        for col_idx, char in enumerate(row):
            if char == "#":
                x = col_idx * WALL_SIZE_X
                y = -row_idx * WALL_SIZE_Y
                z = WALL_HEIGHT
                pose = f"{x} {y} {z} 0 0 0"

                link = f"""    <!-- Wall {wall_id} -->
    <link name="wall_{wall_id}">
      <pose>{pose}</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{WALL_SIZE_X} {WALL_SIZE_Y} {WALL_SIZE_Z}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{WALL_SIZE_X} {WALL_SIZE_Y} {WALL_SIZE_Z}</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
    </link>
"""
                links.append(link)
                wall_id += 1

    with open(output_file, "w") as f:
        f.write(sdf_header)
        f.writelines(links)
        f.write(sdf_footer)

    print(f"✅ SDF generato in {output_file} con {wall_id} muri.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python generate_sdf.py maze.txt [model.sdf]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "model.sdf"

    with open(input_file, "r") as f:
        ascii_map = f.read()

    ascii_to_sdf(ascii_map, output_file)
