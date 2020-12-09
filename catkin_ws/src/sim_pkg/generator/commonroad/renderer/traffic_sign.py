import os, shutil, pkg_resources, math
from os import path

ALL_SIZES = {
    "stvo-274.1": {"w": 0.15, "h": 0.15}, # zone 30 (begin)
    "stvo-274.2": {"w": 0.15, "h": 0.15}, # zone 30 (end)
    "stvo-350-10": {"w": 0.15, "h": 0.15}, # zebra crossing
    "stvo-625-11": {"w": 0.3, "h": 0.1}, # large curve sign (left)
    "stvo-625-21": {"w": 0.3, "h": 0.1} # large curve sign (right)
}
DEFAULT_SIZE = {"w": 0.1, "h": 0.1}
HEIGHT = 0.15

def draw(sign, target_dir):
    os.makedirs(path.join(target_dir, "materials", "textures"), exist_ok=True)
    os.makedirs(path.join(target_dir, "materials", "scripts"), exist_ok=True)

    texture_file = "sign-{0}.png".format(sign.type)
    material_file = "sign-{0}.material".format(sign.type)

    texture_stream = pkg_resources.resource_stream("commonroad.renderer.signs",
        "{0}.png".format(sign.type))
    with open(path.join(target_dir, "materials", "textures", texture_file), "wb") as texture_target:
        shutil.copyfileobj(texture_stream, texture_target)

    with open(path.join(target_dir, "materials", "scripts", material_file), "w") as material_target:
        material_target.write(material("Sign/{0}".format(sign.type), texture_file))

    size = ALL_SIZES.get(sign.type, DEFAULT_SIZE)

    return model(sign.centerPoint.x, sign.centerPoint.y, HEIGHT + size["h"]/2, sign.orientation + math.pi/2,
        "Sign/{0}".format(sign.id), "Sign/{0}".format(sign.type), size["w"], size["h"])

def material(name, file):
    return """
    material {name}
    {{
        technique
        {{
            pass
            {{
                scene_blend alpha_blend
                depth_write off

                texture_unit
                {{
                    texture {file}
                    filtering trilinear
                }}
            }}
        }}
    }}
    """.format(name=name, file=file)

def model(x, y, z, orientation, name, material, width, height):
    return """
    <model name='{name}'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{width} {height}</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://materials/scripts</uri>
              <uri>file://materials/textures</uri>
              <name>{material}</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>{x} {y} {z} {angle} 0 {orientation}</pose>
    </model>
    """.format(x=x, y=y, z=z, orientation=orientation, name=name,
        material=material, width=width, height=height, angle=math.pi/2)
