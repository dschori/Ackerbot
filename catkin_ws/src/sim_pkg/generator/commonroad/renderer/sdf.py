from commonroad.renderer import groundplane, obstacle, traffic_sign, ego_vehicle
from commonroad import schema
from os import path

def generate_sdf(xml_content, target_dir):
    doc = schema.CreateFromDocument(xml_content)

    content = groundplane.draw(doc, target_dir)
    #content += ego_vehicle.draw(target_dir, doc.lanelet)
    for obst in doc.obstacle:
        if obst.type != "blockedArea":
            content += obstacle.draw(obst)
    for sign in doc.trafficSign:
        content += traffic_sign.draw(sign, target_dir)

    with open(path.join(target_dir, "world.sdf"), "w") as file:
        file.write("<sdf version='1.6'><world name='default'>")
        file.write(sun_light())
        file.write(content)
        file.write("</world></sdf>")

def sun_light():
    return """
    <light name='sun_light' type='directional'>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.1 0.1 -0.9</direction>
      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
    </light>
    """
