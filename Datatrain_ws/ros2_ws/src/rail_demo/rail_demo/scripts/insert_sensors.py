from lxml import etree

SENSOR_XML = """
<link name="lidar_link">
  <pose relative_to="base_link">0 0 0.6 0 0 0</pose>
  <sensor name="lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <topic>lidar</topic>
    <gpu_lidar>
      <horizontal>
        <samples>720</samples>
        <min_angle>-1.57</min_angle>
        <max_angle>1.57</max_angle>
      </horizontal>
      <range>
        <min>0.1</min>
        <max>30.0</max>
      </range>
    </gpu_lidar>
  </sensor>
</link>

<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent>
  <child>lidar_link</child>
</joint>
"""

parser = etree.XMLParser(remove_comments=False)
tree = etree.parse("model.sdf", parser)
root = tree.getroot()

# Find model
model = root.find("model")

# Find the comment marker
for node in model.iter():
    if isinstance(node, etree._Comment):
        if "AUTO-GENERATED SENSOR LINKS" in node.text:
            parent = node.getparent()
            index = parent.index(node)

            fragment = etree.fromstring(f"<tmp>{SENSOR_XML}</tmp>")
            for elem in fragment:
                parent.insert(index + 1, elem)
            break

tree.write(
    "model_with_sensors.sdf",
    pretty_print=True,
    xml_declaration=True,
    encoding="UTF-8"
)