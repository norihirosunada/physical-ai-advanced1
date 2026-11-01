"""Adapt upstream SO-101 follower URDF for Humble/Fortress ros2_control."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
JOINTS=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']

def description():
    robot=ET.parse(ROOT/'vendor/so101_new_calib.urdf').getroot()
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename','file://'+str(ROOT/'vendor'/mesh.get('filename')))
    # Replace detailed distal contact meshes by geometric contact proxies.
    # Visuals, joint geometry and inertias remain from the official model.
    def collision_box(link_name, name, xyz, size):
        link=robot.find(f"link[@name='{link_name}']")
        c=ET.SubElement(link,'collision',name=name)
        ET.SubElement(c,'origin',xyz=xyz,rpy='0 0 0')
        ET.SubElement(ET.SubElement(c,'geometry'),'box',size=size)
    for link_name in ['gripper_link','moving_jaw_so101_v1_link']:
        link=robot.find(f"link[@name='{link_name}']")
        for c in list(link.findall('collision')):link.remove(c)
    collision_box('gripper_link','palm_proxy','0.0077 0 -0.0234','0.0454 0.0396 0.0248')
    collision_box('gripper_link','fixed_finger_proxy','-0.0144 -0.000218 -0.0941274','0.013 0.014 0.024')
    collision_box('moving_jaw_so101_v1_link','moving_finger_proxy','-0.0051 -0.0707274 0.0190182','0.014 0.024 0.014')
    ET.SubElement(robot,'link',name='world')
    mount=ET.SubElement(robot,'joint',name='world_mount',type='fixed')
    ET.SubElement(mount,'parent',link='world');ET.SubElement(mount,'child',link='base_link')
    ET.SubElement(mount,'origin',xyz='0 0 1.015',rpy='0 0 0')
    control=ET.SubElement(robot,'ros2_control',name='SO101GazeboSystem',type='system')
    hardware=ET.SubElement(control,'hardware')
    ET.SubElement(hardware,'plugin').text='ign_ros2_control/IgnitionSystem'
    for name in JOINTS:
        joint=ET.SubElement(control,'joint',name=name)
        cmd=ET.SubElement(joint,'command_interface',name='position')
        limit=robot.find(f"./joint[@name='{name}']/limit")
        for key in ['min','max']:
            ET.SubElement(cmd,'param',name=key).text=limit.get('lower' if key=='min' else 'upper')
        for interface in ['position','velocity','effort']:
            ET.SubElement(joint,'state_interface',name=interface)
        physical=robot.find(f"./joint[@name='{name}']")
        if physical.find('dynamics') is None:ET.SubElement(physical,'dynamics',damping='0.05',friction='0.01')
    gazebo=ET.SubElement(robot,'gazebo')
    plugin=ET.SubElement(gazebo,'plugin',filename='libign_ros2_control-system.so',name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text=str(ROOT/'config/controllers.yaml')
    return ET.tostring(robot,encoding='unicode')

if __name__=='__main__':
    out=ROOT/'generated.urdf';out.write_text(description());print(out)
