def generate_world_sdf(
    model_name: str = 'apple_tree_1',
    x: float = 0.0,
    y: float = -1.6,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 1.5708,
) -> str:
    """Return a Gazebo world SDF string.

    Includes the shared apple_tree_ground model at the world origin and the
    chosen model at the specified (x, y, z, roll, pitch, yaw) pose.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
    <world name="apple_tree_world">

        <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
        <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
        <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>

        <physics type="ode">
            <max_step_size>0.01</max_step_size>
            <real_time_factor>1</real_time_factor>
            <max_contacts>10</max_contacts>
            <real_time_update_rate>1000</real_time_update_rate>
        </physics>

        <gravity>0 0 -9.8</gravity>

        <scene>
            <ambient>0.5 0.5 0.5 1</ambient>
            <background>0.3 0.7 0.9 1</background>
            <grid>false</grid>
            <sky>
                <clouds>
                    <speed>12</speed>
                </clouds>
            </sky>
            <shadows>false</shadows>
        </scene>

        <light name='sun' type='directional'>
            <pose>0 0 10 0 0 0</pose>
            <cast_shadows>false</cast_shadows>
            <intensity>1</intensity>
            <direction>-0.5 0.1 -0.9</direction>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <linear>0.01</linear>
                <constant>0.90000000000000002</constant>
                <quadratic>0.001</quadratic>
            </attenuation>
            <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
            </spot>
        </light>

        <include>
            <uri>model://apple_tree_ground</uri>
            <name>apple_tree_ground</name>
            <pose>0 0 0 0 0 -1.57</pose>
        </include>

        <include>
            <uri>model://{model_name}</uri>
            <name>{model_name}</name>
            <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        </include>

    </world>
</sdf>"""
