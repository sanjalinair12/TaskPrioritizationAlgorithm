<?xml version="1.0"?>
<sdf version="1.6">
  <world name="ahtovik_motion_world">
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so"/>
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <physics name="default_physics" default="0" type="ode"><max_step_size>0.01</max_step_size><real_time_update_rate>100</real_time_update_rate></physics>
    <gui fullscreen="0"><camera name="user_camera"><pose>50 -35 85 1.05 0 0.75</pose><view_controller>orbit</view_controller></camera></gui>
    
    <model name="scan_site"><static>true</static><pose>20 80 0.03 0 0 0</pose>
      <link name="link"><visual name="visual"><geometry><cylinder><radius>8</radius><length>0.06</length></cylinder></geometry><material><ambient>1 0 1 0.35</ambient><diffuse>1 0 1 0.35</diffuse></material></visual></link>
    </model>
    <model name="supply_site"><static>true</static><pose>80 80 0.03 0 0 0</pose>
      <link name="link"><visual name="visual"><geometry><cylinder><radius>8</radius><length>0.06</length></cylinder></geometry><material><ambient>1 0.55 0 0.35</ambient><diffuse>1 0.55 0 0.35</diffuse></material></visual></link>
    </model>
    <model name="lifting_site"><static>true</static><pose>50 20 0.03 0 0 0</pose>
      <link name="link"><visual name="visual"><geometry><cylinder><radius>7</radius><length>0.06</length></cylinder></geometry><material><ambient>0 1 1 0.35</ambient><diffuse>0 1 1 0.35</diffuse></material></visual></link>
    </model>
    
    <model name="D1">
      <static>false</static>
      <pose>5 88 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D2">
      <static>false</static>
      <pose>9 82 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D3">
      <static>false</static>
      <pose>13 76 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D4">
      <static>false</static>
      <pose>17 88 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D5">
      <static>false</static>
      <pose>21 82 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D6">
      <static>false</static>
      <pose>25 76 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D7">
      <static>false</static>
      <pose>29 88 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D8">
      <static>false</static>
      <pose>33 82 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D9">
      <static>false</static>
      <pose>37 76 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="D10">
      <static>false</static>
      <pose>41 88 1.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.8</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.8</radius></sphere></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G1">
      <static>false</static>
      <pose>5 45 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G2">
      <static>false</static>
      <pose>10 50 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G3">
      <static>false</static>
      <pose>15 55 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G4">
      <static>false</static>
      <pose>20 60 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G5">
      <static>false</static>
      <pose>25 45 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G6">
      <static>false</static>
      <pose>30 50 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G7">
      <static>false</static>
      <pose>35 55 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G8">
      <static>false</static>
      <pose>40 60 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G9">
      <static>false</static>
      <pose>45 45 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="G10">
      <static>false</static>
      <pose>50 50 0.4 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>1.4 1.0 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>1.4 1.0 0.5</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C1">
      <static>false</static>
      <pose>8 18 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C2">
      <static>false</static>
      <pose>12 24 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C3">
      <static>false</static>
      <pose>16 30 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C4">
      <static>false</static>
      <pose>20 18 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C5">
      <static>false</static>
      <pose>24 24 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C6">
      <static>false</static>
      <pose>28 30 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C7">
      <static>false</static>
      <pose>32 18 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C8">
      <static>false</static>
      <pose>36 24 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C9">
      <static>false</static>
      <pose>40 30 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <model name="C10">
      <static>false</static>
      <pose>44 18 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.9 0.7 0.25</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.9 0.7 0.25</size></box></geometry><material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
  </world>
</sdf>