from typing import List
import pathlib

class Cuboid(object):
    main_folder = pathlib.Path(__file__).absolute().parent
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""
    _all_handles = ["handleZpx", "handleZmx", "handleYmx", "handleYpx", "handleZpxSm", "handleZmxSm", "handleZpxSp",
                    "handleZmxSp", "handleZpy", "handleZmy", "handleXmy", "handleXpy", "handleZpySm", "handleZmySm",
                    "handleZpySp", "handleZmySp", "handleYpz", "handleYmz", "handleXmz", "handleXpz", "handleYpzSm",
                    "handleYmzSm", "handleYpzSp", "handleYmzSp"]

    def __init__(self, lengths) -> None:
        """
        Create urdf/srdf files for the box of given size.
        This object can be passed to hpp function loadEnvironmentObject() or loadObjectModel() as argument.

        """
        self.urdfFilename = str(self.main_folder.joinpath(f"cuboid.urdf"))
        self.srdfFilename = str(self.main_folder.joinpath(f"cuboid.srdf"))
        self.max_handles_depth = 0.07

        self.lengths = [lengths] * 3 if isinstance(lengths, float) else lengths

        assert len(self.lengths) == 3
        with open(self.urdfFilename, 'w') as f:
            f.write(self.urdf(lengths=self.lengths))
        with open(self.srdfFilename, 'w') as f:
            f.write(self.srdf(lengths=self.lengths, max_handles_depth=self.max_handles_depth))

    def initial_configuration(self) -> List[float]:
        """
        generates initial configuration of cuboid in [x, y, z, i, j, k, w]
        """
        return [0.0, 0, self.lengths[2] / 2 + 0.001, ] + [0, 0, 0, 1]

    def handles(cls, prefix: str = ""):
        """
        This function returns list of all handles prepended with the prefix.
        Handle description following:
                    HandleAbc(Sd)
                        A - X/Y/Z from which coordinate will the gripper come from
                        b - m/p if the approach is from minus (m) or plus (p)
                        c - x/y/z is coordinate in which the width of the grip is represented
                        S - optional S means that the handle is a side handle
                        d - m/p argument tells if the side handle is on minus or plus side of cuboid on the third axis
        :param prefix: prefix for handle name
        :return: list of handles [prefix + handleAbc, prefix + handleAbc, ...]
        """
        return [prefix + h for h in cls._all_handles]

    def contact_surfaces(self, prefix):
        """
        This function returns the name of the contact surface of the object
        """
        return [prefix + "box_surface", ]

    @staticmethod
    def urdf(lengths: List[float], material: str = 'red', color_rgba: str = '1 0.2 0.2 0.8') -> str:
        """
        this function generatestext for .urdf file with given parameters to create cuboid object

        :param lengths: sizes of the cuboid object [width in x, width in y, width in z]
        :param material: optional material name
        :param color_rgba: optional color of cuboid
        :return: text of .urdf file with the description of cuboid object
        """
        return f"""
            <robot name="cuboid">
                <link name="base_link">
                    <inertial>
                        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                        <mass value="1"/>
                        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
                    </inertial>
                    <visual>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{lengths[0]} {lengths[1]} {lengths[2]}"/>
                        </geometry>
                        <material name="{material}">
                        <color rgba="{color_rgba}"/>
                    </material>
                    </visual>
                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{lengths[0]} {lengths[1]} {lengths[2]}"/>
                        </geometry>
                    </collision>
                </link>
             </robot>
        """

    @staticmethod
    def srdf(lengths: List[float], max_handles_depth: float) -> str:
        """
        this function generates text for .srdf file with given parameters to create handles and contact surfaces
        :param lengths: [width in x, width in y, width in z]
        :param max_handles_depth: the maximum distance of handles from the object boundary
        :return: text of .srdf file with handles and contact surfaces for the cuboid object
        """

        # is the distance of the handle from the center of the cuboid
        dist = [0 if l / 2 < max_handles_depth else l / 2 - max_handles_depth for l in lengths]
        a, b, c = [l / 2 for l in lengths]

        return f"""
                <?xml version="1.0"?>
                <robot name="cuboid">
                 <handle name="handleZpx" clearance="{lengths[0]}">
                    <position> 0 0 {dist[2]}   0.7071068, 0, 0.7071068, 0</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmx" clearance="{lengths[0]}">
                    <position> 0 0 {-dist[2]}  0.7071067811865476 0.0 -0.7071067811865475 0.0  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYpx" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} 0.0   0.5 0.5 -0.5  -0.5  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYmx" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} 0.0   0.5 -0.5 -0.5 0.5  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZpxSm" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} {dist[2]}   0.65328148 0.27059805 0.65328148 0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZpxSp" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} {dist[2]}   0.65328148  -0.27059805  0.65328148 -0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmxSm" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} {-dist[2]}    0.65328148  -0.27059805 -0.65328148  0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmxSp" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} {-dist[2]}  0.65328148  0.27059805 -0.65328148 -0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                <handle name="handleZpy" clearance="{lengths[1]}">
                    <position> 0 0 {dist[2]}   0.5 -0.5  0.5  0.5 </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmy" clearance="{lengths[1]}">
                    <position> 0 0 {-dist[2]}   0.5 0.5 -0.5 0.5  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleXmy" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 0.0   0.7071067811865476 -0.7071067811865475 0.0 0.0  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleXpy" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 0.0   0 0 -0.7071067811865475 0.7071067811865476 </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZpySm" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 {dist[2]}    0.65328148 -0.65328148  0.27059805  0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZpySp" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 {dist[2]}   0.27059805  -0.27059805  0.65328148  0.65328148</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmySm" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 {-dist[2]}   0.65328148  0.65328148 -0.27059805  0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleZmySp" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 {-dist[2]}   0.27059805  0.27059805 -0.65328148  0.65328148</position>
                    <link name="base_link"/>
                  </handle>
                <handle name="handleYpz" clearance="{lengths[2]}">
                    <position> 0 {dist[1]} 0.0   0.7071067811865476 0.0 0.0 -0.7071067811865475 </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYmz" clearance="{lengths[2]}">
                    <position> 0 {-dist[1]} 0.0   0 0.7071067811865476 0.7071067811865475 0  </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleXmz" clearance="{lengths[2]}">
                    <position> {-dist[0]} 0 0.0   0, 1, 0, 0 </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleXpz" clearance="{lengths[2]}">
                    <position> {dist[0]} 0 0.0   0 0 1 0 </position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYpzSm" clearance="{lengths[2]}">
                    <position> {-dist[0]} {dist[1]} 0  0  0.9238795 -0.3826834 0</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYpzSp" clearance="{lengths[2]}">
                    <position> {dist[0]} {dist[1]} 0.0    0 0.3826834 -0.9238795 0</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYmzSm" clearance="{lengths[2]}">
                    <position> {-dist[0]} {-dist[1]} 0.0   0  0.9238795 0.3826834 0</position>
                    <link name="base_link"/>
                  </handle>
                  <handle name="handleYmzSp" clearance="{lengths[2]}">
                    <position> {dist[0]} {-dist[1]} 0.0    0 0.3826834 0.9238795 0</position>
                    <link name="base_link"/>
                  </handle>
                  <contact name="box_surface">
                    <link name="base_link"/>
                     <point>
                         {-a} {-b} {-c}  {-a} {b} {-c}  {-a} {-b} {c}  {-a} {b} {c}
                         {a} {-b} {-c}   {a} {b} {-c}   {a} {-b} {c}   {a} {b} {c}
                     </point>
                     <shape>4 1 5 4 0     4 2 6 7 3     4 2 3 1 0     4 4 5 7 6     4 4 6 2 0     4 1 3 7 5 </shape>
                  </contact>
                </robot>
               """
