import pathlib

class Table(object):
    main_folder = pathlib.Path(__file__).absolute().parent
    rootJointType = "fix"
    urdfFilename = f"{str(main_folder.joinpath('table.urdf'))}"
    srdfFilename = f"{str(main_folder.joinpath('table.srdf'))}"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, position=None, rpy=None, desk_size=None, leg_display=False) -> None:
        """
        will generate .urdf and .srdf file for environmental object table
        This object can be passed to hpp function loadEnvironmentObject() as argument.

        :param position: position of the table in [x, y, z]
        :param rpy: rotation of the table in [r, p, y]
        :param desk_size: size of the desk in [x, y]
        :param leg_display: True if legs should be displayed, else table will appear as a box
        """

        if desk_size is None:
            desk_size = [1.5, 1.1]
        assert len(desk_size) == 2

        if position is None:
            position = [0, 0, 0]
        assert len(position) == 3

        if rpy is None:
            rpy = [0, 0, 0]
        assert len(position) == 3

        with open(self.urdfFilename, 'w') as f:
            f.write(self.urdf(size=desk_size, pos=position, rot=rpy))
        with open(self.srdfFilename, 'w') as f:
            f.write(self.srdf(size=desk_size))

    @staticmethod
    def urdf(pos, rot, size, material: str = 'brown', color_rgba: str = '0.43 0.34 0.24 0.9'):
        """
        this function generates text for .urdf file with given parameters to create table object

        :param pos: position of the table in [x, y, z]
        :param rot: rotation of the table in [r, p, y]
        :param size: size of the desk in [x, y]
        :param material: optional material name
        :param color_rgba:  optional color of table
        :return: text of .urdf file with the description of table object
        """

        return f"""
                <robot name="table">
                    <link name="base_link">
                        <inertial>
                            <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                            <mass value="1"/>
                            <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
                        </inertial>
                    </link>
                    <link name="desk">
                        <inertial>
                            <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                            <mass value="1"/>
                            <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
                        </inertial>
                        <visual>
                            <origin xyz="{0} {0} {-0.375}" rpy="0 0 0" />
                            <geometry>
                                <box size="{size[0]} {size[1]} {0.75}"/>
                            </geometry>
                            <material name="{material}">
                                <color rgba="{color_rgba}"/>
                            </material>
                        </visual>
                        <collision>
                            <origin xyz="{0} {0} {-0.375}" rpy="0 0 0" />
                            <geometry>
                                <box size="{size[0]} {size[1]} {0.75}"/>
                            </geometry>
                        </collision>
                    </link>

                    <joint name="main_link" type="fixed">
                        <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
                        <parent link="base_link"/>
                        <child link="desk"/>
                    </joint>
                </robot>
                """

    @staticmethod
    def srdf(size):
        """
        this function generates text for .srdf file with given parameters to create contact surfaces

        :param size: size of the desk in [x, y]
        :return: text of .srdf file with contact surfaces for the table object
        """
        return f"""
            <?xml version="1.0"?>
            <robot name="table">
                <contact name="table_surface">
                    <link name="desk"/>
                        <point>
                        {size[0] / 2}   {size[1] / 2}  0        {size[0] / 2}    {-size[1] / 2} 0
                        {-size[0] / 2}  {-size[1] / 2} 0        {-size[0] / 2}   {size[1] / 2}  0
                        </point>
                        <shape>4 3 2 1 0</shape>
                </contact>
            </robot>
            """

    def contact_surfaces(self, prefix):
        """
        This function returns the name of the contact surface of the object
        """
        return [prefix + "table_surface", ]
