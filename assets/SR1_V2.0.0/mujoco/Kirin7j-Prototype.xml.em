<mujoco model='Xtellar.Robotics/Kirin7j-Prototype'>
    <compiler meshdir='../meshes' />

    <default>
        <default class='viscol'>
            <geom
                type='mesh' rgba='@_main_rgba[0] @_main_rgba[1] @_main_rgba[2] @_main_rgba[3]'
                contype='0' conaffinity='0' density='0'
            />
        </default>
        <joint damping='0.1' frictionloss='0.2' armature='0.01' actuatorgravcomp='true' />
    </default>

    <asset>
        <mesh name='m0_base' file='m0_base.stl' scale='0.001 0.001 0.001' />
        <mesh name='m1_shp'  file='m1_shp.stl'  scale='0.001 0.001 0.001' />
        <mesh name='m2_shr'  file='m2_shr.stl'  scale='0.001 0.001 0.001' />
        <mesh name='m3_shy'  file='m3_shy.stl'  scale='0.001 0.001 0.001' />
        <mesh name='m4_elp'  file='m4_elp.stl'  scale='0.001 0.001 0.001' />
        <mesh name='m5_ely'  file='m5_ely.stl'  scale='0.001 0.001 0.001' />
        <mesh name='m6_wrp'  file='m6_wrp.stl'  scale='0.001 0.001 0.001' />
    </asset>

    <worldbody>
        @(taghead_body(link_name='F_arm_base', indent=2))

            @(taghead_body(link_name='L0_base', extra_attrs=['gravcomp="1"']))

                @(tag_inertial(link_name='L0_base', indent=4))
                <geom class='viscol' mesh='m0_base' />

                @(taghead_body(link_name='L1_shp', extra_attrs=['gravcomp="1"']))
                    @(tag_joint(joint_name='J1_shp', indent=5))

                    @(tag_inertial(link_name='L1_shp'))
                    <geom class='viscol' mesh='m1_shp' />

                    @(taghead_body(link_name='L2_shr', extra_attrs=['gravcomp="1"']))
                        @(tag_joint(joint_name='J2_shr'))

                        @(tag_inertial(link_name='L2_shr'))
                        <geom class='viscol' mesh='m2_shr' />

                        @(taghead_body(link_name='L3_shy', extra_attrs=['gravcomp="1"']))
                            @(tag_joint(joint_name='J3_shy'))

                            @(tag_inertial(link_name='L3_shy'))
                            <geom class='viscol' mesh='m3_shy' />

                            @(taghead_body(link_name='L4_elp', extra_attrs=['gravcomp="1"']))
                                @(tag_joint(joint_name='J4_elp'))

                                @(tag_inertial(link_name='L4_elp'))
                                <geom class='viscol' mesh='m4_elp' />

                                @(taghead_body(link_name='F_elbow'))
                                </body>

                                @(taghead_body(link_name='L5_ely', indent=8, extra_attrs=['gravcomp="1"']))
                                    @(tag_joint(joint_name='J5_ely'))

                                    @(tag_inertial(link_name='L5_ely'))
                                    <geom class='viscol' mesh='m5_ely' />

                                    @(taghead_body(link_name='L6_wrp', extra_attrs=['gravcomp="1"']))
                                        @(tag_joint(joint_name='J6_wrp'))

                                        @(tag_inertial(link_name='L6_wrp'))
                                        <geom class='viscol' mesh='m6_wrp' />

                                        @(taghead_body(link_name='F_ee_base', extra_attrs=['gravcomp="1"']))
                                            @(tag_joint(joint_name='J7_wry'))

                                            <inertial pos="0 0 0" mass="1e-14" diaginertia="1e-14 1e-14 1e-14" />

                                            <site pos='0.05 0 0' size='0.05 0.001 0.001' type='box' rgba='1 0 0 1' group='3' />
                                            <site pos='0 0.05 0' size='0.001 0.05 0.001' type='box' rgba='0 1 0 1' group='3' />
                                            <site pos='0 0 0.05' size='0.001 0.001 0.05' type='box' rgba='0 0 1 1' group='3' />
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <contact>
        <exclude body1='L0_base' body2='L1_shp' />
        <exclude body1='L1_shp'  body2='L2_shr' />
        <exclude body1='L2_shr'  body2='L3_shy' />
        <exclude body1='L3_shy'  body2='L4_elp' />
        <exclude body1='L4_elp'  body2='L5_ely' />
        <exclude body1='L5_ely'  body2='L6_wrp' />
        <exclude body1='L6_wrp'  body2='F_ee_base' />

        <exclude body1='L0_base' body2='L2_shr' />
        <exclude body1='L1_shp'  body2='L3_shy' />
        <exclude body1='L2_shr'  body2='L4_elp' />
        <exclude body1='L3_shy'  body2='L5_ely' />
        <exclude body1='L4_elp'  body2='L6_wrp' />
        <exclude body1='L5_ely'  body2='F_ee_base' />
    </contact>
</mujoco>
