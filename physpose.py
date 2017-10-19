import bpy
import math
from mathutils import Vector, Matrix
import json

#import physobject

def create_rect(sx, sy, sz, name, offset = None):
    """
        Creates 3D rectangle with size x (sx), y, and z.
         Y is scaled to 0.0 ... 1.0 range
    """
    bpy.ops.object.mode_set(mode='OBJECT')
    sx *= 0.5
    #sy *= 0.5
    sz *= 0.5
    verts = [[sx, sy, -sz],
             [sx, 0, -sz],
            [-sx, 0, -sz],
            [-sx, sy, -sz],
             [sx, sy, sz],
             [sx, 0, sz],
            [-sx, 0, sz],
            [-sx, sy, sz]]
    if offset:
        for vert in verts:
            vert[0] += offset[0]
            vert[1] += offset[1]
            vert[2] += offset[2]

    faces = [(0, 1, 2, 3),
             (4, 7, 6, 5),
             (0, 4, 5, 1),
             (1, 5, 6, 2),
             (2, 6, 7, 3),
             (4, 0, 3, 7)]

    mesh_data = bpy.data.meshes.new(name + "-MESH")
    mesh_data.from_pydata(verts, [], faces)
    mesh_data.update() # (calc_edges=True) not needed here

    cube_object = bpy.data.objects.new(name, mesh_data)

    scene = bpy.context.scene  
    scene.objects.link(cube_object)  
    phys_object_list.append(cube_object)
    phys_object_dict[name] = cube_object
    return cube_object

def get_phys_name(obname, bonename):
    return 'PHYS-%s-%s' % (obname, bonename)


def create_copy_rotation_constraint(pose_bone):
    const = pose_bone.constraints.new('COPY_ROTATION')
    const.name = 'PHYSPOSE-COPY-ROTATION'
    const.target = phys_object_dict[get_phys_name(ob.name, pose_bone.name)]


def create_cube_at_bone(ob, bone_name, size_x = 0.1, size_z = 0.1, length = None, offset = None):
    bpy.ops.object.mode_set(mode='POSE')
    if bone_name not in ob.pose.bones:
        return False
    bone = ob.pose.bones[bone_name]

    if length is not None:
        pass  # Use the length as provided
    else:
        if isinstance(length, basestring):
            # The length parameter here is a string representing the bone to use for length calculation
            bone_for_dist = ob.pose.bones[length]
            bone_dist = Vector(bone_for_dist.tail) - Vector(bone_for_dist.head)
            print("DEBUG", bone_dist, bone_dist.length)
            length = bone_dist.length
        else:
            #  Use the bone that was passed as length
            bone_dist = Vector(bone.tail) - Vector(bone.head)
            length = bone_dist.length

    bpy.ops.object.mode_set(mode='OBJECT')
    cube = create_rect(size_x, length, size_z, get_phys_name(ob.name, bone_name), offset)
    cube.layers = ob.layers

    mat = Matrix()
    mat[0][0:4] = (bone.x_axis[0], bone.y_axis[0], bone.z_axis[0], bone.head[0])
    mat[1][0:4] = (bone.x_axis[1], bone.y_axis[1], bone.z_axis[1], bone.head[1])
    mat[2][0:4] = (bone.x_axis[2], bone.y_axis[2], bone.z_axis[2], bone.head[2])
    cube.matrix_world = mat
    create_copy_rotation_constraint(bone)
    pose_bones[bone.name] = bone
    return True


def create_cube_at_bone_lr(ob, bone_name, size_x = 0.1, size_z = 0.1, length = None, offset = None):
    create_cube_at_bone(ob, bone_name+".L", size_x, size_z, length, offset)
    create_cube_at_bone(ob, bone_name+".R", size_x, size_z, length, offset)


def setup_constraint(con, bone, p):
    """
        Parameters:
            con: constraint
            bone: pose bone
            p: parameters for constraint (limit x, limit y, limit z, disable_collisions option)
    """
    #cr = bone.rotation_quaternion.to_euler()
    cr = [0,0,0]
    #print("%s:" % bone.name, bone.rotation_quaternion.to_euler())
    con.rigid_body_constraint.use_limit_lin_x = True
    con.rigid_body_constraint.limit_lin_x_lower = 0
    con.rigid_body_constraint.limit_lin_x_upper = 0
    con.rigid_body_constraint.use_limit_lin_y = True
    con.rigid_body_constraint.limit_lin_y_lower = 0
    con.rigid_body_constraint.limit_lin_y_upper = 0
    con.rigid_body_constraint.use_limit_lin_z = True
    con.rigid_body_constraint.limit_lin_z_lower = 0
    con.rigid_body_constraint.limit_lin_z_upper = 0

    if not p:
        p = [-180,180,-180,180,-180,180,True]
        print("warning: no parameters given for bone %s. Default to %s" % (bone.name, p))

    con.rigid_body_constraint.use_limit_ang_x = True
    con.rigid_body_constraint.limit_ang_x_lower = math.radians(p[0] - math.degrees(cr[0]))
    con.rigid_body_constraint.limit_ang_x_upper = math.radians(p[1] - math.degrees(cr[0]))
    con.rigid_body_constraint.use_limit_ang_y = True
    con.rigid_body_constraint.limit_ang_y_lower = math.radians(p[2] + math.degrees(cr[1]))
    con.rigid_body_constraint.limit_ang_y_upper = math.radians(p[3] + math.degrees(cr[1]))
    con.rigid_body_constraint.use_limit_ang_z = True
    con.rigid_body_constraint.limit_ang_z_lower = math.radians(p[4] + math.degrees(cr[2]))
    con.rigid_body_constraint.limit_ang_z_upper = math.radians(p[5] + math.degrees(cr[2]))
    con.rigid_body_constraint.disable_collisions = p[6]


def create_constraint(bone_name, bone_name2, parameters):
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects[ob.name].select = True
    bpy.context.scene.objects.active = bpy.data.objects[ob.name]
    bpy.ops.object.mode_set(mode='POSE')
    bone = ob.pose.bones[bone_name2]
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.empty_add(type='ARROWS', location=bone.head)
    mat = Matrix()
    mat[0][0:4] = (bone.x_axis[0], bone.y_axis[0], bone.z_axis[0], bone.head[0])
    mat[1][0:4] = (bone.x_axis[1], bone.y_axis[1], bone.z_axis[1], bone.head[1])
    mat[2][0:4] = (bone.x_axis[2], bone.y_axis[2], bone.z_axis[2], bone.head[2])
    con = bpy.context.scene.objects.active
    con.matrix_world = mat
    con.layers = ob.layers
    bpy.ops.rigidbody.constraint_add()
    con.rigid_body_constraint.type = 'GENERIC'
    con.rigid_body_constraint.object1 = bpy.data.objects[get_phys_name(ob.name, bone_name)]
    con.rigid_body_constraint.object2 = bpy.data.objects[get_phys_name(ob.name, bone_name2)]
    joint_name = 'JOINT-%s-%s-%s' % (ob.name, bone_name, bone_name2)
    con.name = joint_name
    setup_constraint(con, bone, parameters)
    joints[joint_name] = con


def get_bones_by_chain(bone_name, attach_object_name, chain_length=90, bone_size=0.1, bone_dof=(2, 2, 2), mass=0.01):
    bpy.ops.object.mode_set(mode='OBJECT')
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    bpy.ops.object.mode_set(mode='POSE')
    if bone_name not in armature.pose.bones:
        return False
    bone_list = []    
    bone = armature.pose.bones[bone_name]
    bone_list.append(bone)
    def build_recurse(bone, bone_list):
        for child in bone.children:
            bone_list.append(child)
            build_recurse(child, bone_list)
    build_recurse(bone, bone_list)
    build_chain(bone_list, attach_object_name, bone_size, bone_dof, mass)
    return bone_list


def build_chain(bone_list, attach_object_name, bone_size, bone_dof, mass):
    bpy.ops.object.mode_set(mode='OBJECT')
    armature = get_armature()

    def create_physobj(bone, ob, size_x, size_z, length=None, offset=None):
        bone_dist = Vector(bone.tail) - Vector(bone.head)
        bpy.ops.object.mode_set(mode='OBJECT')
        physobj_name = get_phys_name(ob.name, bone.name)
        if not length:
            length = bone_dist.length
        cube = create_rect(size_x, length, size_z, physobj_name, offset)
        cube.layers = ob.layers
        mat = Matrix()
        mat[0][0:4] = (bone.x_axis[0], bone.y_axis[0], bone.z_axis[0], bone.head[0])
        mat[1][0:4] = (bone.x_axis[1], bone.y_axis[1], bone.z_axis[1], bone.head[1])
        mat[2][0:4] = (bone.x_axis[2], bone.y_axis[2], bone.z_axis[2], bone.head[2])
        cube.matrix_world = mat
        const = bone.constraints.new('COPY_ROTATION')
        const.name = 'PHYSPOSE-COPY-ROTATION'
        const.target = bpy.data.objects[physobj_name]
        return cube

    physobj_list = []
    #disable some constraints???
    for bone in bone_list:   
        for con in bone.constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.KinematicConstraint]:
                con.mute = True
        physobj_list.append( create_physobj(bone, armature, bone_size, bone_size, None, None) )

    bpy.ops.object.select_all(action='DESELECT')
    for phys_object in physobj_list:
        phys_object.select = True
        bpy.context.scene.objects.active = phys_object
    bpy.ops.rigidbody.objects_add(type='ACTIVE')

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    for phys_object in physobj_list:
        #phys_object.rigid_body.angular_damping = 1.0
        #phys_object.rigid_body.linear_damping = 1.0
        phys_object.rigid_body.use_margin = True
        phys_object.rigid_body.collision_margin = 0.001
        phys_object.rigid_body.mass = mass
        #phys_object.rigid_body.collision_shape = 'MESH'


    def create_constraint(armature, bone1, bone2, parameters):
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.empty_add(type='ARROWS', location=bone1.head)
        mat = Matrix()
        mat[0][0:4] = (bone2.x_axis[0], bone2.y_axis[0], bone2.z_axis[0], bone2.head[0])
        mat[1][0:4] = (bone2.x_axis[1], bone2.y_axis[1], bone2.z_axis[1], bone2.head[1])
        mat[2][0:4] = (bone2.x_axis[2], bone2.y_axis[2], bone2.z_axis[2], bone2.head[2])
        con = bpy.context.scene.objects.active
        con.matrix_world = mat
        con.layers = armature.layers
        bpy.ops.rigidbody.constraint_add()
        con.rigid_body_constraint.type = 'GENERIC'
        con.rigid_body_constraint.object1 = bpy.data.objects[get_phys_name(armature.name, bone1.name)]
        con.rigid_body_constraint.object2 = bpy.data.objects[get_phys_name(armature.name, bone2.name)]
        joint_name = 'JOINT-%s-%s-%s' % (armature.name, bone1.name, bone2.name)
        con.name = joint_name
        setup_constraint(con, bone, parameters)

    #BUILD CONSTRAINTS!!!!!!!!!!
    cdx, cdy, cdz = bone_dof
    constraint_parameters = [-cdx,cdx,-cdy,cdy,-cdz,cdz,True]

    for i in range(len(bone_list)):   
        if i+1 < len(bone_list):
            create_constraint( armature, bone_list[i], bone_list[i+1], constraint_parameters )

    #SELECT the root and the connector object and create that constraint
    if attach_object_name and attach_object_name in bpy.data.objects:
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        physobj_list[0].select = True
        bpy.context.scene.objects.active = physobj_list[0]
        bpy.data.objects[attach_object_name].select = True
        bpy.context.scene.objects.active = bpy.data.objects[attach_object_name]
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.empty_add(type='ARROWS', location=physobj_list[0].location)
        con = bpy.context.scene.objects.active
        con.layers = armature.layers
        bpy.ops.rigidbody.constraint_add()
        con.rigid_body_constraint.type = 'GENERIC'
        con.rigid_body_constraint.object1 = physobj_list[0]
        con.rigid_body_constraint.object2 = bpy.data.objects[attach_object_name]
        joint_name = 'JOINT-%s-%s-%s' % (armature.name, attach_object_name, physobj_list[0].name)
        con.name = joint_name
        setup_constraint(con, bone, constraint_parameters)

    const = bone_list[0].constraints.new('COPY_LOCATION')
    const.name = 'PHYSPOSE-COPY-LOCATION'
    const.target = physobj_list[0]


bones_to_create = {
    "pelvis" : [0.27, 0.18, None, (0.0, -0.16, -0.02)],
    "abdomenLower" : [0.23, 0.17],
    "abdomenUpper" : [0.23, 0.16],
    "chestLower" : [0.28, 0.3, 0.13],
    "chestUpper" : [0.12, 0.20, 0.14],
    "neckLower" : [0.1, 0.1],
    "neckUpper" : [0.1, 0.1],
    "head" : [0.30, 0.30, 0.2],
    "ThighBend.L" : [0.11, 0.11, None, (0, 0.07, 0) ],
    "ThighBend.R" : [0.11, 0.11, None, (0, 0.07, 0) ],
    "ThighTwist.L" : [0.11, 0.11],
    "ThighTwist.R" : [0.11, 0.11],
    "Shin.L" : [0.10, 0.10, 0.36],
    "Shin.R" : [0.10, 0.10, 0.36],
    "Collar.L" : [0.1, 0.1, 0.1, (0,0.035,0)],
    "Collar.R" : [0.1, 0.1, 0.1, (0,0.035,0)],
    "ShldrBend.L" : [0.081, 0.081],
    "ShldrBend.R" : [0.081, 0.081],
    "ShldrTwist.L" : [0.081, 0.081],
    "ShldrTwist.R" : [0.081, 0.081],
    "ForearmBend.L" : [0.051, 0.051],
    "ForearmBend.R" : [0.051, 0.051],
    "ForearmTwist.L" : [0.041, 0.041],
    "ForearmTwist.R" : [0.041, 0.041],
    "Hand.L" : [0.10, 0.085, 0.04, ( 0.030, -0.02, 0)],
    "Hand.R" : [0.10, 0.085, 0.04, (-0.030, -0.02, 0)],
    "Foot.L" : [0.16, 0.22, 0.09, (0, -0.061, 0.055)],
    "Foot.R" : [0.16, 0.22, 0.09, (0, -0.061, 0.055)],
    #HAND BONES ----------------------------------------------------
    "Index1.R" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Mid1.R" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Ring1.R" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Pinky1.R" : [0.011, 0.011, 0.06, (0, 0.012, 0) ],
    "Thumb2.R" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Index1.L" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Mid1.L" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Ring1.L" : [0.015, 0.015, 0.08, (0, 0.012, 0) ],
    "Pinky1.L" : [0.011, 0.011, 0.06, (0, 0.012, 0) ],
    "Thumb2.L" : [0.015, 0.015, 0.08, (0, 0.012, 0) ]
}

constraints = [
    [ 'pelvis', 'abdomenLower', [-20,35,-15,15,-15,15,True] ],
    [ 'abdomenLower', 'abdomenUpper', [-25,40,-20,20,-24,24,True] ],
    [ 'abdomenUpper', 'chestLower', [-25,35,-12,12,-20,20,True] ],
    [ 'chestLower', 'chestUpper', [-15,15,-10,10,-10,10,True] ],
    [ 'chestUpper', 'neckLower', [-15,30,-22,22,-40,40,True] ],
    [ 'neckLower', 'neckUpper', [-17,12,-22,22,-10,10,True] ],
    [ 'neckUpper', 'head', [-27,25,-22,22,-20,20,True] ],
    [ 'chestUpper', 'Collar.L', [-26,17,-30,30,-10,50,True] ],
    [ 'chestUpper', 'Collar.R', [-17,26,-30,30,-50,10,True] ],
    [ 'Collar.L', 'ShldrBend.L', [-85,35,0,0,-110,40,True] ],
    [ 'Collar.R', 'ShldrBend.R', [-35,85,0,0,-40,110,True] ],
    [ 'pelvis', 'ThighBend.L', [-115,35,-20,20,-85,20,True] ],
    [ 'pelvis', 'ThighBend.R', [-115,35,-20,20,-25,85,True] ],
    [ 'ThighBend.L', 'ThighTwist.L', [0,0,-55,55,0,0,True] ],
    [ 'ThighBend.R', 'ThighTwist.R', [0,0,-55,55,0,0,True] ],
    [ 'ShldrBend.L', 'ShldrTwist.L', [0,0,-95,80,0,0,True] ],
    [ 'ShldrBend.R', 'ShldrTwist.R', [0,0,-95,80,0,0,True] ],
    [ 'ForearmBend.L', 'ForearmTwist.L', [0,0,-90,80,0,0,True] ],
    [ 'ForearmBend.R', 'ForearmTwist.R', [0,0,-90,80,0,0,True] ],
    [ 'ThighTwist.R', 'Shin.R', [0,0,-12,12,0,140,True] ],
    [ 'ThighTwist.L', 'Shin.L', [0,0,-12,12,-140,0,True] ],
    [ 'ShldrTwist.R', 'ForearmBend.R', [-135,20,0,0,0,0,True] ],
    [ 'ShldrTwist.L', 'ForearmBend.L', [-135,20,0,0,0,0,True] ],
    [ 'ForearmTwist.L', 'Hand.L', [-40,40,-40,40,-90,90,True] ],
    [ 'ForearmTwist.R', 'Hand.R', [-40,40,-40,40,-90,90,True] ],
    [ 'Shin.L', 'Foot.L', [-75,40,-12,12,-10,10,True] ],
    [ 'Shin.R', 'Foot.R', [-75,40,-12,12,-10,10,True] ],
    #HAND BONES---------------------------------------------------------------
    [ 'Hand.R', 'Index1.R', [-5,5,-2,2,-60,5,True] ],
    [ 'Hand.R', 'Mid1.R', [-5,5,-2,2,-60,5,True] ],
    [ 'Hand.R', 'Ring1.R', [-5,5,-2,2,-60,5,True] ],
    [ 'Hand.R', 'Pinky1.R', [-5,5,-2,2,-60,5,True] ],
    [ 'Hand.R', 'Thumb2.R', [-30,30,-30,30,-30,30,True] ],
    [ 'Hand.L', 'Index1.L', [-5,5,-2,2,-5,60,True] ],
    [ 'Hand.L', 'Mid1.L', [-5,5,-2,2,-5,60,True] ],
    [ 'Hand.L', 'Ring1.L', [-5,5,-2,2,-5,60,True] ],
    [ 'Hand.L', 'Pinky1.L', [-5,5,-2,2,-5,60,True] ],
    [ 'Hand.L', 'Thumb2.L', [-30,30,-30,30,-30,30,True] ]
]


def create_rig(shrinkwrap_object_name = None, damping = 1.0):
    bpy.ops.object.mode_set(mode='OBJECT')

    #disable some constraints???
    for bone in ob.pose.bones:   
        for con in bone.constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.KinematicConstraint]:
                con.mute = True

    for k, v in bones_to_create.items():
        xlen = zlen = total_length = offset = None
        if len(v) == 2:
            xlen, zlen = v
        elif len(v) == 3:
            xlen, zlen, total_length = v
        elif len(v) == 4:
            xlen, zlen, total_length, offset = v
        create_cube_at_bone(ob, k, xlen, zlen, total_length, offset)

    bpy.ops.object.select_all(action='DESELECT')
    for phys_object in phys_object_list:
        phys_object.select = True
        bpy.context.scene.objects.active = phys_object
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    bpy.ops.rigidbody.mass_calculate(material='Rubber', density=1522)

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    for phys_object in phys_object_list:
        phys_object.rigid_body.angular_damping = damping
        phys_object.rigid_body.linear_damping = damping
        phys_object.rigid_body.use_margin = True
        phys_object.rigid_body.collision_margin = 0.001

        #phys_object.rigid_body.collision_shape = 'MESH'

    if shrinkwrap_object_name:
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        for phys_object in phys_object_list:
            phys_object.select = True
            bpy.context.scene.objects.active = phys_object
            ss = phys_object.modifiers.new('subsurf-to-apply', 'SUBSURF')
            ss.levels = 2
            ss.subdivision_type = 'SIMPLE'
            sr = phys_object.modifiers.new('shrinkwrap-to-apply', 'SHRINKWRAP')
            sr.target = bpy.data.objects[shrinkwrap_object_name]
            sm = phys_object.modifiers.new('smooth-to-apply', 'SMOOTH')
            bpy.ops.object.modifier_apply(apply_as='DATA', modifier="subsurf-to-apply")
            bpy.ops.object.modifier_apply(apply_as='DATA', modifier="shrinkwrap-to-apply")
            bpy.ops.object.modifier_apply(apply_as='DATA', modifier="smooth-to-apply")

    for constraint in constraints:
        from_constraint_name, to_constraint_name, parameters = constraint
        create_constraint(from_constraint_name, to_constraint_name, parameters)


    bone = ob.pose.bones["pelvis"]
    const = bone.constraints.new('COPY_LOCATION')
    const.target = bpy.data.objects['PHYS-%s-pelvis' % ob.name]
    const.name = "PHYSPOSE-COPY-LOCATION"
    bone = ob.pose.bones["abdomenLower"]
    const = bone.constraints.new('COPY_LOCATION')
    const.target = bpy.data.objects['PHYS-%s-abdomenLower' % ob.name]
    const.name = "PHYSPOSE-COPY-LOCATION"


def restore_physpose_rig():
    bpy.ops.object.mode_set(mode='OBJECT')
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    
    bpy.ops.object.select_all(action='DESELECT')
    physpose_data = json.loads(armature.phys_pose_data)
    bpy.context.scene.layers = armature.layers
    for name in physpose_data['phys_object_names']:
        bpy.data.objects[name].matrix_world = Matrix(physpose_data['stored_pose'][name])
    return True, "Success" 


def set_damping():
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    physpose_data = json.loads(armature.phys_pose_data)
    damping_setting = armature.phys_pose_damping
    for name in physpose_data['phys_object_names']:
        bpy.data.objects[name].rigid_body.angular_damping = damping_setting
        bpy.data.objects[name].rigid_body.linear_damping = damping_setting
    return True, "Success" 


def delete_rig():
    bpy.ops.object.mode_set(mode='OBJECT')
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    
    bpy.ops.object.select_all(action='DESELECT')
    physpose_data = json.loads(armature.phys_pose_data)
    bpy.context.scene.layers = armature.layers

    armature.pose.bones["HandIk.R"].matrix = armature.pose.bones["Hand.R"].matrix.copy()
    armature.pose.bones["HandIk.L"].matrix = armature.pose.bones["Hand.L"].matrix.copy()
    armature.pose.bones["FootIk.R"].matrix = armature.pose.bones["Foot.R"].matrix.copy()
    armature.pose.bones["FootIk.L"].matrix = armature.pose.bones["Foot.L"].matrix.copy()

    for name in physpose_data['bone_names']:
        for con in armature.pose.bones[name].constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.CopyLocationConstraint] \
                and con.name == 'PHYSPOSE-COPY-LOCATION' or con.name == 'PHYSPOSE-COPY-ROTATION':
                armature.pose.bones[name].constraints.remove(con)
            elif con.mute:
                con.mute = False 


    for name in physpose_data['phys_object_names']:
        bpy.data.objects[name].select = True
    for name in physpose_data['joint_names']:
        bpy.data.objects[name].select = True
    
    bpy.ops.object.delete()
    return True, "Success"


def unmute_constraints():
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    armature.select = True
    bpy.context.scene.objects.active = armature
    bpy.context.scene.layers = armature.layers

    physpose_data = json.loads(armature.phys_pose_data)
    for name in physpose_data['bone_names']:
        for con in armature.pose.bones[name].constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.CopyLocationConstraint]:
                if con.name == 'PHYSPOSE-COPY-LOCATION' or con.name == 'PHYSPOSE-COPY-ROTATION':
                    con.mute = False
    return True, "Success"


def pin_phys_object():
    bpy.ops.object.mode_set(mode='OBJECT')
    for o in bpy.context.selected_objects:
        if hasattr(o, 'rigid_body'):
            matrix = o.matrix_world
            o.rigid_body.kinematic = True
            o.matrix_world = matrix
    return True, "Success"


def unpin_phys_object():
    bpy.ops.object.mode_set(mode='OBJECT')
    for o in bpy.context.selected_objects:
        if hasattr(o, 'rigid_body'):
            o.rigid_body.kinematic = False
    return True, "Success"


def set_rotations():
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    armature.select = True
    bpy.context.scene.objects.active = armature
    bpy.context.scene.layers = armature.layers

    bpy.ops.object.mode_set(mode='POSE')
    physpose_data = json.loads(armature.phys_pose_data)
    matrices = {}
    for name in physpose_data['bone_names']:
        matrices[name] = armature.pose.bones[name].matrix
    for name in physpose_data['bone_names']:
        for con in armature.pose.bones[name].constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.CopyLocationConstraint]:
                if con.name == 'PHYSPOSE-COPY-LOCATION' or con.name == 'PHYSPOSE-COPY-ROTATION':
                    con.mute = True
                    armature.pose.bones[name].matrix = matrices[name]
            #elif unmute_ik and type(con) == bpy.types.KinematicConstraint and con.mute:
            #    con.mute = False
    return True, "Success"


def get_set_limits(bone_name_1, bone_name_2):
    for c in constraints:
        if c[0] == bone_name_1 and c[1] == bone_name_2:
            return c[2]
    return [-180,180,-180,180,-180,180,True]


def set_spine_stiffness():
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    set_stiffness('spine', armature.phys_pose_spine_stiffness)
    return True, "Success" 



def set_stiffness(bone_group, scalar):
    bone_groups = {
        'spine': ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower","neckUpper","head"],
        'shoulders': ["chestUpper","Collar.R","Collar.L"]
    }
    armature = get_armature()
    if not armature:
        return False, "Object is not supported type (Armature or Mesh)"
    if bone_group not in bone_groups: 
        return False, "Invalid bone group name: %s" % bone_group

    physpose_data = json.loads(armature.phys_pose_data)
    scalar = armature.phys_pose_spine_stiffness
    spine_bone_names = bone_groups[bone_group]
    print("Checking the following")
    print(spine_bone_names)
    for name in physpose_data['joint_names']:
        po_names = name.split('-')[2:]
        if not (po_names[0] in spine_bone_names and po_names[1] in spine_bone_names):
            continue
        print(po_names)
        p = get_set_limits(po_names[0], po_names[1])
        con = bpy.data.objects[name]
        con.rigid_body_constraint.use_limit_ang_x = True
        con.rigid_body_constraint.limit_ang_x_lower = math.radians(p[0]) * scalar
        con.rigid_body_constraint.limit_ang_x_upper = math.radians(p[1]) * scalar
        con.rigid_body_constraint.use_limit_ang_y = True
        con.rigid_body_constraint.limit_ang_y_lower = math.radians(p[2]) * scalar
        con.rigid_body_constraint.limit_ang_y_upper = math.radians(p[3]) * scalar
        con.rigid_body_constraint.use_limit_ang_z = True
        con.rigid_body_constraint.limit_ang_z_lower = math.radians(p[4]) * scalar
        con.rigid_body_constraint.limit_ang_z_upper = math.radians(p[5]) * scalar
        con.rigid_body_constraint.disable_collisions = p[6]
        
    return True, "Success" 
    

ob = None
amt = None
phys_object_list = []
phys_object_dict = {}
joints = {}
pose_bones = {}


def get_armature():
    if type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
        #we have the mesh selected, grab the armature
        return get_armature_from_mesh(bpy.context.scene.objects.active)
    elif type(bpy.context.scene.objects.active.data) == bpy.types.Armature:
        #we have the armature selected, use it.
        return bpy.context.scene.objects.active
    return None


def get_armature_from_mesh(mesh_object):
    if mesh_object.modifiers and len(mesh_object.modifiers) > 0:
            for modifier in mesh_object.modifiers:
                if type(modifier) == bpy.types.ArmatureModifier:
                    return modifier.object
    return None


def build_rig():
    bpy.ops.object.mode_set(mode='OBJECT')
    global ob, amt, phys_object_list, phys_object_dict, joints, pose_bones

    mesh_name = None
    ob = get_armature()
    if type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
        mesh_object = bpy.context.scene.objects.active
        mesh_name = mesh_object.name
    #if not type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
    #    return False, "Selected object is not a mesh"
    if not ob:
        return False, "Could not find armature"

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.layers = ob.layers
    ob.select = True
    bpy.context.scene.objects.active = ob

    amt = None
    phys_object_list = []
    phys_object_dict = {}
    joints = {}
    pose_bones = {}
    amt = ob.data
    ob.select = True
    bpy.context.scene.objects.active = ob
    create_rig(mesh_name, damping = 1)
    object_data = {
        "phys_object_names": list(phys_object_dict.keys()),
        "joint_names": list(joints.keys()),
        "bone_names": list(pose_bones.keys()),
        "stored_pose": {}
    }
    for name in object_data['phys_object_names']:
        object_data['stored_pose'][name] = [(x[0], x[1], x[2], x[3]) for x in bpy.data.objects[name].matrix_world]
    ob.phys_pose_data = json.dumps(object_data)
    #print(json.dumps(object_data))
    return True, "Success!"


class GeneratePhysPoseRig(bpy.types.Operator):
    bl_idname = "object.generate_physpose_rig"
    bl_label = "Generate PhysPose Rig"
    bl_options = {'REGISTER', 'UNDO'}
    def execute(self, context):
        success, message = build_rig()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class DeletePhysPoseRig(bpy.types.Operator):
    bl_idname = "object.delete_physpose_rig"
    bl_label = "Delete PhysPose Rig"
    bl_options = {'REGISTER', 'UNDO'}
    def execute(self, context):
        success, message = delete_rig()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class SetRotationsPhysPoseRig(bpy.types.Operator):
    bl_idname = "object.set_rotations_physpose_rig"
    bl_label = "Apply PhysPose to Rig"
    def execute(self, context):
        success, message = set_rotations()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class UnmuteConstraintsPhysPoseRig(bpy.types.Operator):
    bl_idname = "object.unmute_constraints_physpose_rig"
    bl_label = "Unmute Constraints on PhysPose Rig"
    def execute(self, context):
        success, message = unmute_constraints()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class SetDampingPhysPoseRig(bpy.types.Operator):
    bl_idname = "object.set_damping_physpose_rig"
    bl_label = "SetDampingPhysPoseRig"
    def execute(self, context):
        success, message = set_damping()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class SetSpineStiffnessPhysPoseRig(bpy.types.Operator):
    bl_idname = "object.set_spine_stiffness_physpose_rig"
    bl_label = "SetSpineStiffnessPhysPoseRig"
    def execute(self, context):
        success, message = set_spine_stiffness()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class PinPhysObject(bpy.types.Operator):
    bl_idname = "object.pin_phys_object"
    bl_label = "PinPhysObject"
    def execute(self, context):
        success, message = pin_phys_object()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class UnpinPhysObject(bpy.types.Operator):
    bl_idname = "object.unpin_phys_object"
    bl_label = "PinPhysObject"
    def execute(self, context):
        success, message = unpin_phys_object()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}


class ResetPhysPoseRig(bpy.types.Operator):
    bl_idname = "object.reset_physpose_rig"
    bl_label = "Reset PhysPose Rig"
    def execute(self, context):
        success, message = restore_physpose_rig()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}



class PhysPosePanel(bpy.types.Panel):
    bl_label = "PhysPose Tools"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
 
    def draw(self, context):
        ob = context.object
        layout = self.layout

        layout.operator("object.generate_physpose_rig", text='Generate PhysPose Rig')
        if ob.phys_pose_data != '':
            layout.operator("object.delete_physpose_rig", text='Delete PhysPose Rig')
            layout.label("Keyframe Tools")
            layout.operator("object.set_rotations_physpose_rig", text='Apply PhysPose to Rig')
            layout.operator("object.unmute_constraints_physpose_rig", text='Unmute Constraints on PhysPose Rig')
            layout.label("Set Damping")
            row = layout.row()
            row.prop(ob, 'phys_pose_damping', slider=True)
            row.operator("object.set_damping_physpose_rig", text='Set Damping')
            row2 = layout.row()
            row2.prop(ob, 'phys_pose_spine_stiffness', slider=True)
            row2.operator("object.set_spine_stiffness_physpose_rig", text='Set Spine Stiffness')
        layout.label("PhysRig Tools")
        if ob.phys_pose_data != '':
            layout.operator("object.reset_physpose_rig", text='Reset PhysPose Rig to Base Pose')
        layout.operator("object.pin_phys_object", text='Pin Object(s)')
        layout.operator("object.unpin_phys_object", text='Unpin Object(s)')


def register():
    bpy.utils.register_class(GeneratePhysPoseRig)
    bpy.utils.register_class(DeletePhysPoseRig)
    bpy.utils.register_class(SetRotationsPhysPoseRig)
    bpy.utils.register_class(UnmuteConstraintsPhysPoseRig)
    bpy.utils.register_class(SetDampingPhysPoseRig)
    bpy.utils.register_class(PinPhysObject)
    bpy.utils.register_class(UnpinPhysObject)
    bpy.utils.register_class(ResetPhysPoseRig)
    bpy.utils.register_class(SetSpineStiffnessPhysPoseRig)

    bpy.utils.register_class(PhysPosePanel)
    bpy.types.Object.phys_pose_data = bpy.props.StringProperty(name = "PhysPoseData")
    bpy.types.Object.phys_pose_damping = bpy.props.FloatProperty(name = "PhysPoseDamping", default=1.0, min=0.0, max=1.0)
    bpy.types.Object.phys_pose_spine_stiffness = bpy.props.FloatProperty(name = "PhysPoseSpineStiffness", default=1.0, min=0.0, max=1.0)


def unregister():
    bpy.utils.unregister_class(GeneratePhysPoseRig)
    bpy.utils.unregister_class(DeletePhysPoseRig)
    bpy.utils.unregister_class(SetRotationsPhysPoseRig)
    bpy.utils.unregister_class(UnmuteConstraintsPhysPoseRig)
    bpy.utils.unregister_class(SetDampingPhysPoseRig)
    bpy.utils.unregister_class(PinPhysObject)
    bpy.utils.unregister_class(UnpinPhysObject)
    bpy.utils.unregister_class(ResetPhysPoseRig)
    bpy.utils.unregister_class(SetSpineStiffnessPhysPoseRig)

    bpy.utils.unregister_class(PhysPosePanel)
    del bpy.types.Object.phys_pose_data
    del bpy.types.Object.phys_pose_damping
    del bpy.types.Object.phys_pose_spine_stiffness


if __name__ == "__main__":
    register()
