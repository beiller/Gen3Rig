import bpy
import math
from mathutils import Vector, Matrix
import json


class PhysObject(): 

    phys_object = None
    armature = None
    pose_bone = None

    def __init__(self, armature, pose_bone, parameters = None, shrinkwrap_object_name = None):
        self.armature = armature
        self.pose_bone = pose_bone

        # If objects already exist, load them up and bypass creation
        if self.get_name() in bpy.data.objects:
            self.phys_object = bpy.data.objects[self.get_name()]
        else:
            if not parameters:
                parameters = [0.1, 0.1]
            size_x, size_z, length, offset, copy_location = (parameters + [None] * 5)[:5]
            self.create_cube_at_bone(size_x, size_z, length, offset, copy_location)

            if shrinkwrap_object_name:
                self.shrinkwrap_to_object(shrinkwrap_object_name)


    def _create_rect(self, sx, sy, sz, name, offset = None):
        """
            Creates 3D rectangle with size x (sx), y, and z.
             Y is scaled to 0.0 ... 1.0 range
        """
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
        self.phys_object = cube_object
        return cube_object


    def get_name(self):
        return 'PHYS-%s-%s' % (self.armature.name, self.pose_bone.name)


    def create_copy_rotation_constraint(self):
        for con in self.pose_bone.constraints:
            if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.KinematicConstraint]:
                con.mute = True
        const = self.pose_bone.constraints.new('COPY_ROTATION')
        const.name = 'PHYSPOSE-COPY-ROTATION'
        const.target = self.phys_object


    def create_copy_location_constraint(self):
        const = self.pose_bone.constraints.new('COPY_LOCATION')
        const.name = 'PHYSPOSE-COPY-LOCATION'
        const.target = self.phys_object


    def create_cube_at_bone(self, size_x = 0.1, size_z = 0.1, length = None, offset = None, copy_location = None):
        bone_dist = Vector(self.pose_bone.tail) - Vector(self.pose_bone.head)
        if not length:
            length = bone_dist.length
        self._create_rect(size_x, length, size_z, self.get_name(), offset)
        self.phys_object.layers = self.armature.layers

        mat = Matrix()
        mat[0][0:4] = (self.pose_bone.x_axis[0], self.pose_bone.y_axis[0], self.pose_bone.z_axis[0], self.pose_bone.head[0])
        mat[1][0:4] = (self.pose_bone.x_axis[1], self.pose_bone.y_axis[1], self.pose_bone.z_axis[1], self.pose_bone.head[1])
        mat[2][0:4] = (self.pose_bone.x_axis[2], self.pose_bone.y_axis[2], self.pose_bone.z_axis[2], self.pose_bone.head[2])
        self.phys_object.matrix_world = mat
        self.create_copy_rotation_constraint()
        if copy_location:
            self.create_copy_location_constraint()
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        self.phys_object.select = True
        bpy.context.scene.objects.active = self.phys_object
        bpy.ops.rigidbody.objects_add(type='ACTIVE')
        bpy.ops.rigidbody.mass_calculate(density=1100) #1.1g/m^3
        self.phys_object.rigid_body.angular_damping = 0.75
        self.phys_object.rigid_body.linear_damping = 0.75
        #self.phys_object.rigid_body.use_margin = True
        #self.phys_object.rigid_body.collision_margin = 0.001
        sx, sy, sz = self.phys_object.dimensions
        """
            470 is an adjustment factor because mass is calculated based on bounds
            so we "shave some off"
        """
        self.phys_object.rigid_body.mass = sx * sy * sz * 470 
        self.phys_object.rigid_body.collision_shape = 'CONVEX_HULL'
        self.phys_object.draw_type = 'BOUNDS'


    def shrinkwrap_to_object(self, shrinkwrap_object_name):
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        self.phys_object.select = True
        bpy.context.scene.objects.active = self.phys_object
        ss = self.phys_object.modifiers.new('subsurf-to-apply', 'SUBSURF')
        ss.levels = 2
        ss.subdivision_type = 'SIMPLE'
        sr = self.phys_object.modifiers.new('shrinkwrap-to-apply', 'SHRINKWRAP')
        sr.target = bpy.data.objects[shrinkwrap_object_name]
        sm = self.phys_object.modifiers.new('smooth-to-apply', 'SMOOTH')
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="subsurf-to-apply")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="shrinkwrap-to-apply")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="smooth-to-apply")
        #Recalculate mass because our shape has changed
        sx, sy, sz = self.phys_object.dimensions
        """
            470 is an adjustment factor because mass is calculated based on bounds
            so we "shave some off"
        """
        self.phys_object.rigid_body.mass = sx * sy * sz * 470


    def set_damping(self, angular_damping_setting, linear_damping_setting = None):
        if not linear_damping_setting:
            linear_damping_setting = angular_damping_setting
        if self.phys_object: 
            self.phys_object.rigid_body.angular_damping = angular_damping_setting
            self.phys_object.rigid_body.linear_damping = linear_damping_setting



class Constraint:
    phys_object1 = None
    phys_object2 = None
    parameters = None
    bone_constraint = None


    def __init__(self, phys_object1, phys_object2, parameters=[-180,180,-180,180,-180,180,True]):
        self.phys_object1 = phys_object1
        self.phys_object2 = phys_object2
        self.parameters = parameters
        # If objects already exist, load them up and bypass creation
        if self.get_name() in bpy.data.objects:
            self.bone_constraint = bpy.data.objects[self.get_name()]
        else:
            self.create_constraint()


    def _setup_constraint(self, con, p, scalar = 1.0):
        """
            Parameters:
                con: constraint
                p: parameters for constraint (limit x, limit y, limit z, disable_collisions option)
        """
        cr = [0,0,0]
        con.rigid_body_constraint.use_limit_lin_x = True
        con.rigid_body_constraint.limit_lin_x_lower = 0
        con.rigid_body_constraint.limit_lin_x_upper = 0
        con.rigid_body_constraint.use_limit_lin_y = True
        con.rigid_body_constraint.limit_lin_y_lower = 0
        con.rigid_body_constraint.limit_lin_y_upper = 0
        con.rigid_body_constraint.use_limit_lin_z = True
        con.rigid_body_constraint.limit_lin_z_lower = 0
        con.rigid_body_constraint.limit_lin_z_upper = 0

        con.rigid_body_constraint.use_limit_ang_x = True
        con.rigid_body_constraint.limit_ang_x_lower = math.radians(p[0] - math.degrees(cr[0])) * scalar
        con.rigid_body_constraint.limit_ang_x_upper = math.radians(p[1] - math.degrees(cr[0])) * scalar
        con.rigid_body_constraint.use_limit_ang_y = True
        con.rigid_body_constraint.limit_ang_y_lower = math.radians(p[2] + math.degrees(cr[1])) * scalar
        con.rigid_body_constraint.limit_ang_y_upper = math.radians(p[3] + math.degrees(cr[1])) * scalar
        con.rigid_body_constraint.use_limit_ang_z = True
        con.rigid_body_constraint.limit_ang_z_lower = math.radians(p[4] + math.degrees(cr[2])) * scalar
        con.rigid_body_constraint.limit_ang_z_upper = math.radians(p[5] + math.degrees(cr[2])) * scalar
        con.rigid_body_constraint.disable_collisions = p[6]


    def get_name(self):
        return 'JOINT-%s-%s-%s' % (self.phys_object1.armature.name, self.phys_object1.pose_bone.name, self.phys_object2.pose_bone.name)


    def create_constraint(self):
        bone = self.phys_object2.pose_bone
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.empty_add(type='ARROWS', location=bone.head)
        mat = Matrix()
        mat[0][0:4] = (bone.x_axis[0], bone.y_axis[0], bone.z_axis[0], bone.head[0])
        mat[1][0:4] = (bone.x_axis[1], bone.y_axis[1], bone.z_axis[1], bone.head[1])
        mat[2][0:4] = (bone.x_axis[2], bone.y_axis[2], bone.z_axis[2], bone.head[2])
        con = bpy.context.scene.objects.active
        con.matrix_world = mat
        con.layers = self.phys_object1.armature.layers
        bpy.ops.rigidbody.constraint_add()
        con.rigid_body_constraint.type = 'GENERIC'
        con.rigid_body_constraint.object1 = self.phys_object1.phys_object
        con.rigid_body_constraint.object2 = self.phys_object2.phys_object
        con.name = self.get_name()
        self._setup_constraint(con, self.parameters)
        self.bone_constraint = con


class PhysPoseRig():
    phys_objects = []
    constraints = []
    armature = None

    def __init__(self, armature):
        armature.select = True
        bpy.context.scene.objects.active = armature
        self.armature = armature


    def set_stiffness(self, scalar, bones=None, override_iterations=None):
        for constraint in self.constraints:
            print(constraint.phys_object1.pose_bone.name)
            print(constraint.phys_object2.pose_bone.name)
            if bones is None\
                or (constraint.phys_object1.pose_bone.name in bones and constraint.phys_object2.pose_bone.name in bones):
                constraint._setup_constraint(constraint.bone_constraint, constraint.parameters, scalar=scalar)
                if override_iterations is not None:
                    constraint.bone_constraint.rigid_body_constraint.use_override_solver_iterations = True
                    constraint.bone_constraint.rigid_body_constraint.solver_iterations = override_iterations
                else:
                    constraint.bone_constraint.rigid_body_constraint.use_override_solver_iterations = False



    def clear_constraints(self):
        for bone in self.armature.pose.bones:
            for con in bone.constraints:
                if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.CopyLocationConstraint] \
                    and 'PHYSPOSE-COPY-LOCATION' in con.name or 'PHYSPOSE-COPY-ROTATION' in con.name :
                    bone.constraints.remove(con)


    def build_chain(self, start_bone_name, attach_bone_name, shrinkwrap_name = None, chain_length=90, bone_size=0.1, bone_dof=(2, 2, 2)):
        if start_bone_name not in self.armature.pose.bones:
            return False

        phys_objects = []
        phys_objects.append( PhysObject(self.armature, self.armature.pose.bones[start_bone_name], None, shrinkwrap_name) )
        def build_recurse(bone, phys_objects):
            for child in bone.children:
                phys_objects.append( PhysObject(self.armature, child, None, shrinkwrap_name) )
                build_recurse(child, phys_objects)
        build_recurse(self.armature.pose.bones[start_bone_name], phys_objects)

        constraints = []
        #BUILD CONSTRAINTS
        cdx, cdy, cdz = bone_dof
        constraint_parameters = [-cdx,cdx,-cdy,cdy,-cdz,cdz,True]

        for i in range(len(phys_objects)):   
            if i+1 < len(phys_objects):
                constraints.append( Constraint(phys_objects[i], phys_objects[i+1], constraint_parameters) )

        attach_phys = None
        for phys in self.phys_objects:
            if attach_bone_name == phys.phys_object.name.split('-')[2]:
                attach_phys = phys
        if attach_phys:
            constraints.append( Constraint(phys_objects[0], attach_phys, constraint_parameters) )
        else:
            print("Unable to find physobject ", attach_bone_name)

        self.phys_objects += phys_objects
        self.constraints += constraints


    def hide_constraints(self):
        for con in self.constraints:
            con.bone_constraint.hide = True


    def create_rig(self, shrinkwrap_object_name = None, damping = 1.0):
        bones_to_create = {
            "pelvis" : [0.27, 0.18, None, (0.0, -0.16, -0.02), True],
            "abdomenLower" : [0.23, 0.17, None, None, True],
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

        phys_map = {}
        for bone_name, params in bones_to_create.items():
            po = PhysObject(self.armature, self.armature.pose.bones[bone_name], params, shrinkwrap_object_name)
            self.phys_objects.append( po )
            phys_map[bone_name] = po

        for constraint in constraints:
            bone_name1, bone_name2, parameters = constraint
            con = Constraint(phys_map[bone_name1], phys_map[bone_name2], parameters)
            self.constraints.append(con)




#total = 0
#for phys in poserig.phys_objects:
#    total += phys.phys_object.rigid_body.mass
#print("Total weight: ", total)

rigs = []
def rig1():
    poserig = PhysPoseRig(bpy.data.objects["GenesisRig1"])
    poserig.clear_constraints()
    poserig.create_rig("Genesis3Female")
    poserig.build_chain("genitals", "pelvis", "shedick", bone_size=0.15)
    poserig.set_stiffness(0.25, ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower","neckUpper"])
    poserig.set_stiffness(0.4, ["head","neckUpper"])
    poserig.set_stiffness(0.5, ["chestUpper","Collar.R","Collar.L"])
    poserig.hide_constraints()
    rigs.append(poserig)

def rig2():
    poserig = PhysPoseRig(bpy.data.objects["GenesisRig3"])
    poserig.clear_constraints()
    poserig.create_rig("Genesis3Female.002")
    braid_dof = 45.0
    poserig.build_chain("braid.R", "head", "FTBraids-skinInstance", bone_size=0.17, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("braid.L", "head", "FTBraids-skinInstance", bone_size=0.17, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.set_stiffness(0.25, ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower","neckUpper"])
    poserig.set_stiffness(0.4, ["head","neckUpper"])
    poserig.set_stiffness(0.5, ["chestUpper","Collar.R","Collar.L"])
    poserig.hide_constraints()
    rigs.append(poserig)

rigmap = {
    'GenesisRig1': rig1,
    'GenesisRig3': rig2
}
for rig in rigmap:
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.layers = bpy.data.objects[rig].layers
    rigmap[rig]()

