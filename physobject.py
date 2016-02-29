import bpy
import math
from mathutils import Vector, Matrix
import json


class PhysObject():
    def __init__(self, armature, pose_bone, parameters = None, shrinkwrap_object_name = None):
        self.armature = armature
        self.pose_bone = pose_bone
        self.phys_object = None

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
        self.phys_object.rigid_body.use_margin = True
        self.phys_object.rigid_body.collision_margin = 0.001
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
    def __init__(self, phys_object1, phys_object2, parameters=[-180,180,-180,180,-180,180,True]):
        self.phys_object1 = phys_object1
        self.phys_object2 = phys_object2
        self.parameters = parameters
        # If objects already exist, load them up and bypass creation
        if self.get_name() in bpy.data.objects:
            self.bone_constraint = bpy.data.objects[self.get_name()]
        else:
            self.create_constraint()

    def point_constraint(self, scalars):
        if len(scalars) < 3:
            return False
        p = self.parameters
        con = self.bone_constraint
        for i in range(3):
            rmin = p[i]
            rmax = p[i+1]
            dist = rmax - rmin
            dist *= ((scalars[i] + 1.0) / 2.0)
            pos = rmin + dist

            p[i] = max(pos - 5.0, rmin)
            p[i+1] = min(pos + 5.0, rmax)
        self._setup_constraint(con, p, 1.0)

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
    def serialize(self):
        data = {
            'phys_objects': [],
            'constraints': [],
            'armature': self.armature.name,
            'shrinkwrap_object_name': self.shrinkwrap_object_name,
            'stored_pose': {}
        }
        for phys_object in self.phys_objects:
            po = {
                'phys_object': phys_object.phys_object.name,
                'armature': phys_object.armature.name,
                'pose_bone': phys_object.pose_bone.name
            }
            data['phys_objects'].append(po)
            data['stored_pose'][phys_object.phys_object.name] = [(x[0], x[1], x[2], x[3]) for x in bpy.data.objects[phys_object.phys_object.name].matrix_world]
        for constraint in self.constraints:
            co = {
                'phys_object1': constraint.phys_object1.phys_object.name,
                'phys_object2': constraint.phys_object2.phys_object.name,
                'parameters': constraint.parameters,
                'bone_constraint': constraint.bone_constraint.name
            }
            data['constraints'].append(co)
        return json.dumps(data)

    def deserialize(self, json_string):
        data = json.loads(json_string)
        self.armature = bpy.data.objects[data['armature']]
        #self.shrinkwrap_obj_name
        phys_obj_dict = {}
        for phys_obj in data['phys_objects']:
            pose_bone_name = phys_obj['pose_bone']
            po = PhysObject(self.armature, self.armature.pose.bones[pose_bone_name], None, None)
            self.phys_objects.append(po)
            phys_obj_dict[po.get_name()] = po
        for con in data['constraints']:
            self.constraints.append(Constraint(phys_obj_dict[con['phys_object1']], phys_obj_dict[con['phys_object2']], con['parameters']))

    def __init__(self, armature):
        armature.select = True
        bpy.context.scene.objects.active = armature
        self.armature = armature
        self.phys_objects = []
        self.constraints = []
        self.shrinkwrap_object_name = None

    def set_rotations(self, scalars, bones=None, override_iterations=None):
        if len(scalars) < 3:
            return False
        for constraint in self.constraints:
            if bones is None\
                    or (constraint.phys_object1.pose_bone.name in bones and constraint.phys_object2.pose_bone.name in bones):
                constraint.point_constraint(scalars)
                if override_iterations is not None:
                    constraint.bone_constraint.rigid_body_constraint.use_override_solver_iterations = True
                    constraint.bone_constraint.rigid_body_constraint.solver_iterations = override_iterations
                else:
                    constraint.bone_constraint.rigid_body_constraint.use_override_solver_iterations = False

    def set_stiffness(self, scalar, bones=None, override_iterations=None):
        for constraint in self.constraints:
            if bones is None\
                    or (constraint.phys_object1.pose_bone.name in bones and constraint.phys_object2.pose_bone.name in bones):
                #scale by scalar (or scalars)
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
                        and 'PHYSPOSE-COPY-LOCATION' in con.name or 'PHYSPOSE-COPY-ROTATION' in con.name:
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
            constraints.append( Constraint(attach_phys, phys_objects[0], constraint_parameters) )
        else:
            print("Unable to find physobject ", attach_bone_name)

        self.phys_objects += phys_objects
        self.constraints += constraints

    def hide_constraints(self):
        for con in self.constraints:
            con.bone_constraint.hide = True

    def apply_stiffness_map(self, stiffness):
        for m in stiffness:
            iterations = None
            if len(m) > 2:
                iterations = m[2]
            self.set_stiffness(m[0], m[1], iterations)

    def create_rig(self, shrinkwrap_object_name=None, damping=1.0):
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
        self.shrinkwrap_object_name = shrinkwrap_object_name
        phys_map = {}
        for bone_name, params in bones_to_create.items():
            po = PhysObject(self.armature, self.armature.pose.bones[bone_name], params, shrinkwrap_object_name)
            self.phys_objects.append( po )
            phys_map[bone_name] = po

        for constraint in constraints:
            bone_name1, bone_name2, parameters = constraint
            con = Constraint(phys_map[bone_name1], phys_map[bone_name2], parameters)
            self.constraints.append(con)

        minimize_twist = [
            [0.6, ["ShldrBend.R","ShldrTwist.R","ShldrBend.L","ShldrTwist.L"], 25],
            [0.6, ["ThighBend.R","ThighTwist.R","ThighBend.L","ThighTwist.L"], 25],
            [0.6, ["ForearmBend.R","ForearmTwist.R","ForearmBend.L","ForearmTwist.L"], 25]
        ]

        self.apply_stiffness_map(minimize_twist)

    def delete_rig(self):
        bpy.ops.object.mode_set(mode='OBJECT')
        armature = self.armature
        if not armature:
            return False, "Object is not supported type (Armature or Mesh)"

        bpy.ops.object.select_all(action='DESELECT')
        physpose_data = json.loads(armature.phys_pose_data)
        bpy.context.scene.layers = armature.layers

        armature.pose.bones["HandIk.R"].matrix = armature.pose.bones["Hand.R"].matrix.copy()
        armature.pose.bones["HandIk.L"].matrix = armature.pose.bones["Hand.L"].matrix.copy()
        armature.pose.bones["FootIk.R"].matrix = armature.pose.bones["Foot.R"].matrix.copy()
        armature.pose.bones["FootIk.L"].matrix = armature.pose.bones["Foot.L"].matrix.copy()

        for phys in self.phys_objects:
            name = phys.phys_object.name
            for con in phys.pose_bone.constraints:
                if type(con) in [bpy.types.CopyRotationConstraint, bpy.types.CopyLocationConstraint] \
                    and con.name == 'PHYSPOSE-COPY-LOCATION' or con.name == 'PHYSPOSE-COPY-ROTATION':
                    phys.pose_bone.constraints.remove(con)
                elif con.mute:
                    con.mute = False

        for phys in self.phys_objects:
            name = phys.phys_object.name
            bpy.data.objects[name].select = True
        for con in self.constraints:
            name = con.bone_constraint.name
            bpy.data.objects[name].select = True

        bpy.ops.object.delete()
        return True, "Success"

def print_total_weight(poserig):
    total = 0
    for phys in poserig.phys_objects:
        total += phys.phys_object.rigid_body.mass
    print("Total weight: ", total)

stiffness_map = [
    [0.01, ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower","neckUpper"]],
    [0.75, ["Hand.R", "Hand.L", "Index1.R", "Mid1.R", "Ring1.R", "Pinky1.R", "Thumb2.R", "Index1.L", "Mid1.L", "Ring1.L", "Pinky1.L", "Thumb2.L"]],
    [0.2, ["head","neckUpper"]],
    [0.25, ["chestUpper","Collar.R","Collar.L"]]
]

rigs = {}
custom_template = {}

def GenesisRig1(poserig):
    braid_dof = 6.0
    poserig.build_chain("genitals", "pelvis", "shedick", bone_size=0.15, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.apply_stiffness_map(stiffness_map)
    poserig.hide_constraints()


def GenesisRig3(poserig):
    braid_dof = 65.0
    poserig.build_chain("braid.R", "head", "FTBraids-skinInstance", bone_size=0.17, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("braid.L", "head", "FTBraids-skinInstance", bone_size=0.17, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("earring.R", "head", "LEarring_3_11775.001", bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("earring.L", "head", "LEarring_3_11775",     bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("sword", "pelvis", "Sword_Centre_4713", bone_size=0.1, bone_dof=(180, 180, 180))
    poserig.apply_stiffness_map(stiffness_map)
custom_template['GenesisRig3'] = GenesisRig3
custom_template['GenesisRig1'] = GenesisRig1


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


def create_rig():
    bpy.ops.object.mode_set(mode='OBJECT')
    shrink_wrap_name = None
    ob = get_armature()
    if type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
        mesh_object = bpy.context.scene.objects.active
        shrink_wrap_name = mesh_object.name
    #if not type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
    #    return False, "Selected object is not a mesh"
    if not ob:
        return False, "Could not find armature"
    if not shrink_wrap_name:
        return False, "Could not find mesh"
    armature_name = ob.name
    poserig = PhysPoseRig(bpy.data.objects[armature_name])
    poserig.clear_constraints()
    poserig.create_rig(shrink_wrap_name)
    if armature_name in custom_template:
        custom_template[armature_name](poserig)
    poserig.hide_constraints()
    rigs[armature_name] = poserig
    ob.phys_pose_data = rigs[armature_name].serialize()

def load_rig(json_data):
    data = json.loads(json_data)
    ob = bpy.data.objects[data['armature']]
    poserig = PhysPoseRig(ob)
    poserig.deserialize(json_data)
    return poserig

physpose_data = """{"constraints": [{"phys_object2": "PHYS-GenesisRig1-abdomenLower", "phys_object1": "PHYS-GenesisRig1-pelvis", "parameters": [-20, 35, -15, 15, -15, 15, true], "bone_constraint": "JOINT-GenesisRig1-pelvis-abdomenLower"}, {"phys_object2": "PHYS-GenesisRig1-abdomenUpper", "phys_object1": "PHYS-GenesisRig1-abdomenLower", "parameters": [-25, 40, -20, 20, -24, 24, true], "bone_constraint": "JOINT-GenesisRig1-abdomenLower-abdomenUpper"}, {"phys_object2": "PHYS-GenesisRig1-chestLower", "phys_object1": "PHYS-GenesisRig1-abdomenUpper", "parameters": [-25, 35, -12, 12, -20, 20, true], "bone_constraint": "JOINT-GenesisRig1-abdomenUpper-chestLower"}, {"phys_object2": "PHYS-GenesisRig1-chestUpper", "phys_object1": "PHYS-GenesisRig1-chestLower", "parameters": [-15, 15, -10, 10, -10, 10, true], "bone_constraint": "JOINT-GenesisRig1-chestLower-chestUpper"}, {"phys_object2": "PHYS-GenesisRig1-neckLower", "phys_object1": "PHYS-GenesisRig1-chestUpper", "parameters": [-15, 30, -22, 22, -40, 40, true], "bone_constraint": "JOINT-GenesisRig1-chestUpper-neckLower"}, {"phys_object2": "PHYS-GenesisRig1-neckUpper", "phys_object1": "PHYS-GenesisRig1-neckLower", "parameters": [-17, 12, -22, 22, -10, 10, true], "bone_constraint": "JOINT-GenesisRig1-neckLower-neckUpper"}, {"phys_object2": "PHYS-GenesisRig1-head", "phys_object1": "PHYS-GenesisRig1-neckUpper", "parameters": [-27, 25, -22, 22, -20, 20, true], "bone_constraint": "JOINT-GenesisRig1-neckUpper-head"}, {"phys_object2": "PHYS-GenesisRig1-Collar.L", "phys_object1": "PHYS-GenesisRig1-chestUpper", "parameters": [-26, 17, -30, 30, -10, 50, true], "bone_constraint": "JOINT-GenesisRig1-chestUpper-Collar.L"}, {"phys_object2": "PHYS-GenesisRig1-Collar.R", "phys_object1": "PHYS-GenesisRig1-chestUpper", "parameters": [-17, 26, -30, 30, -50, 10, true], "bone_constraint": "JOINT-GenesisRig1-chestUpper-Collar.R"}, {"phys_object2": "PHYS-GenesisRig1-ShldrBend.L", "phys_object1": "PHYS-GenesisRig1-Collar.L", "parameters": [-85, 35, 0, 0, -110, 40, true], "bone_constraint": "JOINT-GenesisRig1-Collar.L-ShldrBend.L"}, {"phys_object2": "PHYS-GenesisRig1-ShldrBend.R", "phys_object1": "PHYS-GenesisRig1-Collar.R", "parameters": [-35, 85, 0, 0, -40, 110, true], "bone_constraint": "JOINT-GenesisRig1-Collar.R-ShldrBend.R"}, {"phys_object2": "PHYS-GenesisRig1-ThighBend.L", "phys_object1": "PHYS-GenesisRig1-pelvis", "parameters": [-115, 35, -20, 20, -85, 20, true], "bone_constraint": "JOINT-GenesisRig1-pelvis-ThighBend.L"}, {"phys_object2": "PHYS-GenesisRig1-ThighBend.R", "phys_object1": "PHYS-GenesisRig1-pelvis", "parameters": [-115, 35, -20, 20, -25, 85, true], "bone_constraint": "JOINT-GenesisRig1-pelvis-ThighBend.R"}, {"phys_object2": "PHYS-GenesisRig1-ThighTwist.L", "phys_object1": "PHYS-GenesisRig1-ThighBend.L", "parameters": [0, 0, -55, 55, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ThighBend.L-ThighTwist.L"}, {"phys_object2": "PHYS-GenesisRig1-ThighTwist.R", "phys_object1": "PHYS-GenesisRig1-ThighBend.R", "parameters": [0, 0, -55, 55, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ThighBend.R-ThighTwist.R"}, {"phys_object2": "PHYS-GenesisRig1-ShldrTwist.L", "phys_object1": "PHYS-GenesisRig1-ShldrBend.L", "parameters": [0, 0, -95, 80, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ShldrBend.L-ShldrTwist.L"}, {"phys_object2": "PHYS-GenesisRig1-ShldrTwist.R", "phys_object1": "PHYS-GenesisRig1-ShldrBend.R", "parameters": [0, 0, -95, 80, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ShldrBend.R-ShldrTwist.R"}, {"phys_object2": "PHYS-GenesisRig1-ForearmTwist.L", "phys_object1": "PHYS-GenesisRig1-ForearmBend.L", "parameters": [0, 0, -90, 80, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ForearmBend.L-ForearmTwist.L"}, {"phys_object2": "PHYS-GenesisRig1-ForearmTwist.R", "phys_object1": "PHYS-GenesisRig1-ForearmBend.R", "parameters": [0, 0, -90, 80, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ForearmBend.R-ForearmTwist.R"}, {"phys_object2": "PHYS-GenesisRig1-Shin.R", "phys_object1": "PHYS-GenesisRig1-ThighTwist.R", "parameters": [0, 0, -12, 12, 0, 140, true], "bone_constraint": "JOINT-GenesisRig1-ThighTwist.R-Shin.R"}, {"phys_object2": "PHYS-GenesisRig1-Shin.L", "phys_object1": "PHYS-GenesisRig1-ThighTwist.L", "parameters": [0, 0, -12, 12, -140, 0, true], "bone_constraint": "JOINT-GenesisRig1-ThighTwist.L-Shin.L"}, {"phys_object2": "PHYS-GenesisRig1-ForearmBend.R", "phys_object1": "PHYS-GenesisRig1-ShldrTwist.R", "parameters": [-135, 20, 0, 0, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ShldrTwist.R-ForearmBend.R"}, {"phys_object2": "PHYS-GenesisRig1-ForearmBend.L", "phys_object1": "PHYS-GenesisRig1-ShldrTwist.L", "parameters": [-135, 20, 0, 0, 0, 0, true], "bone_constraint": "JOINT-GenesisRig1-ShldrTwist.L-ForearmBend.L"}, {"phys_object2": "PHYS-GenesisRig1-Hand.L", "phys_object1": "PHYS-GenesisRig1-ForearmTwist.L", "parameters": [-40, 40, -40, 40, -90, 90, true], "bone_constraint": "JOINT-GenesisRig1-ForearmTwist.L-Hand.L"}, {"phys_object2": "PHYS-GenesisRig1-Hand.R", "phys_object1": "PHYS-GenesisRig1-ForearmTwist.R", "parameters": [-40, 40, -40, 40, -90, 90, true], "bone_constraint": "JOINT-GenesisRig1-ForearmTwist.R-Hand.R"}, {"phys_object2": "PHYS-GenesisRig1-Foot.L", "phys_object1": "PHYS-GenesisRig1-Shin.L", "parameters": [-75, 40, -12, 12, -10, 10, true], "bone_constraint": "JOINT-GenesisRig1-Shin.L-Foot.L"}, {"phys_object2": "PHYS-GenesisRig1-Foot.R", "phys_object1": "PHYS-GenesisRig1-Shin.R", "parameters": [-75, 40, -12, 12, -10, 10, true], "bone_constraint": "JOINT-GenesisRig1-Shin.R-Foot.R"}, {"phys_object2": "PHYS-GenesisRig1-Index1.R", "phys_object1": "PHYS-GenesisRig1-Hand.R", "parameters": [-5, 5, -2, 2, -60, 5, true], "bone_constraint": "JOINT-GenesisRig1-Hand.R-Index1.R"}, {"phys_object2": "PHYS-GenesisRig1-Mid1.R", "phys_object1": "PHYS-GenesisRig1-Hand.R", "parameters": [-5, 5, -2, 2, -60, 5, true], "bone_constraint": "JOINT-GenesisRig1-Hand.R-Mid1.R"}, {"phys_object2": "PHYS-GenesisRig1-Ring1.R", "phys_object1": "PHYS-GenesisRig1-Hand.R", "parameters": [-5, 5, -2, 2, -60, 5, true], "bone_constraint": "JOINT-GenesisRig1-Hand.R-Ring1.R"}, {"phys_object2": "PHYS-GenesisRig1-Pinky1.R", "phys_object1": "PHYS-GenesisRig1-Hand.R", "parameters": [-5, 5, -2, 2, -60, 5, true], "bone_constraint": "JOINT-GenesisRig1-Hand.R-Pinky1.R"}, {"phys_object2": "PHYS-GenesisRig1-Thumb2.R", "phys_object1": "PHYS-GenesisRig1-Hand.R", "parameters": [-30, 30, -30, 30, -30, 30, true], "bone_constraint": "JOINT-GenesisRig1-Hand.R-Thumb2.R"}, {"phys_object2": "PHYS-GenesisRig1-Index1.L", "phys_object1": "PHYS-GenesisRig1-Hand.L", "parameters": [-5, 5, -2, 2, -5, 60, true], "bone_constraint": "JOINT-GenesisRig1-Hand.L-Index1.L"}, {"phys_object2": "PHYS-GenesisRig1-Mid1.L", "phys_object1": "PHYS-GenesisRig1-Hand.L", "parameters": [-5, 5, -2, 2, -5, 60, true], "bone_constraint": "JOINT-GenesisRig1-Hand.L-Mid1.L"}, {"phys_object2": "PHYS-GenesisRig1-Ring1.L", "phys_object1": "PHYS-GenesisRig1-Hand.L", "parameters": [-5, 5, -2, 2, -5, 60, true], "bone_constraint": "JOINT-GenesisRig1-Hand.L-Ring1.L"}, {"phys_object2": "PHYS-GenesisRig1-Pinky1.L", "phys_object1": "PHYS-GenesisRig1-Hand.L", "parameters": [-5, 5, -2, 2, -5, 60, true], "bone_constraint": "JOINT-GenesisRig1-Hand.L-Pinky1.L"}, {"phys_object2": "PHYS-GenesisRig1-Thumb2.L", "phys_object1": "PHYS-GenesisRig1-Hand.L", "parameters": [-30, 30, -30, 30, -30, 30, true], "bone_constraint": "JOINT-GenesisRig1-Hand.L-Thumb2.L"}, {"phys_object2": "PHYS-GenesisRig1-genitals.004", "phys_object1": "PHYS-GenesisRig1-genitals", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals-genitals.004"}, {"phys_object2": "PHYS-GenesisRig1-genitals.002", "phys_object1": "PHYS-GenesisRig1-genitals.004", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.004-genitals.002"}, {"phys_object2": "PHYS-GenesisRig1-genitals.006", "phys_object1": "PHYS-GenesisRig1-genitals.002", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.002-genitals.006"}, {"phys_object2": "PHYS-GenesisRig1-genitals.001", "phys_object1": "PHYS-GenesisRig1-genitals.006", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.006-genitals.001"}, {"phys_object2": "PHYS-GenesisRig1-genitals.005", "phys_object1": "PHYS-GenesisRig1-genitals.001", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.001-genitals.005"}, {"phys_object2": "PHYS-GenesisRig1-genitals.003", "phys_object1": "PHYS-GenesisRig1-genitals.005", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.005-genitals.003"}, {"phys_object2": "PHYS-GenesisRig1-genitals.007", "phys_object1": "PHYS-GenesisRig1-genitals.003", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-genitals.003-genitals.007"}, {"phys_object2": "PHYS-GenesisRig1-genitals", "phys_object1": "PHYS-GenesisRig1-pelvis", "parameters": [-6.0, 6.0, -6.0, 6.0, -6.0, 6.0, true], "bone_constraint": "JOINT-GenesisRig1-pelvis-genitals"}], "phys_objects": [{"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Thumb2.L", "pose_bone": "Thumb2.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-head", "pose_bone": "head"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Foot.L", "pose_bone": "Foot.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Collar.L", "pose_bone": "Collar.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ForearmBend.R", "pose_bone": "ForearmBend.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Ring1.R", "pose_bone": "Ring1.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Collar.R", "pose_bone": "Collar.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ForearmBend.L", "pose_bone": "ForearmBend.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-chestLower", "pose_bone": "chestLower"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Thumb2.R", "pose_bone": "Thumb2.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-neckUpper", "pose_bone": "neckUpper"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ForearmTwist.L", "pose_bone": "ForearmTwist.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ShldrBend.L", "pose_bone": "ShldrBend.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ShldrBend.R", "pose_bone": "ShldrBend.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-pelvis", "pose_bone": "pelvis"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ThighTwist.L", "pose_bone": "ThighTwist.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Shin.R", "pose_bone": "Shin.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ThighBend.R", "pose_bone": "ThighBend.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Mid1.R", "pose_bone": "Mid1.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Hand.L", "pose_bone": "Hand.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Index1.R", "pose_bone": "Index1.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Pinky1.R", "pose_bone": "Pinky1.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Index1.L", "pose_bone": "Index1.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-chestUpper", "pose_bone": "chestUpper"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ForearmTwist.R", "pose_bone": "ForearmTwist.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ThighTwist.R", "pose_bone": "ThighTwist.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ShldrTwist.R", "pose_bone": "ShldrTwist.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-abdomenLower", "pose_bone": "abdomenLower"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Hand.R", "pose_bone": "Hand.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Foot.R", "pose_bone": "Foot.R"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Mid1.L", "pose_bone": "Mid1.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Shin.L", "pose_bone": "Shin.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-abdomenUpper", "pose_bone": "abdomenUpper"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ThighBend.L", "pose_bone": "ThighBend.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Pinky1.L", "pose_bone": "Pinky1.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-neckLower", "pose_bone": "neckLower"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-ShldrTwist.L", "pose_bone": "ShldrTwist.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-Ring1.L", "pose_bone": "Ring1.L"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals", "pose_bone": "genitals"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.004", "pose_bone": "genitals.004"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.002", "pose_bone": "genitals.002"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.006", "pose_bone": "genitals.006"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.001", "pose_bone": "genitals.001"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.005", "pose_bone": "genitals.005"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.003", "pose_bone": "genitals.003"}, {"armature": "GenesisRig1", "phys_object": "PHYS-GenesisRig1-genitals.007", "pose_bone": "genitals.007"}], "armature": "GenesisRig1"}"""
#create_rig()
poserig = load_rig(physpose_data)
poserig.delete_rig()

#rigmap = [['GenesisRig3', 'Genesis3Female.002'], ['GenesisRig1', 'Genesis3Female']]
#for rigname, meshname in rigmap:
#    bpy.ops.object.select_all(action='DESELECT')
#    bpy.context.scene.layers = bpy.data.objects[rigname].layers
#    create_rig(rigname, meshname)

#rigs[0].set_rotations((1.0, 0.0, 0.0), ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower","neckUpper"])
"""
def draw_phys_as(type):
  for obj in bpy.data.objects:
    if obj.name[0:4] == 'PHYS':
        obj.draw_type = type
for obj in bpy.data.objects:
    print(obj.name)
    if obj.name[0:5] == 'JOINT':
        print(obj.name)
        obj.hide = True
draw_phys_as('TEXTURED')
"""