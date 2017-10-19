import bpy
import math

import time
from mathutils import Vector, Matrix
import json
import imp

try:
    from . import custom_template
except ImportError:
    custom_template = None


from . import template as makehuman_template
from . import template_genesis3 as genesis3_template
from . import template_rigify as rigify_template

templates = [
    ('MakeHuman v1', makehuman_template),
    ('Genesis 3 Female', genesis3_template),
    ('Rigify (Pitchipoy)', rigify_template)
]


class PhysObject():
    def __init__(self, armature, pose_bone, parameters=None, shrinkwrap_object_name=None, density=1100):
        self.armature = armature
        self.pose_bone = pose_bone
        self.phys_object = None
        self.density = density
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
        return 'PHYS~%s~%s' % (self.armature.name, self.pose_bone.name)

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
        if length is not None:
            if isinstance(length, str):
                # The length parameter here is a string representing the bone to use for length calculation
                bone_for_dist = self.armature.pose.bones[length]
                bone_dist = Vector(bone_for_dist.tail) - Vector(bone_for_dist.head)
                length = bone_dist.length
        else:
            #  Use the bone that was passed as length
            bone_dist = Vector(self.pose_bone.tail) - Vector(self.pose_bone.head)
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
        bpy.ops.rigidbody.mass_calculate(density=self.density) #1.1g/m^3
        self.phys_object.rigid_body.angular_damping = 0.5
        self.phys_object.rigid_body.linear_damping = 0.5
        self.phys_object.rigid_body.use_margin = True
        self.phys_object.rigid_body.collision_margin = 0.002
        sx, sy, sz = self.phys_object.dimensions
        """
            470 is an adjustment factor because mass is calculated based on bounds
            so we "shave some off"
        """
        self.phys_object.rigid_body.mass = sx * sy * sz * 470 
        self.phys_object.rigid_body.collision_shape = 'CONVEX_HULL'
        #self.phys_object.rigid_body.collision_shape = 'MESH'

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
        sm.iterations = 4
        disp = self.phys_object.modifiers.new('displace-to-apply', 'DISPLACE')
        disp.strength = -0.002
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="subsurf-to-apply")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="shrinkwrap-to-apply")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="smooth-to-apply")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="displace-to-apply")
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

default_constraint_parameters = [-180, 180, -180, 180, -180, 180, True]


class Constraint:
    def __init__(self, phys_object1, phys_object2, parameters=default_constraint_parameters):
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
            rmin = p[i*2]
            rmax = p[i*2+1]
            dist = rmax - rmin
            dist *= ((scalars[i] + 1.0) / 2.0)
            pos = rmin + dist

            p[i*2] = max(pos - 5.0, rmin)
            p[i*2+1] = min(pos + 5.0, rmax)
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
        return 'JOINT~%s~%s~%s' % (self.phys_object1.armature.name, self.phys_object1.pose_bone.name, self.phys_object2.pose_bone.name)

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
            'stored_pose': {},
            'saved_state': {}
        }
        for phys_object in self.phys_objects:
            po = {
                'phys_object': phys_object.phys_object.name,
                'armature': phys_object.armature.name,
                'pose_bone': phys_object.pose_bone.name
            }
            data['phys_objects'].append(po)
            data['stored_pose'][phys_object.phys_object.name] = [(x[0], x[1], x[2], x[3]) for x in self.rest_phys_matrix[phys_object.phys_object.name]]
            data['saved_state'][phys_object.pose_bone.name] = [(x[0], x[1], x[2], x[3]) for x in self.saved_state[phys_object.pose_bone.name]]
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
            self.rest_phys_matrix[po.get_name()] = Matrix(data['stored_pose'][po.get_name()])
            self.saved_state[pose_bone_name] = Matrix(data['saved_state'][pose_bone_name])
        for con in data['constraints']:
            self.constraints.append(Constraint(phys_obj_dict[con['phys_object1']], phys_obj_dict[con['phys_object2']], con['parameters']))

    def __init__(self, armature, template):
        armature.select = True
        bpy.context.scene.objects.active = armature
        self.armature = armature
        self.phys_objects = []
        self.constraints = []
        self.shrinkwrap_object_name = None
        self.rest_phys_matrix = {}
        self.template = template
        self.saved_state = {}

    def set_rest_matrix(self):
        for phys_object in self.phys_objects:
            self.rest_phys_matrix[phys_object.phys_object.name] = bpy.data.objects[phys_object.phys_object.name].matrix_world

    def set_rotations(self, scalars, bones=None, override_iterations=None):
        if len(scalars) < 3:
            return False
        for constraint in self.constraints:
            if bones is None\
                    or (constraint.phys_object1.pose_bone.name in bones or constraint.phys_object2.pose_bone.name in bones):
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

    def build_chain(self, start_bone_name, attach_bone_name, shrinkwrap_name=None, chain_length=90, bone_size=0.1, bone_dof=(2, 2, 2), density=1100):
        if start_bone_name not in self.armature.pose.bones:
            print("Error - start bone not found in chain - %s" % start_bone_name)
            return False

        phys_objects = []
        phys_objects.append(PhysObject(self.armature, self.armature.pose.bones[start_bone_name], None, shrinkwrap_name))

        def build_recurse(bone, phys_objects, counter, max_depth):
            for child in bone.children:
                phys_objects.append(PhysObject(self.armature, child, [bone_size, bone_size], shrinkwrap_name, density))
                if counter < max_depth:
                    build_recurse(child, phys_objects, counter+1, max_depth)
        build_recurse(self.armature.pose.bones[start_bone_name], phys_objects, 0, chain_length)

        constraints = []
        #BUILD CONSTRAINTS
        cdx, cdy, cdz = bone_dof
        constraint_parameters = [-cdx,cdx,-cdy,cdy,-cdz,cdz,True]

        for i in range(len(phys_objects)):   
            if i+1 < len(phys_objects):
                constraints.append( Constraint(phys_objects[i], phys_objects[i+1], constraint_parameters) )

        attach_phys = None
        for phys in self.phys_objects:
            if attach_bone_name == phys.phys_object.name.split('~')[2]:
                attach_phys = phys
        if attach_phys:
            constraints.append( Constraint(attach_phys, phys_objects[0], constraint_parameters) )
        else:
            print("Unable to find physobject ", attach_bone_name)

        self.phys_objects += phys_objects
        self.constraints += constraints
        self.set_rest_matrix()

    def hide_constraints(self, hidden=True):
        for con in self.constraints:
            con.bone_constraint.hide = hidden

    def hide_phys_objects(self, hidden=True):
        for con in self.phys_objects:
            con.phys_object.hide = hidden

    def draw_phys_objects(self, draw_type='BOUNDS'):
        for con in self.phys_objects:
            con.phys_object.draw_type = draw_type

    def apply_stiffness_map(self, stiffness):
        for m in stiffness:
            self.apply_stiffness(m)

    def apply_stiffness(self, m):
        iterations = None
        if len(m) > 2:
            iterations = m[2]
        self.set_stiffness(m[0], m[1], iterations)

    def save_pose_matrix_state(self):
        for bone in self.armature.pose.bones:
            mat = Matrix()
            mat[0][0:4] = (bone.x_axis[0], bone.y_axis[0], bone.z_axis[0], bone.head[0])
            mat[1][0:4] = (bone.x_axis[1], bone.y_axis[1], bone.z_axis[1], bone.head[1])
            mat[2][0:4] = (bone.x_axis[2], bone.y_axis[2], bone.z_axis[2], bone.head[2])
            self.saved_state[bone.name] = mat
            bone.location = (0, 0, 0)
            bone.rotation_quaternion = (1, 0, 0, 0)
            bone.scale = (1, 1, 1)
        bpy.context.scene.update()

    def move_physobj_collision_layer(self, ob_name_list, layer_number=1):
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        for phys_obj in self.phys_objects:
            mybone = phys_obj.get_name().split('~')[-1]
            if mybone in ob_name_list:
                number_of_layers = len(phys_obj.phys_object.rigid_body.collision_groups)
                phys_obj.phys_object.rigid_body.collision_groups = [i in {layer_number} for i in range(number_of_layers)]

    def create_rig(self, shrinkwrap_object_name=None, damping=1.0):
        self.save_pose_matrix_state()
        bones_to_create = self.template['bones']
        constraints = self.template['constraints']

        self.shrinkwrap_object_name = shrinkwrap_object_name
        phys_map = {}
        for bone_name, params in bones_to_create.items():
            po = PhysObject(self.armature, self.armature.pose.bones[bone_name], params, shrinkwrap_object_name)
            self.phys_objects.append(po)
            phys_map[bone_name] = po

        for constraint in constraints:
            bone_name1, bone_name2, parameters = constraint
            con = Constraint(phys_map[bone_name1], phys_map[bone_name2], parameters)
            self.constraints.append(con)

        if custom_template is not None and self.armature.name in custom_template.templates:
            imp.reload(custom_template)
            custom_template.templates[self.armature.name](self)
        else:
            #No custom template exists.
            print("No custom template found", custom_template, self.armature.name)

        if 'collision_group_ext' in self.template:
            collision_group_ext = self.template['collision_group_ext']
            self.move_physobj_collision_layer(collision_group_ext, layer_number=1)

        # MUTE PRE-EXISTING CONSTRAINTS
        for phys in self.phys_objects:
            name = phys.phys_object.name
            for con in phys.pose_bone.constraints:
                if con.name != 'PHYSPOSE-COPY-LOCATION' and con.name != 'PHYSPOSE-COPY-ROTATION':
                    con.mute = True

        self.set_rest_matrix()
        if 'minimize_twist' in self.template:
            self.apply_stiffness_map(self.template['minimize_twist'])

        self.hide_constraints()
        self.draw_phys_objects('BOUNDS')
        if 'stiffness_map' in self.template:
            self.apply_stiffness_map(self.template['stiffness_map'])
        rigs[self.armature.name] = self
        self.armature.phys_pose_data = rigs[self.armature.name].serialize()
        bpy.context.scene.frame_set(0)
        return True, "Success"

    def apply_saved_state(self, set_kinematic=False):
        for phys in self.phys_objects:
            bone_name = phys.phys_object.name.split('~')[-1]
            if bone_name in self.saved_state:
                phys.phys_object.rigid_body.kinematic = set_kinematic
                phys.phys_object.matrix_world = self.saved_state[bone_name].copy()

    def delete_rig(self):
        do_hide = False
        if self.armature.hide:
            self.armature.hide = False
            do_hide = True
        bpy.ops.object.mode_set(mode='OBJECT')
        armature = self.armature
        if not armature:
            return False, "Object is not supported type (Armature or Mesh)"

        bpy.ops.object.select_all(action='DESELECT')
        physpose_data = json.loads(armature.phys_pose_data)
        bpy.context.scene.layers = armature.layers

        #armature.pose.bones["HandIk.R"].matrix = armature.pose.bones["Hand.R"].matrix.copy()
        #armature.pose.bones["HandIk.L"].matrix = armature.pose.bones["Hand.L"].matrix.copy()
        #armature.pose.bones["FootIk.R"].matrix = armature.pose.bones["Foot.R"].matrix.copy()
        #armature.pose.bones["FootIk.L"].matrix = armature.pose.bones["Foot.L"].matrix.copy()

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
        for constraint in self.constraints:
            name = constraint.bone_constraint.name
            bpy.data.objects[name].hide = False
            bpy.data.objects[name].select = True

        bpy.ops.object.delete()
        if do_hide:
            self.armature.hide = True
        return True, "Success"

    def restore_physpose_rig(self):
        do_hide = False
        if self.armature.hide:
            self.armature.hide = False
            do_hide = True
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        #oldlayers = bpy.context.scene.layers
        #bpy.context.scene.layers = self.armature.layers
        for phys_obj in self.phys_objects:
            bpy.data.objects[phys_obj.get_name()].matrix_world = self.rest_phys_matrix[phys_obj.get_name()]
        #bpy.context.scene.layers = oldlayers
        if do_hide:
            self.armature.hide = True
        return True, "Success"


def print_total_weight(poserig):
    total = 0
    for phys in poserig.phys_objects:
        total += phys.phys_object.rigid_body.mass
    print("Total weight: ", total)

rigs = {}


def get_armature():
    if bpy.context.scene.objects.active.name[0:4] == 'PHYS':
        phystype, armature, bone = bpy.context.scene.objects.active.name.split('~')
        return bpy.data.objects[armature]
    elif type(bpy.context.scene.objects.active.data) == bpy.types.Mesh:
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


def get_poserig():
    ob = get_armature()
    template = templates[bpy.context.scene['phys_pose_template']][1]
    poserig = PhysPoseRig(ob, template)
    poserig.deserialize(ob.phys_pose_data)
    return poserig


def create_rig():
    bpy.ops.object.mode_set(mode='OBJECT')
    shrink_wrap_name = None
    ob = get_armature()
    ob.phys_pose_data = ""
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
    template = templates[bpy.context.scene['phys_pose_template']][1]
    imp.reload(template)
    poserig = PhysPoseRig(bpy.data.objects[armature_name], template.template)
    poserig.clear_constraints()
    poserig.create_rig(shrink_wrap_name)
    return True, "Success"


operators_list = {}


def create_operator_class(class_name, idname, label, options, execute_function):
    def handle_func_call(self, context):
        success, message = self.do_work()
        if not success:
            self.report({'INFO'}, message)
        return {'FINISHED'}

    type_map = {
        'bl_idname': idname,
        'bl_label': label,
        'bl_options': options,
        'do_work': execute_function,
        'execute': handle_func_call
    }

    operator_class = type(class_name, (bpy.types.Operator,), type_map)
    operators_list[class_name] = operator_class
    return operator_class


def generate_physpose_rig(self):
    return create_rig()


def delete_physpose_rig(self):
    poserig = get_poserig()
    poserig.delete_rig()
    return True, "Success"


def pin_phys_object(self):
    bpy.ops.object.mode_set(mode='OBJECT')
    for o in bpy.context.selected_objects:
        if hasattr(o, 'rigid_body') and o.rigid_body is not None:
            matrix = o.matrix_world
            o.rigid_body.kinematic = True
            o.matrix_world = matrix
    return True, "Success"


def unpin_phys_object(self):
    bpy.ops.object.mode_set(mode='OBJECT')
    for o in bpy.context.selected_objects:
        if hasattr(o, 'rigid_body') and o.rigid_body is not None:
            o.rigid_body.kinematic = False
    return True, "Success"


def reset_physpose_rig(self):
    poserig = get_poserig()
    return poserig.restore_physpose_rig()


def set_damping_physpose_rig(self):
    poserig = get_poserig()
    armature = get_armature()
    damping_setting = armature.phys_pose_damping
    for phys_object in poserig.phys_objects:
        phys_object.phys_object.rigid_body.angular_damping = damping_setting
        phys_object.phys_object.rigid_body.linear_damping = damping_setting
    return True, "Success"


def generate_set_stiffness_function(index, attrname):
    def set_stiffness_physpose_rig(self):
        poserig = get_poserig()
        armature = get_armature()
        damping_setting = getattr(armature, attrname)
        new_stiffness = list(poserig.template.template['stiffness_map'][index])
        new_stiffness[0] = damping_setting
        poserig.apply_stiffness(new_stiffness)
        return True, "Success"
    return set_stiffness_physpose_rig

def draw_bounds_physpose_rig(self):
    poserig = get_poserig()
    poserig.draw_phys_objects('BOUNDS')
    return True, "Success"


def draw_full_physpose_rig(self):
    poserig = get_poserig()
    poserig.draw_phys_objects('TEXTURED')
    return True, "Success"


def collide_mesh(self):
    poserig = get_poserig()
    for phys_object in poserig.phys_objects:
        phys_object.phys_object.rigid_body.collision_shape = 'MESH'
    return True, "Success"


def collide_hull(self):
    poserig = get_poserig()
    for phys_object in poserig.phys_objects:
        phys_object.phys_object.rigid_body.collision_shape = 'CONVEX_HULL'
    return True, "Success"


def apply_saved_state(self):
    poserig = get_poserig()
    poserig.apply_saved_state()
    return True, "Success"


def point_fingers(self):
    poserig = get_poserig()
    scalars = [-0.5, -0.5, -0.5]
    bones = [
        "Index1.R",
        "Mid1.R",
        "Ring1.R",
        "Pinky1.R",
        "Thumb1.R",
        "Index2.R",
        "Mid2.R",
        "Ring2.R",
        "Pinky2.R",
        "Thumb2.R",
    ]
    poserig.set_rotations(scalars, bones, 30)
    scalars = [0.5, 0.5, 0.5]
    bones = [
        "Index1.L",
        "Mid1.L",
        "Ring1.L",
        "Pinky1.L",
        "Thumb1.L",
        "Index2.L",
        "Mid2.L",
        "Ring2.L",
        "Pinky2.L",
        "Thumb2.L",
    ]
    poserig.set_rotations(scalars, bones, 30)
    return True, "Success"


GeneratePhysPoseRig = create_operator_class('GeneratePhysPoseRig', "object.generate_physpose_rig", "Generate PhysPose Rig", {'REGISTER', 'UNDO'}, generate_physpose_rig)
DeletePhysPoseRig = create_operator_class('DeletePhysPoseRig', "object.delete_physpose_rig", "Delete PhysPose Rig", {'REGISTER', 'UNDO'}, delete_physpose_rig)
PinPhysObject = create_operator_class('PinPhysObject', "object.pin_phys_object", "PinPhysObject", {'REGISTER', 'UNDO'}, pin_phys_object)
UnpinPhysObject = create_operator_class('UnpinPhysObject', "object.unpin_phys_object", "UnpinPhysObject", {'REGISTER', 'UNDO'}, unpin_phys_object)
ResetPhysPoseRig = create_operator_class('ResetPhysPoseRig', "object.reset_physpose_rig", "ResetPhysPoseRig", {'REGISTER', 'UNDO'}, reset_physpose_rig)
SetDampingPhysPoseRig = create_operator_class('SetDampingPhysPoseRig', "object.set_damping_physpose_rig", "SetDampingPhysPoseRig", {'REGISTER', 'UNDO'}, set_damping_physpose_rig)

SetSpineStiffnessPhysPoseRig = create_operator_class('SetSpineStiffnessPhysPoseRig', "object.set_spine_stiffness_physpose_rig", "SetSpineStiffnessPhysPoseRig", {'REGISTER', 'UNDO'}, generate_set_stiffness_function(0, "phys_pose_spine_stiffness"))
SetNeckStiffnessPhysPoseRig = create_operator_class('SetNeckStiffnessPhysPoseRig', "object.set_neck_stiffness_physpose_rig", "SetNeckStiffnessPhysPoseRig", {'REGISTER', 'UNDO'}, generate_set_stiffness_function(2, "phys_pose_neck_stiffness"))
SetShoulderStiffnessPhysPoseRig = create_operator_class('SetShoulderStiffnessPhysPoseRig', "object.set_shoulder_stiffness_physpose_rig", "SetShoulderStiffnessPhysPoseRig", {'REGISTER', 'UNDO'}, generate_set_stiffness_function(3, "phys_pose_shoulder_stiffness"))

DrawBoundsPhysPoseRig = create_operator_class('DrawBoundsPhysPoseRig', "object.draw_bounds_physpose_rig", "DrawBoundsPhysPoseRig", {'REGISTER', 'UNDO'}, draw_bounds_physpose_rig)
DrawFullPhysPoseRig = create_operator_class('DrawFullPhysPoseRig', "object.draw_full_physpose_rig", "DrawFullPhysPoseRig", {'REGISTER', 'UNDO'}, draw_full_physpose_rig)
SetCollisionMesh = create_operator_class('SetCollisionMesh', "object.collide_mesh", "SetCollisionMesh", {'REGISTER', 'UNDO'}, collide_mesh)
SetCollisionHull = create_operator_class('SetCollisionHull', "object.collide_hull", "SetCollisionHull", {'REGISTER', 'UNDO'}, collide_hull)
ApplySavedState = create_operator_class('ApplySavedState', "object.apply_saved_state", "ApplySavedState", {'REGISTER', 'UNDO'}, apply_saved_state)
#PointFingers = create_operator_class('PointFingers', "object.point_fingers", "PointFingers", {'REGISTER', 'UNDO'}, point_fingers)


class PhysPosePanel(bpy.types.Panel):
    bl_label = "PhysPose Tools"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    myprop = None

    def draw(self, context):
        ob = context.object
        layout = self.layout

        layout.prop(bpy.context.scene, 'phys_pose_template')
        layout.operator("object.generate_physpose_rig", text='Generate PhysPose Rig')
        armature = get_armature()
        if armature is not None:
            layout.operator("object.delete_physpose_rig", text='Delete PhysPose Rig')
        layout.label("Pinning")
        layout.operator("object.pin_phys_object", text='Pin Object(s)')
        layout.operator("object.unpin_phys_object", text='Unpin Object(s)')
        if armature is not None:
            layout.label("Poses")
            #layout.prop(bpy.context.scene, 'pose_lib_selector')
            #layout.operator("object.apply_saved_state", text='Apply Pose')
            layout.operator("object.reset_physpose_rig", text='Reset to Base Pose')
            #layout.operator("object.point_fingers", text='Clench Fingers')
            #layout.label("Keyframe Tools")
            #layout.operator("object.set_rotations_physpose_rig", text='Apply PhysPose to Rig')
            #layout.operator("object.unmute_constraints_physpose_rig", text='Unmute Constraints on PhysPose Rig')
            layout.label("Set Damping")
            row = layout.row()
            row.prop(armature, 'phys_pose_damping', slider=True)
            row.operator("object.set_damping_physpose_rig", text='Set Damping')
            row2 = layout.row()
            row2.prop(armature, 'phys_pose_spine_stiffness', slider=True)
            row2.operator("object.set_spine_stiffness_physpose_rig", text='Set Spine Stiffness')
            row2 = layout.row()
            row2.prop(armature, 'phys_pose_neck_stiffness', slider=True)
            row2.operator("object.set_neck_stiffness_physpose_rig", text='Set Neck Stiffness')
            row2 = layout.row()
            row2.prop(armature, 'phys_pose_shoulder_stiffness', slider=True)
            row2.operator("object.set_shoulder_stiffness_physpose_rig", text='Set Shoulder Stiffness')

        row = layout.row()
        row.operator("object.draw_bounds_physpose_rig", text='Draw As Bounds')
        row.operator("object.draw_full_physpose_rig", text='Draw As Shapes')
        row2 = layout.row()
        row2.operator("object.collide_mesh", text='Collide as Mesh')
        row2.operator("object.collide_hull", text='Collide as Convex Hull')


def register():
    for operator_class_name in operators_list:
        operator_class = operators_list[operator_class_name]
        bpy.utils.register_class(operator_class)

    bpy.utils.register_class(PhysPosePanel)
    bpy.types.Object.phys_pose_data = bpy.props.StringProperty(name = "PhysPoseData")
    bpy.types.Object.phys_pose_damping = bpy.props.FloatProperty(name = "PhysPoseDamping", default=1.0, min=0.0, max=1.0)
    bpy.types.Object.phys_pose_spine_stiffness = bpy.props.FloatProperty(name = "PhysPoseSpineStiffness", default=0.05, min=0.0, max=1.0)
    bpy.types.Object.phys_pose_neck_stiffness = bpy.props.FloatProperty(name = "PhysPoseNeckStiffness", default=0.2, min=0.0, max=1.0)
    bpy.types.Object.phys_pose_shoulder_stiffness = bpy.props.FloatProperty(name = "PhysPoseShoulderStiffness", default=0.25, min=0.0, max=1.0)
    bpy.types.Scene.phys_pose_template = bpy.props.EnumProperty(
        items=[(x[0], x[0], x[0]) for x in templates],
        name="Template"
    )
    def get_poselib_names(self, context):
        armature = get_armature()
        if armature.pose_library is None:
            return [('No Poselib', 'No Poselib', 'No Poselib')]
        return [(x.name, x.name, x.name) for x in armature.pose_library.pose_markers]
    bpy.types.Scene.pose_lib_selector = bpy.props.EnumProperty(items=get_poselib_names, name="Pose")


def unregister():
    for operator_class_name in operators_list:
        operator_class = operators_list[operator_class_name]
        bpy.utils.unregister_class(operator_class)

    bpy.utils.unregister_class(PhysPosePanel)
    del bpy.types.Object.phys_pose_data
    del bpy.types.Object.phys_pose_damping
    del bpy.types.Object.phys_pose_spine_stiffness
    del bpy.types.Object.phys_pose_neck_stiffness
    del bpy.types.Object.phys_pose_shoulder_stiffness
    # del bpy.types.Object.pose_lib_selector


if __name__ == "__main__":
    register()
