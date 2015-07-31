#====================== BEGIN GPL LICENSE BLOCK ======================
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#======================= END GPL LICENSE BLOCK ========================

# <pep8 compliant>

import math
from mathutils import Vector, Matrix
import sys
import os
import bpy
from .utils import *

ob = None
amt = None
WGT_PREFIX = "WGT-"  # Prefix for widget objects


layers_map = {
"Genesis3Female":0,
"hip":1,
"pelvis":1,
"lThighBend":2,
"lThighTwist":2,
"lShin":2,
"lFoot":16,
"lMetatarsals":2,
"lToe":2,
"lSmallToe4":3,
"lSmallToe4_2":3,
"lSmallToe3":3,
"lSmallToe3_2":3,
"lSmallToe2":3,
"lSmallToe2_2":3,
"lSmallToe1":3,
"lSmallToe1_2":3,
"lBigToe":3,
"lBigToe_2":3,
"lHeel":2,
"rThighBend":4,
"rThighTwist":4,
"rShin":4,
"rFoot":16,
"rMetatarsals":4,
"rToe":4,
"rSmallToe4":5,
"rSmallToe4_2":5,
"rSmallToe3":5,
"rSmallToe3_2":5,
"rSmallToe2":5,
"rSmallToe2_2":5,
"rSmallToe1":5,
"rSmallToe1_2":5,
"rBigToe":5,
"rBigToe_2":5,
"rHeel":4,
"abdomenLower":1,
"abdomenUpper":1,
"chestLower":1,
"chestUpper":1,
"lCollar":1,
"lShldrBend":6,
"lShldrTwist":6,
"lForearmBend":6,
"lForearmTwist":6,
"lHand":16,
"lThumb1":7,
"lThumb2":7,
"lThumb3":7,
"lCarpal1":7,
"lIndex1":7,
"lIndex2":7,
"lIndex3":7,
"lCarpal2":7,
"lMid1":7,
"lMid2":7,
"lMid3":7,
"lCarpal3":7,
"lRing1":7,
"lRing2":7,
"lRing3":7,
"lCarpal4":7,
"lPinky1":7,
"lPinky2":7,
"lPinky3":7,
"rCollar":1,
"rShldrBend":8,
"rShldrTwist":8,
"rForearmBend":8,
"rForearmTwist":8,
"rHand":16,
"rThumb1":9,
"rThumb2":9,
"rThumb3":9,
"rCarpal1":9,
"rIndex1":9,
"rIndex2":9,
"rIndex3":9,
"rCarpal2":9,
"rMid1":9,
"rMid2":9,
"rMid3":9,
"rCarpal3":9,
"rRing1":9,
"rRing2":9,
"rRing3":9,
"rCarpal4":9,
"rPinky1":9,
"rPinky2":9,
"rPinky3":9,
"neckLower":1,
"neckUpper":1,
"head":1,
"upperTeeth":10,
"lowerJaw":10,
"lowerTeeth":16,
"tongue01":10,
"tongue02":10,
"tongue03":10,
"tongue04":10,
"lowerFaceRig":10,
"lNasolabialLower":10,
"rNasolabialLower":10,
"lNasolabialMouthCorner":10,
"rNasolabialMouthCorner":10,
"lLipCorner":10,
"lLipLowerOuter":10,
"lLipLowerInner":10,
"LipLowerMiddle":10,
"rLipLowerInner":10,
"rLipLowerOuter":10,
"rLipCorner":10,
"LipBelow":10,
"Chin":10,
"lCheekLower":10,
"rCheekLower":10,
"BelowJaw":10,
"lJawClench":10,
"rJawClench":10,
"upperFaceRig":10,
"rBrowInner":10,
"rBrowMid":10,
"rBrowOuter":10,
"lBrowInner":10,
"lBrowMid":10,
"lBrowOuter":10,
"CenterBrow":10,
"MidNoseBridge":10,
"lEyelidInner":16,
"lEyelidUpperInner":16,
"lEyelidUpper":16,
"lEyelidUpperOuter":16,
"lEyelidOuter":16,
"lEyelidLowerOuter":16,
"lEyelidLower":16,
"lEyelidLowerInner":16,
"rEyelidInner":16,
"rEyelidUpperInner":16,
"rEyelidUpper":16,
"rEyelidUpperOuter":16,
"rEyelidOuter":16,
"rEyelidLowerOuter":16,
"rEyelidLower":16,
"rEyelidLowerInner":16,
"lSquintInner":10,
"lSquintOuter":10,
"rSquintInner":10,
"rSquintOuter":10,
"lCheekUpper":10,
"rCheekUpper":10,
"Nose":10,
"lNostril":10,
"rNostril":10,
"lLipBelowNose":10,
"rLipBelowNose":10,
"lLipUpperOuter":10,
"lLipUpperInner":10,
"LipUpperMiddle":10,
"rLipUpperInner":10,
"rLipUpperOuter":10,
"lLipNasolabialCrease":10,
"rLipNasolabialCrease":10,
"lNasolabialUpper":10,
"rNasolabialUpper":10,
"lNasolabialMiddle":10,
"rNasolabialMiddle":10,
"lEye":10,
"rEye":10,
"lEar":10,
"rEar":10,
"lPectoral":1,
"rPectoral":1,
"EyeGaze":10,
"lEyeGaze":10,
"rEyeGaze":10,
"rEyelidUpperGaze":10,
"lEyelidUpperGaze":10,
"rEyelidLowerGaze":10,
"lEyelidLowerGaze":10,
"lFootIk":2,
"lLegTargetIk":2,
"rFootIk":4,
"rLegTargetIk":4,
"lHandIk":6,
"lArmTargetIk":6,
"rHandIk":8,
"rArmTargetIk":8
}

def do_setup():
	print("bone hierarchy")
	def bone_recurse(depth, bone):
		print(' ' * depth + bone.name)
		for b in bone.children:
			bone_recurse(depth+1, b)
	bone_recurse(1, ob.pose.bones["Genesis3Female"])
	bpy.ops.object.mode_set(mode='EDIT')
	root = amt.edit_bones["Genesis3Female"]
	root.tail = (0,1,0)
	orig_layers =  (False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False)
	for bone in ob.pose.bones:
		amt.edit_bones[bone.name].layers = orig_layers

def finalize_layers():
	bpy.ops.object.mode_set(mode='EDIT')
	global layers_map
	layers = []
	for i in range(32):
		l =  [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
		l[i] = True
		layers.append(tuple(l))
	for k,v in layers_map.items():
		if k in amt.edit_bones:
			amt.edit_bones[k].layers = layers[v]
		else:
			amt.edit_bones[k].layers = layers[17] #unused layers

def create_gaze_bones():
	bpy.ops.object.mode_set(mode='EDIT')
	lGaze = amt.edit_bones.new("lEyeGaze")
	rGaze = amt.edit_bones.new("rEyeGaze")
	gaze = amt.edit_bones.new("EyeGaze")

	lGaze.parent = amt.edit_bones["EyeGaze"]
	lGaze.head = (amt.edit_bones["lEye"].head[0], -0.4, amt.edit_bones["lEye"].head[2])
	lGaze.tail = (amt.edit_bones["lEye"].head[0], -0.45, amt.edit_bones["lEye"].head[2])

	rGaze.parent = amt.edit_bones["EyeGaze"]
	rGaze.head = (amt.edit_bones["rEye"].head[0], -0.4, amt.edit_bones["rEye"].head[2])
	rGaze.tail = (amt.edit_bones["rEye"].head[0], -0.45, amt.edit_bones["rEye"].head[2])

	gaze.parent = amt.edit_bones["head"]
	gaze.head = (0, -0.4, amt.edit_bones["rEye"].head[2])
	gaze.tail = (0, -0.48, amt.edit_bones["rEye"].head[2])

	rEyelidUpperGaze = amt.edit_bones.new("rEyelidUpperGaze")
	rEyelidUpperGaze.head = (amt.edit_bones["rEyelidUpper"].head[0], -0.08, amt.edit_bones["rEyelidUpper"].head[2] + 0.0055)
	rEyelidUpperGaze.tail = (amt.edit_bones["rEyelidUpper"].head[0], -0.085, amt.edit_bones["rEyelidUpper"].head[2] + 0.0055)
	rEyelidUpperGaze.roll = 3.14159
	rEyelidUpperGaze.parent = amt.edit_bones["head"]

	lEyelidUpperGaze = amt.edit_bones.new("lEyelidUpperGaze")
	lEyelidUpperGaze.head = (amt.edit_bones["lEyelidUpper"].head[0], -0.08, amt.edit_bones["lEyelidUpper"].head[2] + 0.0055)
	lEyelidUpperGaze.tail = (amt.edit_bones["lEyelidUpper"].head[0], -0.085, amt.edit_bones["lEyelidUpper"].head[2] + 0.0055)
	lEyelidUpperGaze.roll = 3.14159
	lEyelidUpperGaze.parent = amt.edit_bones["head"]

	rEyelidLowerGaze = amt.edit_bones.new("rEyelidLowerGaze")
	rEyelidLowerGaze.head = (amt.edit_bones["rEyelidLower"].head[0], -0.08, amt.edit_bones["rEyelidLower"].head[2] - 0.007)
	rEyelidLowerGaze.tail = (amt.edit_bones["rEyelidLower"].head[0], -0.085, amt.edit_bones["rEyelidLower"].head[2] - 0.007)
	rEyelidLowerGaze.roll = 3.14159
	rEyelidLowerGaze.parent = amt.edit_bones["head"]

	lEyelidLowerGaze = amt.edit_bones.new("lEyelidLowerGaze")
	lEyelidLowerGaze.head = (amt.edit_bones["lEyelidLower"].head[0], -0.08, amt.edit_bones["lEyelidLower"].head[2] - 0.007)
	lEyelidLowerGaze.tail = (amt.edit_bones["lEyelidLower"].head[0], -0.085, amt.edit_bones["lEyelidLower"].head[2] - 0.007)
	lEyelidLowerGaze.roll = 3.14159
	lEyelidLowerGaze.parent = amt.edit_bones["head"]


	"""
		SET UP CONSTRAINTS
	"""
	def setup_track_to(bone_name, track_bone_name):
		rEye = ob.pose.bones[bone_name]
		const = rEye.constraints.new('TRACK_TO')
		const.target = bpy.data.objects["GenesisRig"]
		const.subtarget = track_bone_name
		const.track_axis = "TRACK_Z"
		const.up_axis = "UP_Y"	

	def setup_transform(bone_name, track_bone_name):
		dest_bone = ob.pose.bones[bone_name]
		const = dest_bone.constraints.new('TRANSFORM')
		const.target = bpy.data.objects["GenesisRig"]
		const.subtarget = track_bone_name
		const.from_min_z = -0.01
		const.from_max_z = 0.01
		const.map_to_x_from = 'Z'	
		const.map_to = 'ROTATION'
		const.to_min_x_rot = -1.39626
		const.to_max_x_rot = 1.39626
		const.target_space = 'LOCAL'
		const.owner_space = 'LOCAL'

	def do_lr_const(function_name, bone_name, track_bone_name):
		function_name('l'+bone_name, 'l'+track_bone_name)
		function_name('r'+bone_name, 'r'+track_bone_name)

	bpy.ops.object.mode_set(mode='POSE')

	do_lr_const(setup_track_to, "Eye", "EyeGaze")

	do_lr_const(setup_transform, "EyelidUpper", "EyelidUpperGaze")
	do_lr_const(setup_transform, "EyelidUpperOuter", "EyelidUpperGaze")
	do_lr_const(setup_transform, "EyelidUpperInner", "EyelidUpperGaze")
	do_lr_const(setup_transform, "EyelidLower", "EyelidLowerGaze")
	do_lr_const(setup_transform, "EyelidLowerOuter", "EyelidLowerGaze")
	do_lr_const(setup_transform, "EyelidLowerInner", "EyelidLowerGaze")

	bpy.ops.object.mode_set(mode='EDIT')


def create_leg_ik():
	leg_layers =  (False, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False)
	bpy.ops.object.mode_set(mode='EDIT')
	rootBoneName = "Genesis3Female"

	lFootIk = amt.edit_bones.new("lFootIk")
	rFootIk = amt.edit_bones.new("rFootIk")
	rLegTargetIk = amt.edit_bones.new("rLegTargetIk")	
	lLegTargetIk = amt.edit_bones.new("lLegTargetIk")
	lFootIk.layers = leg_layers
	rFootIk.layers = leg_layers
	rLegTargetIk.layers = leg_layers
	lLegTargetIk.layers = leg_layers

	lFootIk.parent = amt.edit_bones[rootBoneName]
	lFootIk.head = amt.edit_bones["lFoot"].head
	lFootIk.tail = scale_bone_from_origin("lFoot", 0.1)

	rFootIk.parent = amt.edit_bones[rootBoneName]
	rFootIk.head = amt.edit_bones["rFoot"].head
	rFootIk.tail = scale_bone_from_origin("rFoot", 0.1)

	rLegTargetIk.parent = amt.edit_bones[rootBoneName]
	rLegTargetIk.head = (amt.edit_bones["rShin"].head[0], -0.5, amt.edit_bones["rShin"].head[2])
	rLegTargetIk.tail = (amt.edit_bones["rShin"].head[0], -0.7, amt.edit_bones["rShin"].head[2])

	lLegTargetIk.parent = amt.edit_bones[rootBoneName]
	lLegTargetIk.head = (amt.edit_bones["lShin"].head[0], -0.5, amt.edit_bones["lShin"].head[2])
	lLegTargetIk.tail = (amt.edit_bones["lShin"].head[0], -0.7, amt.edit_bones["lShin"].head[2])

	"""
		SET UP CONSTRAINTS
	"""
	bpy.ops.object.mode_set(mode='POSE')

	rShin = ob.pose.bones["rShin"]
	const = rShin.constraints.new('IK')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "rFootIk"
	const.pole_target = bpy.data.objects["GenesisRig"]
	const.pole_subtarget = "rLegTargetIk"
	const.chain_count = 3
	const.pole_angle = 2.28638
	ob.pose.bones["rShin"].lock_ik_y = True
	ob.pose.bones["rShin"].lock_ik_x = True
	ob.pose.bones["rThighTwist"].lock_ik_x = True
	ob.pose.bones["rThighTwist"].lock_ik_z = True

	lShin = ob.pose.bones["lShin"]
	const = lShin.constraints.new('IK')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "lFootIk"
	const.pole_target = bpy.data.objects["GenesisRig"]
	const.pole_subtarget = "lLegTargetIk"
	const.chain_count = 3
	const.pole_angle = 0.855208
	ob.pose.bones["lShin"].lock_ik_y = True
	ob.pose.bones["lShin"].lock_ik_x = True
	ob.pose.bones["lThighTwist"].lock_ik_x = True
	ob.pose.bones["lThighTwist"].lock_ik_z = True

	rFoot = ob.pose.bones["rFoot"]
	const = rFoot.constraints.new('COPY_ROTATION')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "rFootIk"

	lFoot = ob.pose.bones["lFoot"]
	const = lFoot.constraints.new('COPY_ROTATION')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "lFootIk"

	bpy.ops.object.mode_set(mode='EDIT')

def scale_bone_from_origin(bone_name, length):
	"""
		returns the bone tail Vector(x,y,z)
	"""
	direction = Vector(amt.edit_bones[bone_name].tail) - Vector(amt.edit_bones[bone_name].head)
	direction.normalize()
	direction *= length
	return Vector(amt.edit_bones[bone_name].head) + direction


def create_arm_ik():
	arm_layers =  (False, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False)
	bpy.ops.object.mode_set(mode='EDIT')
	rootBoneName = "Genesis3Female"

	lHandIk = amt.edit_bones.new("lHandIk")
	rHandIk = amt.edit_bones.new("rHandIk")
	rArmTargetIk = amt.edit_bones.new("rArmTargetIk")
	lArmTargetIk = amt.edit_bones.new("lArmTargetIk")
	lHandIk.layers = arm_layers
	rHandIk.layers = arm_layers
	rArmTargetIk.layers = arm_layers
	lArmTargetIk.layers = arm_layers

	lHandIk.parent = amt.edit_bones[rootBoneName]
	lHandIk.head = amt.edit_bones["lHand"].head
	lHandIk.tail = scale_bone_from_origin("lHand", 0.1)

	rHandIk.parent = amt.edit_bones[rootBoneName]
	rHandIk.head = amt.edit_bones["rHand"].head
	rHandIk.tail = scale_bone_from_origin("rHand", 0.1)

	rArmTargetIk.parent = amt.edit_bones[rootBoneName]
	rArmTargetIk.head = (amt.edit_bones["rForearmBend"].head[0], 0.5, amt.edit_bones["rForearmBend"].head[2])
	rArmTargetIk.tail = (amt.edit_bones["rForearmBend"].head[0], 0.65, amt.edit_bones["rForearmBend"].head[2])

	lArmTargetIk.parent = amt.edit_bones[rootBoneName]
	lArmTargetIk.head = (amt.edit_bones["lForearmBend"].head[0], 0.5, amt.edit_bones["lForearmBend"].head[2])
	lArmTargetIk.tail = (amt.edit_bones["lForearmBend"].head[0], 0.65, amt.edit_bones["lForearmBend"].head[2])

	"""
		SET UP CONSTRAINTS
	"""
	bpy.ops.object.mode_set(mode='POSE')

	rForearmTwist = ob.pose.bones["rForearmTwist"]
	const = rForearmTwist.constraints.new('IK')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "rHandIk"
	const.pole_target = bpy.data.objects["GenesisRig"]
	const.pole_subtarget = "rArmTargetIk"
	const.chain_count = 4
	const.pole_angle = -1.6057
	ob.pose.bones["rForearmTwist"].lock_ik_x = True
	ob.pose.bones["rForearmTwist"].lock_ik_z = True
	ob.pose.bones["rShldrTwist"].lock_ik_x = True
	ob.pose.bones["rShldrTwist"].lock_ik_z = True

	lForearmTwist = ob.pose.bones["lForearmTwist"]
	const = lForearmTwist.constraints.new('IK')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "lHandIk"
	const.pole_target = bpy.data.objects["GenesisRig"]
	const.pole_subtarget = "lArmTargetIk"
	const.chain_count = 4
	const.pole_angle = -1.51844
	ob.pose.bones["lForearmTwist"].lock_ik_x = True
	ob.pose.bones["lForearmTwist"].lock_ik_z = True
	ob.pose.bones["lShldrTwist"].lock_ik_x = True
	ob.pose.bones["lShldrTwist"].lock_ik_z = True

	rHand = ob.pose.bones["rHand"]
	const = rHand.constraints.new('COPY_ROTATION')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "rHandIk"

	lHand = ob.pose.bones["lHand"]
	const = lHand.constraints.new('COPY_ROTATION')
	const.target = bpy.data.objects["GenesisRig"]
	const.subtarget = "lHandIk"

	bpy.ops.object.mode_set(mode='EDIT')


def create_hand_widget(rig, bone_name):
    ob = create_widget(rig, bone_name)
    if ob != None:
        verts = [(0.7, 1.5, 0.0), (0.7, -0.25, 0.0), (-0.7, -0.25, 0.0), (-0.7, 1.5, 0.0), (0.7, 0.723, 0.0), (-0.7, 0.723, 0.0), (0.7, 0.0, 0.0), (-0.7, 0.0, 0.0)]
        edges = [(1, 2), (0, 3), (0, 4), (3, 5), (4, 6), (1, 6), (5, 7), (2, 7)]
        mesh = ob.data
        mesh.from_pydata(verts, edges, [])
        mesh.update()

        mod = ob.modifiers.new("subsurf", 'SUBSURF')
        mod.levels = 2

def create_foot_widget(rig, bone_name):
    ob = create_widget(rig, bone_name)
    if ob != None:
        verts = [(0.7, 1.5, 0.0), (0.7, -0.25, 0.0), (-0.7, -0.25, 0.0), (-0.7, 1.5, 0.0), (0.7, 0.723, 0.0), (-0.7, 0.723, 0.0), (0.7, 0.0, 0.0), (-0.7, 0.0, 0.0)]
        edges = [(1, 2), (0, 3), (0, 4), (3, 5), (4, 6), (1, 6), (5, 7), (2, 7)]
        mesh = ob.data
        mesh.from_pydata(verts, edges, [])
        mesh.update()

        mod = ob.modifiers.new("subsurf", 'SUBSURF')
        mod.levels = 2


def setup_widgets():
	def do_create_widget(widget_function, bone_name):
		widget_function(ob, bone_name)
		ob.pose.bones[bone_name].custom_shape = bpy.data.objects[WGT_PREFIX + bone_name]
		ob.data.bones[bone_name].show_wire = True

	bpy.ops.object.mode_set(mode='OBJECT')

	do_create_widget(create_sphere_widget, "lArmTargetIk")
	do_create_widget(create_sphere_widget, "rArmTargetIk")

	do_create_widget(create_sphere_widget, "lLegTargetIk")
	do_create_widget(create_sphere_widget, "rLegTargetIk")

	do_create_widget(create_hand_widget, "lHandIk")
	do_create_widget(create_hand_widget, "rHandIk")

	do_create_widget(create_foot_widget, "lFootIk")
	do_create_widget(create_foot_widget, "rFootIk")

	do_create_widget(create_circle_widget, "lEyeGaze")
	do_create_widget(create_circle_widget, "rEyeGaze")
	do_create_widget(create_circle_widget, "EyeGaze")

	do_create_widget(create_cube_widget, "rEyelidUpperGaze")
	do_create_widget(create_cube_widget, "lEyelidUpperGaze")

	do_create_widget(create_cube_widget, "rEyelidLowerGaze")
	do_create_widget(create_cube_widget, "lEyelidLowerGaze")

	do_create_widget(create_root_widget, "Genesis3Female")

	#create circle widget
	"""

		SET UP FACE WIDGETS

	"""
	face_layers =  (False, False, False, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False)
	create_sphere_widget(ob, "lowerFaceRig")
	for bone in (ob.pose.bones["upperFaceRig"].children + ob.pose.bones["lowerFaceRig"].children):
		if bone.name[1:7] == "Eyelid":
			continue
		bpy.ops.object.mode_set(mode='EDIT')
		vec = amt.edit_bones[bone.name].head
		amt.edit_bones[bone.name].tail = scale_bone_from_origin(bone.name, 0.005)
		amt.edit_bones[bone.name].layers = face_layers
		bpy.ops.object.mode_set(mode='OBJECT')
		ob.pose.bones[bone.name].custom_shape = bpy.data.objects[WGT_PREFIX + "lowerFaceRig"]
		ob.data.bones[bone.name].show_wire = True
	bpy.ops.object.mode_set(mode='OBJECT')

	def do_scale(bone_name, size, function_name):
		bpy.ops.object.mode_set(mode='EDIT')
		amt.edit_bones[bone_name].tail = scale_bone_from_origin(bone_name, size)
		bpy.ops.object.mode_set(mode='OBJECT')
		do_create_widget(function_name, bone_name)	

	def do_scale_lr(bone_name, size, function_name):
		do_scale("l"+bone_name, size, function_name)
		do_scale("r"+bone_name, size, function_name)

	do_scale("head", 0.18, create_circle_widget)
	do_scale("chestLower", 0.18, create_circle_widget)
	do_scale("chestUpper", 0.18, create_circle_widget)
	do_scale("pelvis", 0.18, create_circle_widget)
	do_scale("hip", 0.35, create_cube_widget)
	do_scale("lowerJaw", 0.02, create_sphere_widget)

	do_scale_lr("Pectoral", 0.02, create_sphere_widget)
	do_scale_lr("Ear", 0.02, create_sphere_widget)
	do_scale_lr("Metatarsals", 0.02, create_sphere_widget)
	do_scale_lr("Heel", 0.02, create_sphere_widget)

	do_create_widget(create_circle_widget, "lEyeGaze")
	do_create_widget(create_circle_widget, "lEyeGaze")
	do_create_widget(create_bone_widget, "tongue01")
	do_create_widget(create_bone_widget, "tongue02")
	do_create_widget(create_bone_widget, "tongue03")
	do_create_widget(create_bone_widget, "tongue04")

	def do_create_widget_lr(function_name, bone_name):
		do_create_widget(function_name, "l"+bone_name)
		do_create_widget(function_name, "r"+bone_name)

	do_create_widget_lr(create_bone_widget, "Collar")
	do_create_widget_lr(create_limb_widget, "ThighBend")
	do_create_widget_lr(create_limb_widget, "ThighTwist")
	do_create_widget_lr(create_limb_widget, "Shin")
	do_create_widget_lr(create_limb_widget, "ShldrBend")
	do_create_widget_lr(create_limb_widget, "ShldrTwist")
	do_create_widget_lr(create_limb_widget, "ForearmBend")
	do_create_widget_lr(create_limb_widget, "ForearmTwist")

	for n in ["Carpal", "Pinky", "Ring", "Mid", "Index", "Thumb"]:
		func_to_call = create_limb_widget
		for i in range(1,4):
			if n != "Carpal" and i == 3:
				func_to_call = create_bone_widget
				bpy.ops.object.mode_set(mode='EDIT')
				amt.edit_bones["l%s%s" % (n,i)].tail = scale_bone_from_origin("l%s%s" % (n,i), 0.015)
				amt.edit_bones["r%s%s" % (n,i)].tail = scale_bone_from_origin("r%s%s" % (n,i), 0.015)
				bpy.ops.object.mode_set(mode='OBJECT')	
			do_create_widget_lr(func_to_call, "%s%s" % (n, i))
	do_create_widget_lr(create_limb_widget, "Carpal4")

	for n in ["BigToe", "SmallToe1", "SmallToe2", "SmallToe3", "SmallToe4"]:
		do_create_widget_lr(create_limb_widget, n)
	for n in ["BigToe_2", "SmallToe1_2", "SmallToe2_2", "SmallToe3_2", "SmallToe4_2"]:
		bpy.ops.object.mode_set(mode='EDIT')
		amt.edit_bones["l%s" % n].tail = scale_bone_from_origin("l%s" % n, 0.015)
		amt.edit_bones["r%s" % n].tail = scale_bone_from_origin("r%s" % n, 0.015)
		bpy.ops.object.mode_set(mode='OBJECT')	
		do_create_widget_lr(create_bone_widget, n)

def rename_bones():
	def rename_vertex_group(oldname, newname):
		for mesh_object in bpy.data.objects:
			if mesh_object.vertex_groups:
				for v in mesh_object.vertex_groups:
					if v.name == oldname:
						v.name = newname

	for bone in ob.data.bones:
		if bone.name[0] == 'l' and bone.name[1] == bone.name[1].upper():
			bone.name = bone.name[1:] + ".L"
			rename_vertex_group(bone.name, bone.name[1:] + ".L")
		elif bone.name[0] == 'r' and bone.name[1] == bone.name[1].upper():
			bone.name = bone.name[1:] + ".R"
			rename_vertex_group(bone.name, bone.name[1:] + ".R")


