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

bl_info = {
    "name": "Gen3Rig",
    "version": (0, 1),
    "author": "beiller",
    "blender": (2, 40, 0),
    "description": "Automatic rig conversion for genesis 3 characters, exported as DAE",
    "location": "View3d tools panel",
    "wiki_url": "https://github.com/beiller/Gen3Rig/wiki",
    "tracker_url": "https://github.com/beiller/Gen3Rig/issues",
    "category": "Rigging"
}

if "bpy" in locals():
    import imp
    imp.reload(utils)
    imp.reload(main)
    imp.reload(physobject)
else:
    from . import utils, main, physobject
import bpy

class Gen3RigBuild(bpy.types.Operator):
    """Gen3RigBuild"""
    bl_idname = "object.convert_gen3_rig"
    bl_label = "Convert Genesis 3 Rig"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        bpy.ops.object.mode_set(mode='OBJECT')
        ob = bpy.context.object
        ob.show_x_ray = True
        ob.name = "GenesisRig"
        amt = ob.data
        amt.name = "GenesisRigArmature"
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        main.ob = ob
        main.amt = amt
        main.do_setup()
        main.create_gaze_bones()
        main.create_arm_ik()
        main.create_leg_ik()
        main.setup_widgets()
        main.finalize_layers()
        main.rename_bones()
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.context.scene.update()

        return {'FINISHED'}

#
#    Menu in UI region
#
class UIPanel(bpy.types.Panel):
    bl_label = "Convert Genesis 3 Rig"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
 
    def draw(self, context):
        self.layout.operator("object.convert_gen3_rig", text='Convert Genesis 3 Rig')


def menu_func(self, context):
    self.layout.operator(Gen3RigBuild.bl_idname)


def register():
    bpy.utils.register_class(Gen3RigBuild)
    bpy.utils.register_class(UIPanel)
    physobject.register()


def unregister():
    bpy.utils.unregister_class(Gen3RigBuild)
    bpy.utils.unregister_class(UIPanel)
    physobject.unregister()


if __name__ == "__main__":
    register()

