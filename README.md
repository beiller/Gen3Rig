# Gen3Rig
Convert from genesis 3 DAE export into a IK rig

Will show up in the properties panel. Select armature in object or pose mode and press generate button.

# Instructions for Rigify (Pitchipoy)

Update we now support a Rigify rig (Pitchipoy)!

Map the mesh to an armature modifier, select the *mesh* and then click "Generate Physpose Rig" button.
* Make sure under template, select "Rigify (Pitchipoy)"
* Select only the mesh, then click "Generate Physpose Rig".
  * boxes are created which are really convex mesh (click display as shapes button to view the meshes)
  * pin will pin one of the convex meshes
* Make sure to create a "ground plane" rigid body object or your character will just "fall"
* Press alt+a (play) to start the simulation

Make sure the rig is in FK mode afterwards to see results.

# Instructions for Genesis 3

* Import the dae file exported from daz
    * The armature may not be scaled properly. Use edit mode to fix it's position (morphs that shrink seem to cause this)
* Select the armature and click "Convert Genesis 3 rig" in properties panel (at bottom but top of physpose tools)
    * An armature is created with shapes, and constraints, but all layers are hidden. Unhide all layers to see the full rig.
    * The "bone roll" on the legs is wrong for some reason sometimes. To fix, enter edit mode, select all leg bones and set roll to 0 degrees.

# Notes about use on makehuman

Makehuman (old version, export full rig) is experimental. Select makehuman v1 to try.

# Hints

* up the frame counter to 300000 (maximum) and also set Rigid Body Cache to 300000 end frame.
* up steps per second to 60, and solver iterations to 500 for best results (slow)
* click "draw as shapes" button to see concave mesh and debug weird collisions that make it unstable. Try moving things on different collision layers if its seriously messed up.

# A custom template

I used a custom template for my example video if you have seen it. The hair is also physics enabled (pony-tails).

Make a file in the root directory of this plugin, called custom_template.py. A function must be created
that has the same name as the Armature object. Add contents such as this:

~~~~
def PricessRig(poserig):
    # freedom of movement for the bone chain
    braid_dof = 65.0
    # build a chain of physbones. Start at "braid.R" which is the name of the bone.
    # Connect the chain root to bone "head"
    # Give the name of the mesh object to "shrink wrap" the rigid bodies so they can collide correctly
    # An Initial size of the bone, pre-shrink-wrap. Ensure its large enough to encapsulate the mesh
    # set dof
    # Density of the rigid body object.
    poserig.build_chain("braid.R", "head", "FTBraids-skinInstance.001", bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof), density=10)
    poserig.build_chain("braid.L", "head", "FTBraids-skinInstance.001", bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof), density=10)

    # Also I made jewlery dynamic
    # I used 2 bones instead of 1 for each jewlery piece. Limitation of this script at the moment.
    poserig.build_chain("bellyRing1", "abdomenLower", "MoonStoneBellyCharm_41324", bone_size=0.1, bone_dof=(180, 3, 180))
    poserig.build_chain("earring.R", "head", "REarring", bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof))
    poserig.build_chain("earring.L", "head", "LEarring", bone_size=0.1, bone_dof=(braid_dof, braid_dof, braid_dof))

    # This call is used to move things around on layers (so the earrings dont go crazy colliding with everything)
    poserig.move_physobj_collision_layer(['earring.R', 'earring.L', 'earring.R.001', 'earring.L.001'], layer_number=3)

def MyRig2(poserig):
    pass

#This line, PricessRig is the function name and ALSO the name of the armature object.

templates['PricessRig'] = PricessRig
templates['MyRig2'] = MyRig2
~~~~