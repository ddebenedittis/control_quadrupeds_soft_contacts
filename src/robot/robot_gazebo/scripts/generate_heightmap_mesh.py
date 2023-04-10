#!/usr/bin/env python3
 
import bpy
import rclpy



def generate_heightmap_meshes():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False, confirm=False)

    bpy.ops.mesh.primitive_plane_add(size=2, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))

    bpy.ops.object.editmode_toggle()

    bpy.ops.mesh.subdivide(number_cuts=31)
    bpy.ops.mesh.subdivide(number_cuts=15)

    bpy.ops.object.editmode_toggle()

    bpy.ops.texture.new()
    bpy.data.textures["Texture"].type = 'IMAGE'
    bpy.ops.image.open(filepath="/home/davide/Dropbox/phd/quadrupeds/control_quadrupeds_soft_contacts/src/robot/robot_gazebo/models/heightmaps/heightmap_1.png", directory="/home/davide/Dropbox/phd/quadrupeds/control_quadrupeds_soft_contacts/src/robot/robot_gazebo/models/heightmaps", files=[{"name":"heightmap_1.png", "name":"heightmap_1.png"}], relative_path=True, show_multiview=False)
    bpy.data.textures["Texture"].image = bpy.data.images["heightmap_1.png"]

    bpy.ops.object.modifier_add(type='DISPLACE')
    bpy.context.object.modifiers["Displace"].texture = bpy.data.textures["Texture"]
    bpy.ops.object.modifier_apply(modifier='Displace')

    bpy.ops.object.modifier_add(type='DISPLACE')
    bpy.context.object.modifiers["Displace"].direction = "Z"
    bpy.context.object.modifiers["Displace"].strength = 0.81
    bpy.ops.object.modifier_apply(modifier='Displace')

    bpy.ops.object.shade_smooth()

    bpy.ops.object.convert(target='MESH')

    bpy.ops.wm.collada_export(filepath='/home/davide/Dropbox/phd/quadrupeds/control_quadrupeds_soft_contacts/src/robot/robot_gazebo/models/heightmaps/heightmap_visual.dae')

    # ============================================================================ #

    bpy.ops.object.editmode_toggle()

    bpy.ops.mesh.select_all(action='SELECT')

    bpy.ops.mesh.remove_doubles(threshold=0.01)

    bpy.ops.wm.collada_export(filepath='/home/davide/Dropbox/phd/quadrupeds/control_quadrupeds_soft_contacts/src/robot/robot_gazebo/models/heightmaps/heightmap_collision.dae')



def main(args=None):
    rclpy.init(args=args)
    
    generate_heightmap_meshes()
    
    rclpy.shutdown()



if __name__ == '__main__':
    main()