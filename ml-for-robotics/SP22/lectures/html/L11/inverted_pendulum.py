import sapien.core as sapien
from sapien.utils.viewer import Viewer
from PIL import Image, ImageColor
import numpy as np
import transforms3d
import time

def create_pendulum(
        scene: sapien.Scene,
        joint_friction=0.0,
        joint_damping=0.0,
        density=1.0,
) -> sapien.Articulation:
    builder: sapien.ArticulationBuilder = scene.create_articulation_builder()
    base: sapien.LinkBuilder = builder.create_link_builder()  # LinkBuilder is similar to ActorBuilder
    base.set_name('base')
    base_half_size = np.array([0.0, 0.0, 0.0])
    base.add_box_shape(half_size=base_half_size, density=density)
    base.add_box_visual(half_size=base_half_size, color=[0.8, 0.6, 0.4])

    rod = builder.create_link_builder(base)
    rod.set_name('rod')
    rod.set_joint_name('rod_joint')
    rod_half_size = np.array([0.1, 0.1, 1])
    rod.add_box_shape(half_size=rod_half_size, density=density)
    rod.add_box_visual(half_size=rod_half_size, color=[0.8, 0, 0])

    # The x-axis of the joint frame is the rotation axis of a revolute joint.
    rod.set_joint_properties(
        'revolute',
        limits=[[-np.deg2rad(10*180), np.deg2rad(10*180)]],  # joint limits (for each DoF)
        # parent_pose refers to the relative transformation from the parent frame to the joint frame
        # child_pose refers to the relative transformation from the child frame to the joint frame
        parent_pose=sapien.Pose(
            p=[0, 0, 0],
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0)
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, (rod_half_size[2]+0.1)], # try the other option later
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0.01)
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    inverted_pendulum = builder.build(fix_base = True)
    inverted_pendulum.set_name('inverted_pendulum')
    return inverted_pendulum


def get_joints_dict(articulation: sapien.Articulation):
    joints = articulation.get_joints()
    joint_names =  [joint.name for joint in joints]
    assert len(joint_names) == len(set(joint_names)), 'Joint names are assumed to be unique.'
    return {joint.name: joint for joint in joints}


def parse_args():
    import argparse
    parser = argparse.ArgumentParser()

    parser.add_argument('--static-friction', default=0.0, type=float, help='static friction')
    parser.add_argument('--dynamic-friction', default=0.0, type=float, help='dynamic friction')
    parser.add_argument('--restitution', default=0.1, type=float, help='restitution (elasticity of collision)')
    parser.add_argument('--joint-friction', default=0.0, type=float, help='joint friction')
    parser.add_argument('--joint-damping', default=0.0, type=float,
                        help='joint damping (resistance proportional to joint velocity)')

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene_config.gravity = np.array([0.0, 0.0, -9.8])
    scene_config.default_static_friction = args.static_friction
    scene_config.default_dynamic_friction = args.dynamic_friction
    scene_config.default_restitution = args.restitution
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 100.0)

    # ---------------------------------------------------------------------------- #
    # Build an inverted pendulum
    # ---------------------------------------------------------------------------- #
    pendulum = create_pendulum(scene, joint_friction=args.joint_friction, joint_damping=args.joint_damping)
    pendulum.set_pose(sapien.Pose(p=[0., 0., 0.0]))
    print('The dof of the articulation is', pendulum.dof)
    # ---------------------------------------------------------------------------- #

    viewer = Viewer(renderer)
    viewer.set_scene(scene)

    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(y=0, p=-np.arctan2(2, 2), r=0)
    viewer.window.set_camera_parameters(near=0.001, far=100, fovy=1)
    viewer.focus_actor(pendulum)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    rscene = scene.get_render_scene()
    rscene.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    joints = get_joints_dict(pendulum)
    print(joints.keys())

    position = 0.0  # position target of joints
    velocity = 0.0  # velocity target of joints
    steps = 0

    Kp = 50
    Kv = 10
    Ki = 5
    th_d = np.deg2rad(180)
    v_d = 0

    c = 0
    images = []
    while not viewer.closed:
        if steps > 200:
            th = pendulum.get_qpos()
            v = pendulum.get_qvel()
            e = th-th_d
            e_v = v-v_d
            c=c+e
            tau = -Kp*e-Kv*e_v-Ki*c
            pendulum.set_qf(tau)

            rgba = viewer.window.download_float_target("Color")
            rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
            rgba_pil = Image.fromarray(rgba_img)
            images.append(rgba_pil)

            if steps > 600:
                break

        scene.step()
        scene.update_render()
        viewer.render()

        if steps % 100 == 0:
            if steps > 200:
                print('in control, steps = %d' % steps)
            print('Pose', pendulum.get_pose())
            print('Joint positions', pendulum.get_qpos())
            print('Joint velocities', pendulum.get_qvel())
        steps += 1

    images[0].save('./inverted_pendulum.gif',
               save_all=True, append_images=images[1:], optimize=False, duration=40, loop=0)
    del scene


if __name__ == '__main__':
    main()
