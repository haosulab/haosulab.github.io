"""Create an articulation (a tree of actors/links connected by joints)

Each actor in the articulation is also called link.
The robot is an instance of articulation.

Concepts:
    - Create an articulation
    - Control the articulation basically (builtin position and velocity controller)
        sapien.Articulation.set_qf, sapien.Joint.set_drive_velocity_target
    - sapien.Articulation.get_qpos, sapien.Articulation.get_qvel
"""

import sapien.core as sapien
from sapien.utils.viewer import Viewer
from PIL import Image, ImageColor, ImageDraw, ImageFont
import numpy as np
import transforms3d
from sympy import *
import time

exec(open('./cartpole_analysis.py').read())

def create_cartpole(
        scene: sapien.Scene,
        joint_friction=0.0,
        joint_damping=0.0,
        density=1000.0,
) -> sapien.Articulation:
    builder: sapien.ArticulationBuilder = scene.create_articulation_builder()

    base: sapien.LinkBuilder = builder.create_link_builder()  # LinkBuilder is similar to ActorBuilder
    base.set_name('base')

    cart = builder.create_link_builder(base)
    cart.set_name('cart')
    cart.set_joint_name('cart_joint')
    cart_half_size = np.array([0.2, 1.5, 1.5])
    cart.add_box_shape(half_size=cart_half_size, density=1)
    cart.add_box_visual(half_size=cart_half_size, color=[0.8, 0.6, 0.4])

    cart.set_joint_properties(
        'prismatic',
        limits=[[-np.inf, np.inf]],
        parent_pose=sapien.Pose(
            p=[0, 0, 0],
            q=transforms3d.quaternions.axangle2quat([0, 0, 1], np.deg2rad(90))
        ),
        child_pose=sapien.Pose(
            p=[0, 0, 0],
            q=transforms3d.quaternions.axangle2quat([0, 0, 1], np.deg2rad(90))
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    rod = builder.create_link_builder(cart)
    rod.set_name('rod')
    rod.set_joint_name('rod_joint')
    rod_half_size = np.array([0.02, 0.02, 1])
    rod.add_box_shape(half_size=rod_half_size, density=0.001)
    rod.add_box_visual(half_size=rod_half_size, color=[0.8, 0, 0])

    # The x-axis of the joint frame is the rotation axis of a revolute joint.
    rod.set_joint_properties(
        'revolute',
        limits=[[-np.inf, np.inf]],  # joint limits (for each DoF)
        parent_pose=sapien.Pose(
            p=[-(cart_half_size[0]+rod_half_size[0]+0.1), 0, (cart_half_size[2])],
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0)
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, -(rod_half_size[2]+0.1)], # try the other option later
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0.00)
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    sphere = builder.create_link_builder(rod)
    sphere.set_name('sphere')
    sphere.set_joint_name('sphere_joint')
    sphere.add_sphere_shape(radius = 0.1, density = 1)
    sphere.add_sphere_visual(radius = 0.1, color=[0.8, 0, 0])

    # The x-axis of the joint frame is the rotation axis of a revolute joint.
    sphere.set_joint_properties(
        'revolute',
        limits=[[0, 0]],  # joint limits (for each DoF)
        parent_pose=sapien.Pose(
            p=[0, 0, rod_half_size[2]],
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0)
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, 0.0], # try the other option later
            q=transforms3d.quaternions.axangle2quat([1, 0, 0], 0)
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    cartpole = builder.build(fix_base=True)
    cartpole.set_name('cartpole')

    return cartpole


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
    gravity = 9.81
    scene_config.gravity = np.array([0.0, 0.0, -gravity])
    scene_config.default_static_friction = args.static_friction
    scene_config.default_dynamic_friction = args.dynamic_friction
    scene_config.default_restitution = args.restitution
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 200.0)

    scene.add_ground(altitude=0)
    # ---------------------------------------------------------------------------- #
    # Build an inverted cartpole
    # ---------------------------------------------------------------------------- #
    cartpole = create_cartpole(scene, joint_friction=args.joint_friction, joint_damping=args.joint_damping)
    cartpole.set_pose(sapien.Pose(p=[0, 0, 2], q=[1, 0, 0, 0]))
    print('The dof of the articulation is', cartpole.dof)
    # ---------------------------------------------------------------------------- #
    viewer = Viewer(renderer)
    viewer.set_scene(scene)

    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(y=0, p=-np.arctan2(2, 2), r=0)
    viewer.window.set_camera_parameters(near=0.001, far=100, fovy=1)
    viewer.focus_actor(cartpole)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    rscene = scene.get_render_scene()
    rscene.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    # ---------------------------------------------------------------------------- #
    # Control the cart pole
    # ---------------------------------------------------------------------------- #
    joints = get_joints_dict(cartpole)
    print(joints.keys())
    steps = 0

    p_d = 0                 # target cart position. must be 0
    dpdt_d = 0              # target cart velocity. must be 0
    th_d = np.deg2rad(0)    # target rod pose. must be 0
    dthdt_d = 0             # target rod velocity. must be 0

    ##### controller params
    links = cartpole.get_links()
    Aeval = Ataylor.subs(M, links[1].get_mass()).subs(m, links[3].get_mass()).subs(l, 1).subs(g, gravity)
    Beval = B.subs(M, links[1].get_mass()).subs(m, links[3].get_mass()).subs(l, 1).subs(th, 0)

    Qmat = eye(2*dof, 2*dof)
    Qmat[0, 0] = 1
    Qmat[1, 1] = 100
    Qmat[2, 2] = 1
    Qmat[3, 3] = 1
    Rmat = eye(1, 1)

    Anp = np.array(Aeval.tolist()).astype(np.float64)
    Bnp = np.array(Beval.tolist()).astype(np.float64)
    Qmatnp = np.array(Qmat.tolist()).astype(np.float64)
    Rmatnp = np.array(Rmat.tolist()).astype(np.float64)

    S = sl.solve_continuous_are(Anp, Bnp, Qmatnp, Rmatnp)
    K = np.linalg.inv(Rmatnp)@Bnp.T@S
    print('K', K)

    images = []
    perturb_round = 0
    while not viewer.closed:
        pos = cartpole.get_qpos()[0]
        vel = cartpole.get_qvel()[0]
        theta = cartpole.get_qpos()[1]
        omega = cartpole.get_qvel()[1]

        f = np.array([0])
        if steps > perturb_round:
            x = np.array([pos, theta, vel, omega])
            f = -K@x
            print('controlling, at step %d.' % (steps-perturb_round), 'p', x[0], 'th', x[1], 'f', f)
            cartpole.set_qf(np.array([f[0], 0, 0]))
        else:
            print('perturbing the system')
            cartpole.set_qf(np.array([0, 10, 0]))

        if steps % 1 == 0:
            viewer.render()
            scene.update_render()
            rgba = viewer.window.download_float_target('Color')
            rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
            rgba_pil = Image.fromarray(rgba_img)

            draw = ImageDraw.Draw(rgba_pil)
            font = ImageFont.truetype("OpenSans-Regular.ttf", size=40)
            if steps <= perturb_round:
                draw.text((0, 0), 'perturbing the system', (0,0,0), font=font)
            else:
                draw.text((0, 0), 'controlling the system, at step %d, f=%f' % (steps-perturb_round, f[0]), (0,0,0), font=font)

            images.append(rgba_pil)
            if steps > 100:
               break

        scene.step()
        steps += 1

    print('saving video...')
    images[0].save('./cartpole.gif',
               save_all=True, append_images=images[1:], optimize=False, duration=40, loop=0)

    del scene


if __name__ == '__main__':
    main()

# Aeval = simplify(Ataylor.subs(M, 10).subs(m, 1).subs(l, 1).subs(g, 9.81))
# Beval = simplify(B.subs(M, 10).subs(m, 1).subs(l, 1).subs(th, 0))
#
# ##### controller params
# Qmat = eye(2*dof, 2*dof)
# Rmat = eye(1, 1)
#
# Anp = np.array(Aeval.tolist()).astype(np.float64)
# Bnp = np.array(Beval.tolist()).astype(np.float64)
# Qmatnp = np.array(Qmat.tolist()).astype(np.float64)
# Rmatnp = np.array(Rmat.tolist()).astype(np.float64)
#
# S = sl.solve_continuous_are(Anp, Bnp, Qmatnp, Rmatnp)
#
# K = np.linalg.inv(Rmatnp)@Bnp.T@S
