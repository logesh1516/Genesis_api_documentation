
# Genesis API Documentation

This document provides a concise reference for the **Genesis** simulation framework, detailing its core classes, methods, and properties.
> [!NOTE]
> Only essential api's are documented here , everything will be added in the future

---

## Table of Contents
- [How to use these API](#how-to-use-these-api)
- [Data Types](#data-types)  
- [Link & Joint Methods](#link--joint-methods)  
- [Kinematics](#kinematics)  
- [Path Planning](#path-planning)  
- [Degree of Freedom (DOF) Control](#degree-of-freedom-dof-control)  
- [Scene Properties](#scene-properties)  

---

## How to use these API

An Examlpe code block 

```py

        import genesis as gs
        
        gs.init(backend=gs.cpu)
        
        scene = gs.Scene()
        
        plane = scene.add_entity(
            gs.morphs.Plane(),
        )
        franka = scene.add_entity(
            gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
        )
        gs.logger.info(franka.get_dofs_position()) #<- example usage 
        
        scene.build()
        for i in range(1000):
            scene.step()
```

## Data Types

| Type                     | Description |
|--------------------------|-------------|
| **Tensor**               | Represents vectors or matrices returned from physics queries. Methods include: `get_pos()`, `get_quat()`, `get_vel()`, `get_ang()`, `get_verts()`, `get_AABB()` (returns a 2×3 array for lower and upper bounds),`inertial_i` (returns inertial tag in URDF: `ixx, ixy, ixz, iyy, iyz, izz`).|
| **NumPy**                | Provides numerical values and parameters: `get_mass()`, `set_mass(value)`, `set_friction(value)`, `get_friction()`.                  |
| **int**                  | Typically used for counting objects: `n_joints` returns the number of joints in a robot. |
| **bool**                 | Boolean flags: `is_leaf` (true if the link has no child links), `is_fixed`|
| **property**             | Scene information: `solver` retrieves the solver information of the scene. |

---

## Link & Joint Methods

### Accessing Links & Joints

- `get_joint(str)` → Returns a joint object by name.  
- `get_link(str)` → Returns a link object by name.  

### Link Methods

- `get_pos()` → Current link position.  
- `get_links_pos()` → Positions of all links.  
- `get_links_quat()` → Orientations (quaternions) of all links.  
- `get_links_vel()` → Linear velocities of all links.  
- `get_links_ang()` → Angular velocities of all links.  
- `get_links_acc_ang()` → Angular accelerations of all links.  

### Link Manipulation

- `set_pos(pos[], relative=bool)` → Set link position.  
- `set_quat(quat[], relative=bool)` → Set link orientation (quaternion).  

---

## Kinematics

- `get_jacobian(link)` → Returns a **Tensor** representing the Jacobian of a link.  
- `inverse_kinematics(link, pos[], quat[])` → Computes joint configuration for a desired link pose.  
- `inverse_kinematics_multilink(links, poss[], quats[])` → Computes joint configuration for multiple links simultaneously.  
- `forward_kinematics(qpos[])` → Computes the forward kinematics for a given joint configuration.  

---

## Path Planning

- `plan_path(qpos_goal[], timeout, num_waypoints, planner=(RRT || RRTConnect))` → Plans a path to a target joint configuration.  

---

## Degree of Freedom (DOF) Control

### Setting DOFs

| Method | Description |
|--------|-------------|
| `set_dofs_kp(kp[])` | Set positional gain (P) for joints. |
| `get_dofs_kp()` | Get current positional gain. |
| `set_dofs_kv(kv[])` | Set velocity gain (D) for joints. |
| `get_dofs_kv()` | Get current velocity gain. |
| `set_dofs_force_range([])` | Set allowable force range for DOFs. |
| `get_dofs_force_range()` | Get force range of DOFs. |
| `set_dofs_damping([])` | Set damping for DOFs. |
| `get_dofs_damping()` | Get damping of DOFs. |
| `set_dofs_velocity([])` | Set maximum velocity for DOFs. |
| `get_dofs_velocity()` | Get current DOF velocities. |
| `set_dofs_frictionloss([])` | Set friction loss for DOFs. |
| `get_dofs_friction_loss()` | Get friction loss of DOFs. |
| `set_dofs_position([])` | Set joint positions. |
| `get_dofs_position()` | Get joint positions. |

### Controlling DOFs

| Method | Description |
|--------|-------------|
| `control_dofs_force([])` | Apply force/torque control to DOFs. |
| `control_dofs_velocity([])` | Apply velocity control to DOFs. |
| `control_dofs_position([])` | Apply position control to DOFs. |
| `set_qpos([])` | Set joint angles. |
| `get_qpos()` | Get current joint angles. |
| `get_dofs_force()` | Get forces applied on DOFs. |
| `get_dofs_force()` | Get min/max force limits for DOFs. |
| `zero_all_dofs_velocity()` | Reset all joint velocities to zero. |

---

## Scene Properties

- `solver` → Retrieves the solver information of the scene (e.g., physics solver type, parameters).

