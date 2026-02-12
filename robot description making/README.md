# How to Make a Robot Description Package

This file includes my personal attempts to make a robot description package based on a single urdf package.

You can find relevant official tutorial in [isaac sim robot set-up tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/tutorial_import_assemble_manipulator.html).

## General Steps
It is possible to encounter quite a few errors during the steps below. Therefore, the following steps only
serves as a structure to help developers clarify the sequence of making a robot description package. Detailed
methods and possibl errors can be found in the modules after GENERAL STEPS.
### 1. Use the xacro file to modularize components of the robot.
### 2. Turn to the README file and test the visualization.
    >    Note: Errors may be encountered during this stage, mostly because of the conflict between **macro definition and 
   data transmission**. You can use LLM to help fix it.
### 3. Write the files in /config:
   ```bash
   config
   ├── ocs2
   │   ├── fixed_base.info
   │   └── task.info
   └── ros2_control
       └── ros2_controllers.yaml
   ```
   Play the ocs2 control demo (mobile & arm)  to make sure it works.

   - **Possible ERROR**: 
   There is only a base_link model(without renderings) in your scene and the end effector balls. The terminal provides details below:
    ```bash
    [ERROR] [mobile_manipulator_mpc_node-3]: process has died [pid 26023, exit code -6, cmd '/home/shimanliang/ros2_ws/install/ocs2_mobile_manipulator_ros/lib/ocs2_mobile_manipulator_ros/mobile_manipulator_mpc_node --ros-args -r __node:=mobile_manipulator_mpc --params-file /tmp/launch_params_9stbutru --params-file /tmp/launch_params_pee3x8ze --params-file /tmp/launch_params_79vuvgqn'].
    ```
   - **Possible REASON**: 
   The order of joints in task.info doesn't match that in robot.xacro.
   > The mistake I've made: The robotic arm section reuses an existing robotic arm package, but the end-effector names did not match the references in robot.xacro (i.e., they were named differently). Changing them to match resolved the issue.

### 4. Import the robot into isaac sim, assign the nodes and try riviz control.

## XACRO and URDF
### Generating a URDF file
We usually use .xacro files to manage different parts of the robot, and use the following command to automatically generate a urdf file:
   ```bash
   source ~/ros2_ws/install/setup.bash
   xacro your_xacro_file_name.xacro -o your_urdf_file_name.urdf
   ```
    - **Possible ERROR**：file not found
    - **Possible REASON**:Maybe you have changed the context of the xacro file. So you'd better rebuild it first.
    
    
### The logic of an xacro file compilation
   ```bash
   xacro
   ├── components
   │   ├── arm.xacro
   │   ├── base.xacro
   │   ├── gripper.xacro
   │   └── wheel.xacro
   ├── component.xacro
   ├── robot.xacro
   ```
- The file "robot.xacro" serve as a general operator, where the following
xacro files in the component folder are cited and serially compiled.
- The exact files in the component folder depends on situation, which means
that it is acceptable that some robots don't own grippers or wheels and so forth.

## Robot Assembler Using
### Version of isaac-sim
- Use isaac sim 5.0 to assemble parts.
  > Note: As 6.0 is already downloaded in the original environment, isaacsim 5.0 ought to be downloaded using a distrobox.
- Use isaac sim 6.0 to run the control.
### Mind the DOMAIN ID
The background is that previously, all simulations ran normally locally on my host machine. However, to use the Robot Assembler in IsaacSim for robot component assembly, I installed IsaacSim 5.0 within a Ubuntu 22 container (originally I used IsaacSim 6.0 on the host). After successfully installing version 5.0 today, I imported the robot's base part into Isaac using the `import from ros2 urdf node` command. However, when I attempted to import the lift part next, all RVIZ instances failed and started reporting errors. Functionality only resumed after I modified my DOMAINID within the same terminal.

The root cause lies in: Isaac Sim's URDF import plugin is essentially an active ROS 2 node. It competed for resources with your host machine's Launch script on the same channel, causing severe resource contention.

Here is Gemini's analysis:
```bash
This is a classic and profound case of ROS 2 multi-robot/multi-environment communication conflict. Based on your background description, we can trace the root cause behind this chaos.

The fundamental issue lies in: Isaac Sim's URDF import plugin is essentially an active ROS 2 node. It engaged in severe resource contention with your host machine's Launch script within the same “channel.”

Below is a detailed breakdown of the causes and the logic explaining why you previously avoided this issue but encountered it now:
1. Root Cause: The “Invisible” Publisher Within the Isaac Sim 5.0 Container

When using Isaac Sim's Import from ROS 2 URDF node, Isaac Sim does not simply read a file. Instead, it launches a built-in ROS 2 node (typically containing a Bridge).

    UR5's Origin: Many built-in examples and test templates in Isaac Sim (especially older 5.0 versions) default to Universal Robots (UR) as the example robot. When you launch the import node with incomplete configuration overrides, Isaac Sim may activate a default robot_state_publisher or parameter server internally. This publisher continuously sends UR5 data to the /robot_description topic.

    Topic Collision: In ROS 2, if two nodes publish to the same topic (e.g., /robot_description), subscribers (like RViz) will alternately receive data from both. This explains why the joint list in RViz briefly displays the correct UR5 data before reverting to UR5's data.

2. Why did it work before but not now?

This involves changes to three variables:

Variable 1: Container Network Mode
When launching the Ubuntu 22 container, you most likely used the --net=host parameter. This fully exposes the ROS 2 discovery mechanism (DDS) within the container to the host machine.

    Variable 2: Isaac Sim 5.0 Behavior Differences Isaac Sim 6.0 and 5.0 implement the ROS 2 Bridge differently. The Import from ROS 2 URDF node in version 5.0 is more prone to leaving “hanging” residual nodes while waiting for connections.

    Variable 3: Node “Stickiness” After importing the Base section, the Isaac Sim import node may not fully close or release topic ownership. When preparing to import the Lift section, the previous import process continues publishing old (or default) URDF data in the background.

3. What Step Was Missed?

You didn't make any major mistakes—you simply overlooked ROS 2's default “fully open” communication mode.

The critical failure point lies in running two systems simultaneously on the same LAN (container and host share the network) and under the same DOMAIN_ID, both attempting to define “who the robot is.”

    The host machine's manipulator.launch.py declares: “I am the ARX robot.”

    The Isaac Sim import plugin (possibly with residual or default values) declares: “I am the UR5 robot.”

    RViz, acting as a “listener,” attempts to receive messages from both parties. Since ROS 2 lacks ROS 1's Master management mechanism, this leads to confusion.

4. Why does modifying DOMAIN_ID resolve the issue?

Modifying DOMAIN_ID effectively adds a layer of physical isolation to your terminal:

    Your host machine terminal (DOMAIN_ID 88) can now only receive ARX signals it sends itself.

    UR5 signals emitted by Isaac Sim inside the container (DOMAIN_ID 0) are blocked by this barrier.

    Result: RViz only receives data from DOMAIN 88, naturally eliminating interference.

5. Deep Dive: How to Prevent This in the Future?

When integrating Isaac Sim with ROS 2, follow this recommended workflow:

    Define Isolation Environments: If you only need to import URDF from ROS 2, assign Isaac Sim a dedicated ROS_DOMAIN_ID when launching its container, or include `export ROS_LOCALHOST_ONLY=1` in the container startup command.

    Clean residual topics: After each import failure or component change, check the host machine with `ros2 topic echo /robot_description` to verify the output matches your expectations.

    Beware of duplicate node names: The multiple `/dwj_Robot_node` instances you observed indicate Isaac Sim or its underlying drivers generated zombie nodes during reconnection attempts. These nodes continuously reset the parameter server.

Summary: This is not a technical error on your part, but a typical side effect of ROS 2's automatic discovery mechanism (DDS) in complex development environments. Your current solution (modifying DOMAIN_ID) is precisely the most standard and effective professional approach to resolving such conflicts.
```
As a result, make sure to occupy a domain id without conflicting with others under the same wifi.

> Note: Most issues encountered later with robots twitching or crashing in RVIZ are generally related to DOMAIN ID conflicts.

#### Ways to set a domain id
- You can set the domain id each time when opening a new terminal：
>  1. Set your own DOMAIN ID in the window where you're about to launch the visualization node
     ```bash
     export ROS_DOMAIN_ID=180 # Set arbitrarily to avoid conflicts
     ```
>  2. Launch the visualization node in the same terminal (CRITICAL)
>  3. Set the same DOMAIN ID in the terminal where you'll run IsaacSim
>  4. Start IsaacSim
>  5. Follow the steps to configure node import, then execute `import`

- Or you can set it permanently in .bashrc

### After importing modules
After importing each component and selecting its path, it will be saved as a USD file.

When assembling the entire robot, create a new scene and drag each component's USD file directly into it, placing them side-by-side under the parent prim.

After selecting the Robot Assembler and initiating assembly, the specific transform values must be viewed in RViz's TF view.
Note that these transform values represent world coordinates, whereas the origin xyz in XACRO refers to the joint coordinate system. Therefore, they must be viewed and modified within RViz.


## Model Processing
### Blender rendering
- Export as .obj
   > Note: with **Y as forward axis** and **Z as up axis**

- Export as .glb
   > Note: /Transform/+Y up should be disabled

### Model pivots
We usually set the pivot of a model not simply setting origin to its geometry, but in accordance with the joint origin.
For instance, setting the pivot to the side of bottom of a lift_link (as we usually need the lift joint to be straight, 
relatively).

In this case, change the pivot of the model in blender to your wanted origin. Record the transform data, and accordinately
change the origin number in your xacro file (to make sure it remains visually the same).

### Simple colliders
Since a complex model may affect the simulation, we usually struggle to reduce the complication of the link models.

One available method is to use simple models generated by an xacro file. You can follow the steps below to alter it after
other things are settled (which means that this process is not necessary, but serves as an optimization).
- Check the format of your original model file.
  - If .obj :
    - Drag the model file of the link into blender.
    - You'd better change its X orientation from 90 to 0.
    - Export it as an .stl file.
  - If .stl :
    - Go to the next step.
- Open meshlab.
  - If you don't have the app, use the command below:
    ```bash
    sudo apt install meshlab
    ```
    Use the command below to activate it:
    ```bash
    meshlab
    ```
  - Click File > Import Mesh > Your_STL_FILE > OPEN
  - Click Render > Show Box Corners
    - If your model is nearly regular, you can directly use the figure in the top left corner:
      - **Size** in the bounding box == **size of the box** in your xacro file
      - **Center** in the bounding box == **origin xyz** in your xacro file
    - If not, the figure can serve as a reference. You can use several regular geometries to assemble.
      - Xacro files support more than one collion parts. You can assign them with different names.
      - In most cases, you have to alter the accurate figures using your eyes.

### Self collisions
We set the self collisions in .info files to make sure links don't conflict with others.

There are two main figures used here:
- activationDistance: the maximum distance when the pairs are constrained by force (because of the close distance)
- minimumDistance: the minimum distance allowed between the pairs

To better understand the figures, we take d as the distance between the pairs.
- If d > activationDistance: smooth likes nothing happens
- If minimumDistance < d < activationDistance: resistance appears, and the closer the pairs are, the stronger the resistance is
- If d < minimumDistance: technically unreachable

Remember to regenerate the .urdf file, or the routine will overrun.


## Isaac-sim Processing
### Action Graph
Take ARX-X7S as an example:

Structure:

Graphs:
- Ros_jointStates
![joint graph](../.images/graph1.png)
- holonomic controller
![graph2](../.images/graph2.png)
![graph3](../.images/graph3.png)

## Errors
### error: no match articulations
```bash
[Error] [omni.physx.tensors.plugin] Pattern '/World/your_robot_name' did not match any rigid bodies
[Error] [omni.physx.tensors.plugin] Provided pattern list did not match any articulations
```
> Possible REASON: There is a lack of **articulation root** in the robot root prim, which results from the USD file itself (instead of anything that can be monitored through the property panel in isaac sim).

> Possible SOLUTION: Try add an articulation root to the root of the robot (mostly named after the robot's name).

Sometimes you may encounter with the error situation below:
```bash
[Error] [omni.physicsschema.plugin] UsdPhysics: Nested articulation roots are not allowed.
```
> REASON: There are more than one articulation root in the robot.

Possible solution: check each element of the robot to ensure that no **articulation root** in physics property is enabled (mostly it appears in a joint, especially the root joint) . In this case, use the script below to check for once:
```bash
from pxr import UsdPhysics, Usd
import omni.usd

stage = omni.usd.get_context().get_stage()
robot_root_path = "/World/Your_Robot_Name"
robot_prim = stage.GetPrimAtPath(robot_root_path)

def find_nested_articulation_roots(prim):
    root_list = []
    for child_prim in Usd.PrimRange(prim):
        if child_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_list.append(child_prim.GetPath())
    return root_list

if robot_prim:
    print("=== Scanning all the robots for ArticulationRoot ===")
    all_roots = find_nested_articulation_roots(robot_prim)
    
    if not all_roots:
        print("Not finding any ArticulationRoot。")
    else:
        print(f"Have found {len(all_roots)}  ArticulationRoots:")
        for path in all_roots:
            # If root prim:
            if path == robot_prim.GetPath():
                print(f"  [root] {path}")
            else:
                print(f"  [false！] {path} <- this has to be removed！")
else:
    print("No robots can be found.")
```

### error: the robot twitching in isaacsim
Prerequisite: After clicking simulate, no errors appear in the terminal.

Cause of jerkiness: Internal collisions within the robot.

**Possible SOLUTION**: Locate the articulation root and uncheck “Self collisions enabled.”

### error: TF_OLD_DATA
This happens when I was trying to use the command below to start a simulation:
```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_lift2s hardware:=isaac
  ```
It turns out that the robot doesn't move at all and the terminal continuously giving out warnings
marking the error of TF_OLD_DATA.

**Possible REASON**: ROS 2 namespace conflict issue

    Before modification, both Isaac Sim publishing JointState and ROS 2 receiving commands likely occurred under the root namespace (/), causing the following conflicts:
        
    - Topic Authority Conflict: Your ros2_control plugin (TopicBasedSystem) subscribed to /joint_states while simultaneously attempting to publish to the same topic or one with a similar path.
        
    - TF Broadcasters “Fighting”: When Isaac Sim's namespace is empty, it publishes directly to the global /joint_states and /tf. Your ROS-side robot_state_publisher also processes this data.
         
    - Feedback Loop Oscillation: Due to identical namespaces, ROS 2 nodes may mistakenly interpret Isaac's “state data” as their own “control feedback.” This explains why Rviz previously showed no response—the controller either believed the current position had reached the target or was “overridden” by conflicting data from the same namespace.

**Possible SOLUTION**: Change the namespace of ros2 publish joint state node to isaac (the default is none)

### error: one arm twitches while the other remains normal
Technically, the two robot arms have to be the same. So when the situation occurs, there
is possibility that the mapping relation doesn't match.

**Possible SOLUTION**: check your fixed_base.info based on your urdf file.
And make sure:
- The sequence of the joints is accordant.
- All fixed joints are not included(default).
- The joints to be listed in "Removed joints" are whose types are not fixed while you don't want it to be controlled below.
  - Mostly gripper joints.

As the mapping relation is extremely important, which means that all lists should in accordance with the joint tree, I personally
suggest the website [Robot URDF Viewer](https://viewer.robotsfan.com/) to assist in joint listing.