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
每一个部件导入并选择路径后，会保存为usd文件

拼接整个机器人的时候，新建一个场景，直接把每一个部件的usd托进去，在父prim下并列

选择robot assembler并开始assemble后，具体的transform数值要在rviz的TF里展开看。
注意，这里的transform数值是世界坐标，而xacro里的origin xyz是相对于joint坐标系而言的，所以得再RVIZ里去看和修改。


## Blender Rendering
- Export as .obj
   > Note: with **Y as forward axis** and **Z as up axis**

- Export as .glb
   > Note: /Transform/+Y up should be disabled




## Action Graph
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
> REASON: There is a lack of **articulation root** in the robot root prim, which results from the USD file itself (instead of anything that can be monitored through the property panel in isaac sim).

However, the following script can be used to add a **physics** property to the robot prim:
```bash
from pxr import UsdPhysics, Usd
import omni.usd

# 获取当前场景
stage = omni.usd.get_context().get_stage()
# 你的机器人根Prim路径
robot_root_path = "/World/lift_2s/lift2"
robot_prim = stage.GetPrimAtPath(robot_root_path)

if robot_prim:
    # 1. 为顶层Prim添加ArticulationRootAPI（关键！）
    if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
        print(f"✓ 已为 {robot_root_path} 添加 ArticulationRootAPI")
    
    # 2. 遍历机器人，为所有几何体（Mesh）添加刚体和碰撞属性
    from pxr import UsdGeom
    rigid_count = 0
    for prim in Usd.PrimRange(robot_prim):
        if prim.IsA(UsdGeom.Mesh):
            # 添加RigidBodyAPI
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(prim)
            # 添加CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            rigid_count += 1
    
    print(f"✓ 已为 {rigid_count} 个Mesh部件添加刚体与碰撞属性")
    print("修复完成。请保存场景并重新测试Action Graph。")
else:
    print(f"✗ 错误：未找到路径 {robot_root_path}")
```


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
robot_root_path = "/World/lift_2s/lift2"
robot_prim = stage.GetPrimAtPath(robot_root_path)

def find_nested_articulation_roots(prim):
    root_list = []
    for child_prim in Usd.PrimRange(prim):
        if child_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_list.append(child_prim.GetPath())
    return root_list

if robot_prim:
    print("=== 正在扫描机器人中所有的 ArticulationRoot ===")
    all_roots = find_nested_articulation_roots(robot_prim)
    
    if not all_roots:
        print("未发现任何 ArticulationRoot。")
    else:
        print(f"共发现 {len(all_roots)} 个 ArticulationRoot:")
        for path in all_roots:
            # 判断是否为顶层
            if path == robot_prim.GetPath():
                print(f"  [顶层] {path}")
            else:
                print(f"  [嵌套错误！] {path} <- 这个必须被移除！")
else:
    print("找不到机器人。")
```

### error: isaacsim里机器人抽搐
前提：点击仿真后，终端无报错

抽搐原因：机器人内部碰撞

解决方法：找到articulation root，取消勾选self collisions enabled