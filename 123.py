# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
'''
UR10 controller
'''
# from omni.isaac.examples.base_sample import BaseSample
# from omni.isaac.core.utils.types import ArticulationAction
# from omni.isaac.core.utils.nucleus import get_assets_root_path
# from omni.isaac.core.utils.stage import add_reference_to_stage
# from omni.isaac.core.objects import DynamicCuboid
# import omni.isaac.core.utils.stage as stage_utils
# from omni.isaac.core import World
# from omni.isaac.core.prims import XFormPrim
# from pxr import Usd, UsdGeom, Sdf, Gf,UsdPhysics
# import omni.usd
# import numpy as np
# import carb


# class HelloWorld(BaseSample):
#     def __init__(self) -> None:
#         super().__init__()  

#         return
#     def setup_scene(self):
#         world = World(stage_units_in_meters=100)
#         # physic_tool=PhysicsContex(prim_path="/physicsScene")
#         # physic_tool.set_gravity()
#         stage_utils.set_stage_units(0.01)
#         print(stage_utils.get_stage_units())
#         world.scene.add_default_ground_plane()
#         # you configure a new server with /Isaac folder in it
#         assets_root_path = get_assets_root_path()
#         if assets_root_path is None:
#             # Use carb to log warnings, errors and infos in your application (shown on terminal)
#             carb.log_error("Could not find nucleus server with /Isaac folder")
#         asset_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"
#         # This will create a new XFormPrim and point it to the usd file as a reference
#         # Similar to how pointers work in memory
#         rope_prim_path="/World/Fancy_Rope"
#         rope_usd_path="D:/Documents/SRT/ov/pkg/isaac_sim-2023.1.1/extension_examples/user_examples/Rope.usd"
#         add_reference_to_stage(usd_path=rope_usd_path, prim_path=rope_prim_path)
#         # Wrap the jetbot prim root under a Robot class and add it to the Scene
#         # to use high level api to set/ get attributes as well as initializing
#         # physics handles needed..etc.
#         # Note: this call doesn't create the Jetbot in the stage window, it was already
#         # created with the add_reference_to_stage
#         rope_prim=XFormPrim(prim_path=rope_prim_path,name="rope")
#         rope=world.scene.add(rope_prim)
#         # jetbot_robot = world.scene.add(UR10(prim_path="/World/Fancy_Robot", name="fancy_robot"))
#         # Note: before a reset is called, we can't access information related to an Articulation
#         # because physics handles are not initialized yet. setup_post_load is called after
#         # the first reset so we can do so there
#         # print("Num of degrees of freedom before first reset: " + str(jetbot_robot.num_dof)) # prints None
#         return
#     def create_joint(self,prim_path1=None,prim_path2=None,joint_path=None):
#         stage = omni.usd.get_context().get_stage()
#         joint = UsdPhysics.FixedJoint.Define(stage, joint_path)

#         # define joint bodies
#         joint.CreateBody0Rel().SetTargets(prim_path1)
#         joint.CreateBody1Rel().SetTargets(prim_path2)

#         # define joint positions
#         joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
#         joint.CreateLocalPos1Attr().Set(-Gf.Vec3f(0.0, 0.0, 0.0))
#         joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
#         joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
#         return joint
#     async def setup_post_load(self):
#         self._world = self.get_world()
#         # print(self._iksolver.get_end_effector_frame())
#         self._rope=self._world.scene.get_object("rope")
#         self._stage: Usd.Stage = omni.usd.get_context().get_stage()
#         self._pointInstance:UsdGeom.PointInstancer=UsdGeom.PointInstancer.Get(self._stage,Sdf.Path("/World/Fancy_Rope/Rope0/rigidBodyInstancer"))
#         print(self._rope,self._pointInstance)
#         print(self._pointInstance.GetProtoIndicesAttr())
#         # self._cube=self._world.scene.add(
#         #     DynamicCuboid(prim_path="/World/dynamic_cube",
#         #                     name="Fancy_cube",
#         #                     position=np.array([-151.5, 0, 72.1]),
#         #                     scale=np.array([1, 1, 1]),
#         #                     color=np.array([0, 0, 1.0])))
#         # print(self._rope,self._cube)
#         # self._joint=self.create_joint(Sdf.Path("/World/dynamic_cube"),Sdf.Path("/World/Fancy_Rope"),Sdf.Path("/World/dynamic_cube/joint"))
#         # This is an implicit PD controller of the jetbot/ articulation
#         # setting PD gains, applying actions, switching control modes..etc.
#         # can be done through this controller.
#         # Note: should be only called after the first reset happens to the world
#         # If no articulation_controller was passed during class instantiation, 
#         # a default controller of type ArticulationController (a Proportional-Derivative 
#         # controller that can apply position targets, velocity targets and efforts) will be used
#         # Adding a physics callback to send the actions to apply actions with every
#         # physics step executed.
#         self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
#         return
#     def send_robot_actions(self, step_size):
#         print("!!!!")
#         # Every articulation controller has apply_action method
#         # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
#         # as optional args. It accepts numpy arrays of floats OR lists of floats and None
#         # None means that nothing is applied to this dof index in this step
#         # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
#         return
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.tasks import BaseTask
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdShade,  Usd #omni.usd.libs
from pxr import UsdPhysics #omni.usd.schema.physics
from pxr import PhysxSchema #omni.usd.schema.physx
from omni.physx.scripts import physicsUtils
from omni.isaac.core.utils.stage import get_current_stage
import omni
from omni.physx.scripts import utils
import numpy as np



class FrankaPlaying(BaseTask):
    
    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name, world):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.495, 0, 0.33 ])
        self._task_achieved = False
        self._world = world
        self._t = 8
        return

    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self.stage1 = get_current_stage()  
        #print("the stage1 is:",self.stage1)     
        self._rope = self.create(self.stage1)                       # add the rope in the scene


        return


    ######################################################## Rope
    
    def create(self, stage):
        self._stage:Usd.Stage = stage
        #print("the stage in hello world is:",self._stage)
        prim = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World"))
        self._defaultPrimPath:Sdf.Path = prim.GetPath()
        print("the defaultPrimPath is:",self._defaultPrimPath) #/World
        

        # configure ropes:
        self._linkHalfLength = 0.03/self._t
        self._linkRadius = 0.5 * self._linkHalfLength
        self._ropeLength = 0.25 #3
        self._numRopes = 1
        self._ropeSpacing = 2 #0.15    ## the spacing between different ropes
        self._ropeColor = Gf.Vec3f(0.4, 0.2, 0.1)
        self._coneAngleLimit = 170
        self._rope_damping = 5.0
        self._rope_stiffness = 0.4

        # configure collider capsule:
        self._capsuleY = 0.2
        self._capsuleHeight = 30 #10
        self._capsuleRadius = 0.2
        self._capsuleRestOffset = -2.0  ###
        self._capsuleColor = Gf.Vec3f(0.2, 0.1, 0.4)

        # physics options:
        self._contactOffset = 2.0  ###
        self._physicsMaterialPath = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        #print("The path is: ",self._physicsMaterialPath)
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material:UsdPhysics.MaterialAPI = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5) #0.5
        material.CreateDynamicFrictionAttr().Set(0.5) #0.5
        material.CreateRestitutionAttr().Set(0)

        self._createRopes()
        #self._createColliderCapsule()
        

    def _createCapsule(self, path: Sdf.Path, location: Gf.Vec3f):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
        #capsule_name = path.split("/")[-1]
        #print('the type of the path is :',type(path))
        #print('the path of the capsule is :',path)
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.AddTranslateOp().Set(location)
        capsuleGeom.AddOrientOp().Set(Gf.Quatf(1.0))
        capsuleGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(.0005)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)
        

    def _createJoint(self, previousPath: Sdf.Path, linkPath: Sdf.Path):
        jointPath = linkPath.AppendChild("SphericalJoint")
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        joint.CreateBody0Rel().SetTargets([previousPath])
        joint.CreateBody1Rel().SetTargets([linkPath])
        if previousPath.name == "Link0":
            #jointX = 0.5
            #joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5/self._t, 0, 0))
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            #joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.0225/self._t, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        else:
            jointX = self._linkHalfLength #- 0.5 * self._linkRadius
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(jointX, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-jointX, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # locked DOF (lock - low is greater than high)
        d6Prim = joint.GetPrim()
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transX")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transY")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transZ")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "rotX")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)

        # Moving DOF:
        dofs = ["rotY", "rotZ"]
        for d in dofs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)

            # joint drives for rope dynamics:
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._rope_damping)
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)
        


    def _createRopes(self):
        #linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        linkLength = 2.0 * self._linkHalfLength
        numLinks = int(self._ropeLength / linkLength)
        #xStart = -numLinks * linkLength * 0.5
        xStart = -0.495
        zStart = -(self._numRopes // 2) * self._ropeSpacing
        
        self.Link_name = []
        self.Link_loc = []

        for ropeInd in range(self._numRopes):
            xformPath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            ropeXform = UsdGeom.Xform.Define(self._stage, xformPath)
            z = zStart + ropeInd * self._ropeSpacing + 0.015  #### +33
            y = self._capsuleY + self._capsuleRadius + self._linkRadius * 1.4 
            ropeXform.AddTranslateOp().Set(Gf.Vec3f(0, y, z))
            previousPath = Sdf.Path()
            print('the previosPath is :', previousPath)
            for linkInd in range(numLinks):
                #print('now the linkInd is :',linkInd)
                # 创建第一个胶囊体，和DynamicCube连接
                if linkInd == 0:
                    x = xStart + linkInd * linkLength 
                    linkPath = xformPath.AppendChild(f"Link{linkInd}")
                    self.Link_name.append(linkPath)
                    #loc = Gf.Vec3f(x, 0, 0)
                    self._world.scene.add(DynamicCuboid(prim_path="/World/Rope0/Link0",
                                            name=linkPath.name,
                                            position=Gf.Vec3f(x, y, z),
                                            scale=np.array([0.06/4, 0.03/4, 0.03/4]),   #Gf.Vec3f(0.045, 0.03, 0.03)  np.array([0.045, 0.03, 0.03])
                                            color=np.array([1.0, 0, 0])))
                    self.Link_loc.append(Gf.Vec3f(x, y, z))
                    if not previousPath.isEmpty:
                        #  创建previousPath, linkPath两个对象的joint
                        self._createJoint(previousPath, linkPath)
                    previousPath = linkPath
                else:
                    x = xStart + linkInd * linkLength #* 1.2
                    print("the x is :",x)
                    linkPath = xformPath.AppendChild(f"Link{linkInd}")
                    self.Link_name.append(linkPath)
                    loc = Gf.Vec3f(x, 0, 0)
                #prim_pos,prim_rot = linkPath.name.get_world_pose()
                #print('the prim_pos is :',prim_pos)
                    self.Link_loc.append(Gf.Vec3f(x, y, z))
                    # 创建两个关节间的胶囊体，并且添加物理属性
                    self._createCapsule(linkPath, loc)
                    if not previousPath.isEmpty:
                        self._createJoint(previousPath, linkPath)
                    previousPath = linkPath
            #print('the link name and the location are:',Link_name,Link_loc)
            #return Link_name, Link_loc





class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # We add the task to the world here
        world.add_task(FrankaPlaying(name="my_first_task", world=world))
        return
