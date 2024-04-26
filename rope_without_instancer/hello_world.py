
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core import objects,World,tasks
from omni.isaac.franka.controllers import PickPlaceController
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdShade,  Usd #omni.usd.libs
from pxr import UsdPhysics #omni.usd.schema.physics
from pxr import PhysxSchema #omni.usd.schema.physx
from omni.physx.scripts import physicsUtils
from omni.isaac.core.utils.stage import get_current_stage
from omni.physx.scripts import utils
import sys
sys.path.append(r'D:\\Documents\\SRT\\ov\\pkg\\isaac_sim-2023.1.1\\extension_examples\\user_examples')
from random_target import randomTarget
import numpy as np
import time
import carb


class MoveRope(tasks.BaseTask):
    
    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        # 左右端点的目标位置
        self._goalR_position=None
        self._goalL_position=None
        # self._goalR_position=np.array([0.8,1.8,2.6])
        # self._goalL_position=np.array([0,2,2.6])
        self._originPosition = None
        # 任务刚开始时，左右两个动力块儿到目标的距离
        self._distanceR=None
        self._distanceL=None
        self._task_achieved = False
        self._jointDriveY=[]
        self._jointDriveZ=[]
        # self._arr=[]
        return

    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self.stage:Usd.Stage = get_current_stage()  
        #print("the stage1 is:",self.stage1)     
        self._rope = self.create(self.stage,scene)                       # add the rope in the scene
        return


    ######################################################## Rope
    def create(self, stage,scene:World.scene):
        self._stage:Usd.Stage = stage
        #print("the stage in hello world is:",self._stage)
        prim:PhysxSchema.PhysxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(self._stage.GetPrimAtPath("/World")) #获取当前的世界场景的句柄
        self._defaultPrimPath:Sdf.Path = prim.GetPath() #获取当前的世界场景的prim_path
        # print("the defaultPrimPath is:",self._defaultPrimPath) #/World
        

        # configure ropes:
        self._linkHalfLength = 0.01
        self._linkRadius = 0.5 * self._linkHalfLength
        self._ropeLength = 1 #3
        self._numRopes = 1
        self._ropeSpacing = 0.15 #0.15    ## the spacing between different ropes
        self._ropeColor = Gf.Vec3f(0.4, 0.2, 0.1)
        self._coneAngleLimit = 120 #np.pi/36
        self._density = 50* 27*0.01  # 变小可以看出力的作用变小，但没有看出阻尼和刚度的影响
        # Force or acceleration = stiffness * (targetPosition - position)>damping * (targetVelocity - velocity)
        self._rope_damping = 10000#1*0.01*0.01# *0.00001 nothing happened# *10**10 nothing happened # mass*DIST_UNITS*DIST_UNITS/second/degrees.
        self._rope_stiffness =10000 #0.1*0.01*0.01# *0.00001 nothing happened# *10**10 nothing happened#: mass*DIST_UNITS*DIST_UNITS/degrees/second/second
        # self._max_force = 10 #mass*DIST_UNITS*DIST_UNITS/second/second
        # configure original position:
        self._height = self._ropeLength/2+1

        # physics options:
        self._contactOffset = 2.0*0.01*0.3  ###
        self._physicsMaterialPath:Sdf.Path = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        #print("The path is: ",self._physicsMaterialPath)
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material:UsdPhysics.MaterialAPI = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5) #0.5 Static friction coefficient. Unitless
        material.CreateDynamicFrictionAttr().Set(0.5) #0.5 Dynamic friction coefficient. Unitless.
        material.CreateRestitutionAttr().Set(0)

        self._createRopes(scene)

    def _createCapsule(self, path: Sdf.Path,position:Gf.Vec3f):
        '''
        定义了单个胶囊体的几何和物理特征
        '''
        capsuleGeom:UsdGeom.Capsule = UsdGeom.Capsule.Define(self._stage, path) #定义胶囊体形状，返回一个prim
        capsuleGeom.CreateHeightAttr(2*self._linkHalfLength) #胶囊体高
        capsuleGeom.CreateRadiusAttr(self._linkRadius) #半径
        capsuleGeom.CreateAxisAttr("X") #伸展轴向
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])
        capsuleGeom.AddTranslateOp().Set(position)
        # capsuleGeom.AddOrientOp().Set(Gf.Quatf(1.0))
        # 添加物理属性
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI:UsdPhysics.MassAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(self._density) #kg/m^3
        physxCollisionAPI:PhysxSchema.PhysxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        # Contact offset of a collision shape.
        # Default value -inf means default is picked by the simulation based on the shape extent.
        # Range: [maximum(0, restOffset), inf) Units: distance
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)
    def _createJoint(self, Body0Path:Sdf.Path,Body1Path:Sdf.Path):
        '''
        定义了关节的类型和物理限制属性，还未指定连接实体
        '''        
        jointPath = Body1Path.AppendChild("D6joint")
        # 默认的D6关节，6个自由度均不限制
        joint:UsdPhysics.Joint = UsdPhysics.Joint.Define(self._stage, jointPath) # 定义一个未连接的关节
        # 定义关节连接实体
        joint.CreateBody0Rel().SetTargets([Body0Path])
        joint.CreateBody1Rel().SetTargets([Body1Path])
        # 定义关节位置
        jointX=self._linkHalfLength
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(jointX,0,0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-jointX,0,0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        # timeCode=Sdf.TimeCode()
        # timeCode.GetValue()
        # print(joint.GetBreakForceAttr().Get(timeCode)) # incf
        # locked DOF (lock - low is greater than high)
        d6Prim = joint.GetPrim()
        # 限制X,Y,Z的线位移
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transX")
        limitAPI.CreateLowAttr(1) #Units: degrees or distance depending on trans or rot axis applied to. -inf means not limited in negative direction.
        limitAPI.CreateHighAttr(-1)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transY")
        limitAPI.CreateLowAttr(1)
        limitAPI.CreateHighAttr(-1)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transZ")
        limitAPI.CreateLowAttr(1)
        limitAPI.CreateHighAttr(-1)
        # 限制X方向的转动
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "rotX")
        limitAPI.CreateLowAttr(1)
        limitAPI.CreateHighAttr(-1)

        # Moving DOF:
        dofs = ["rotY", "rotZ"]
        for d in dofs:
            limitAPI:UsdPhysics.LimitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)

            # joint drives for rope dynamics:
            driveAPI: UsdPhysics.DriveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            # maxForce=inf
            # timeCode=Sdf.TimeCode()
            # timeCode.GetValue()
            # print("the max force is ",driveAPI.GetMaxForceAttr().Get(timeCode))
            driveAPI.CreateDampingAttr(self._rope_damping)
            # print("damping:",driveAPI.GetDampingAttr().Get(timeCode))
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)
            # self._arr.append(driveAPI.GetTypeAttr())
            # print("target position",driveAPI.GetTargetPositionAttr().Get(timeCode)) #0.0
            # 尝试添加Y,Z轴的力约束
            # driveAPI.CreateMaxForceAttr(self._max_force)
            if d=="rotY":
                self._jointDriveY.append(driveAPI)
            else :
                self._jointDriveZ.append(driveAPI)

    def _createRopes(self,scene:World.scene):
        """
        实现了pointInstancer和jointInstancer的定义和两者的连接
        """
        linkLength = 2.0 * self._linkHalfLength
        numLinks = int(self._ropeLength / linkLength) #胶囊体个数
        xStart = -numLinks * linkLength * 0.5 #绳子x方向的起点，以原点对称分布
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        for ropeInd in range(self._numRopes):# 循环建立每根绳子
            xformPath:Sdf.Path = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}") 
            xform:UsdGeom.Xform=UsdGeom.Xform.Define(self._stage, xformPath) #建立一根绳子的Xform参考
            
            y = yStart + ropeInd * self._ropeSpacing # 确定某根绳子在y,z方向上的分布
            z = self._height 
            xform.AddTranslateOp().Set(Gf.Vec3f(0, y, z))

            # capsulePath:Sdf.Path = xformPath.AppendChild("capsule")
            cubePathR:Sdf.Path = xformPath.AppendChild("dynamicCubeR")
            cubePathL:Sdf.Path = xformPath.AppendChild("dynamicCubeL")
            # 更新第一个动力块儿的索引、坐标和姿态
            x=xStart
            # print(Gf.Vec3f(x,y,z))
            previousPath = cubePathR
            self._dynamicCubeR:objects.DynamicCuboid=objects.DynamicCuboid(
                                    prim_path="/World/Rope0/dynamicCubeR",
                                    name="DynamicCubeR",
                                    position=Gf.Vec3f(x,y,z),# position in world
                                    scale=np.array([0.02, 0.01, 0.01]),   #Gf.Vec3f(0.045, 0.03, 0.03)  np.array([0.045, 0.03, 0.03])
                                    color=np.array([1.0, 0, 0]))
            scene.add(self._dynamicCubeR)
           
            for linkInd in range(1,numLinks-1):
                CapsulePath:Sdf.Path = xformPath.AppendChild(f"Capsule{linkInd}")
                x = xStart + linkInd * linkLength #等间距更新x
                self._createCapsule(path=CapsulePath,position=Gf.Vec3f(x,0,0))# position in Xform
                if not previousPath.isEmpty:
                        #  创建previousPath和当前Capsule的关节
                        self._createJoint(Body0Path=previousPath, Body1Path=CapsulePath)
                previousPath = CapsulePath


            # 更新最后一个动力块儿的索引、坐标和姿态
            x=xStart + (numLinks-1) * linkLength
            self._dynamicCubeL:objects.DynamicCuboid=objects.DynamicCuboid(
                                    prim_path="/World/Rope0/dynamicCubeL",
                                    name="DynamicCubeL",
                                    position=Gf.Vec3f(x,y,z),
                                    scale=np.array([0.02, 0.01, 0.01]),   #Gf.Vec3f(0.045, 0.03, 0.03)  np.array([0.045, 0.03, 0.03])
                                    color=np.array([1.0, 0, 0]))
            scene.add(self._dynamicCubeL)
            if not previousPath.isEmpty:
                        #  创建previousPath和当前DynamicCube的关节
                        self._createJoint(Body0Path=previousPath, Body1Path=cubePathL)
            # 记录左右动力块儿和绳子中点(球心的位置)
            self._originPosition=np.array(  [[xStart,y,z],
                                            [0,y,z],
                                            [xStart + (numLinks-1) * linkLength,y,z]])
            self._goalR_position = self._originPosition[0]
            self._goalL_position = self._originPosition[2]
            # self._goalR_position=np.array([0.8,1.8,2.6])
            # self._goalL_position=np.array([0,2,2.6])
   
   
    #### 关于控制的函数 #####
    def get_observations(self):
        cubeRposition,_ =self._dynamicCubeR.get_world_pose()
        cubeRvelocity=self._dynamicCubeR.get_linear_velocity()
        cubeLposition,_ =self._dynamicCubeL.get_world_pose()
        cubeLvelocity=self._dynamicCubeL.get_linear_velocity()
        timeCode=Sdf.TimeCode()
        timeCode.GetValue()
        observations={
            self._dynamicCubeR.name:{
                "targetPosition":self._goalR_position,
                "position":cubeRposition,
                "velocity":cubeRvelocity,
                "distance":self._distanceR
            },
            self._dynamicCubeL.name:{
                "targetPosition":self._goalL_position,
                "position":cubeLposition,
                "velocity":cubeLvelocity,
                "distance":self._distanceL
            },
            "driveY":self._jointDriveY,
            "driveZ":self._jointDriveZ,

        }
        return observations
    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        # '''
        # 每次物理走一步都检查是否达到目标位置，如果达到就停止，并且改变颜色
        # '''
        ### 需要添加L的逻辑
        # count=0
        # for Y in self._jointDriveY:
        #     count+=1
        #     timeCode=Sdf.TimeCode()
        #     timeCode.GetValue()
        #     print(count,Y.GetTargetPositionAttr().Get(timeCode))
        # for Z in self._jointDriveZ:
        #     count+=1
        #     timeCode=Sdf.TimeCode()
        #     timeCode.GetValue()
        #     print(count,Z.GetTargetPositionAttr().Get(timeCode))
        cubeRposition,_ =self._dynamicCubeR.get_world_pose()
        cubeLposition,_ =self._dynamicCubeL.get_world_pose()
        # print(self._task_achieved)
        # print("i go there")
        if not self._task_achieved and (np.mean(np.abs(self._goalR_position-cubeRposition))<0.02 or np.mean(np.abs(self._goalL_position-cubeLposition))<0.02) :

            # self._dynamicCube.apply_visual_material().set_color(color=np.array([0, 0, 1.0]))
            self._task_achieved=True
        elif self._task_achieved :
            print("sleep for a second!!")
            time.sleep(0.1)
            self._goalR_position,self._goalL_position=randomTarget(originPoint=self._originPosition[1],radius=self._ropeLength/2)
            print(self._goalR_position,self._goalL_position)
            self._distanceR = np.linalg.norm(cubeRposition-self._goalR_position)
            self._distanceL = np.linalg.norm(cubeLposition-self._goalL_position)
            self._task_achieved=False
        return super().pre_step(time_step_index, simulation_time)
    def post_reset(self):
        '''
        没次reset后，将物块儿颜色复原，并且把任务进度清除
        '''
        self._dynamicCubeR.set_linear_velocity(np.array([0,0,0]))
        self._dynamicCubeL.set_linear_velocity(np.array([0,0,0]))
        # self._dynamicCube.apply_visual_material().set_color(color=np.array([1, 0, 0]))
        self._task_achieved=False
        return 
    def is_done(self):
        return self._task_achieved

    

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # We add the task to the world here
        # world.add_task(MoveRope(name="my_first_task", world=world))
        world.add_task(MoveRope(name="my_first_task"))
        return
    async def setup_post_load(self):
        self._world:World=self.get_world()
        # 关闭物块儿的外力响应
        self._dynamicCubeR:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeR")
        # print(self._dynamicCubeR)
        self._dynamicCubeR.disable_rigid_body_physics()
        self._dynamicCubeL:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeL")
        # print(self._dynamicCubeL)
        self._dynamicCubeL.disable_rigid_body_physics()
        # print("get world ok!!")

        
        self._world.add_physics_callback("move_close_to_target",callback_fn=self.moveCube)
        await self._world.play_async()
        return 
    def moveCube(self,step_size):
        current_observations=self._world.get_observations()
        # 设置rotY,rotZ位置为0
        # count=0
        # for drive in current_observations["driveY"]:
        #     # timeCode=Sdf.TimeCode()
        #     # timeCode.GetValue()
        #     drive.CreateTargetPositionAttr(0)
        #     # count+=1
        #     # print("target Position:",drive.GetTargetPositionAttr().Get(timeCode),count)
        # for drive in current_observations["driveZ"]:
        #     # timeCode=Sdf.TimeCode()
        #     # timeCode.GetValue()
        #     drive.CreateTargetPositionAttr(0)
            # drive.GetTargetPositionAttr().Get(timeCode)
            # count+=1
            # print("target Position:",drive.GetTargetPositionAttr().Get(timeCode),count)
        # print("pointInstancer positions:",current_observations["pointInstancer"]["positions"])
        # print(current_observations["jointForce"]["Y"])
        # 控制右滑块儿
        targetPositionR = current_observations["DynamicCubeR"]["targetPosition"]
        currentPositionR =current_observations["DynamicCubeR"]["position"]
        # print("now positionR is:",currentPositionR)
        # 将速度归一化到定值，方向由位置差确定
        disVectorR=targetPositionR-currentPositionR
        disVectorRNorm=np.linalg.norm(disVectorR)
        # disVectorR0=current_observations["DynamicCubeR"]["distance"]
        # print(disVectorR0)
        # print(f"Now distance is:{disVectorR}")
        if self._world.is_done("my_first_task"):
            currentvelocityR=current_observations["DynamicCubeR"]["velocity"]
            velocityR = -currentvelocityR
            # self._dynamicCubeR.set_world_pose(position=currentPositionR)
        else :
             velocityR =0.5*disVectorR/disVectorRNorm
        self._dynamicCubeR.set_linear_velocity(velocity=velocityR)
        # print("now velocityR is:",velocityR)
    
        # 控制左滑块儿
        targetPositionL = current_observations["DynamicCubeL"]["targetPosition"]
        currentPositionL =current_observations["DynamicCubeL"]["position"]
        # print("now positionL is:",currentPositionL)
        # 将速度归一化到定值，方向由位置差确定
        disVectorL=targetPositionL-currentPositionL
        disVectorLNorm=np.linalg.norm(disVectorL)
        # disVectorL0=current_observations["DynamicCubeL"]["distance"]
        # print(disVectorL0)
        # print(f"Now distance is:{disVectorL}")
        if  self._world.is_done("my_first_task"):
            currentvelocityL=current_observations["DynamicCubeR"]["velocity"]
            velocityL = -currentvelocityL
            # self._dynamicCubeL.set_world_pose(position=currentPositionL)
        else:
            velocityL = 0.5*disVectorL/disVectorLNorm
        self._dynamicCubeL.set_linear_velocity(velocity=velocityL)
        
        # print("now velocityL is:",velocityL)
        # print(self._world.is_done("my_first_task"))
        # 左右滑块儿到位，停止
        # if self._world.is_done("my_first_task") :
        #     self._world.pause()
        return
