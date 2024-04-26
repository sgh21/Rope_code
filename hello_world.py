
from omni.isaac.examples.base_sample import BaseSample
import omni.replicator.core as rep
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
sys.path.append(r'D:/softwares/python3.10.4/Lib/site-packages')
import pandas as pd
sys.path.append(r'D:/Documents/SRT/ov/pkg/isaac_sim-2023.1.1/extension_examples/user_examples')
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
        self._coneAngleLimit = 5 #np.pi/36
        self._density = 50* 27*0.01  # 变小可以看出力的作用变小，但没有看出阻尼和刚度的影响
        # Force or acceleration = stiffness * (targetPosition - position)>damping * (targetVelocity - velocity)
        self._rope_damping = 0#1*0.01*0.01# *0.00001 nothing happened# *10**10 nothing happened # mass*DIST_UNITS*DIST_UNITS/second/degrees.
        self._rope_stiffness =10 #0.1*0.01*0.01# *0.00001 nothing happened# *10**10 nothing happened#: mass*DIST_UNITS*DIST_UNITS/degrees/second/second
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

    def _createCapsule(self, path: Sdf.Path):
        '''
        定义了单个胶囊体的几何和物理特征
        '''
        capsuleGeom:UsdGeom.Capsule = UsdGeom.Capsule.Define(self._stage, path) #定义胶囊体形状，返回一个prim
        capsuleGeom.CreateHeightAttr(2*self._linkHalfLength) #胶囊体高
        capsuleGeom.CreateRadiusAttr(self._linkRadius) #半径
        capsuleGeom.CreateAxisAttr("X") #伸展轴向
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])
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
    def _createJoint(self, jointPath):
        '''
        定义了关节的类型和物理限制属性，还未指定连接实体
        '''        
        # 默认的D6关节，6个自由度均不限制
        joint:UsdPhysics.Joint = UsdPhysics.Joint.Define(self._stage, jointPath) # 定义一个未连接的关节
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
        # # 尝试使用驱动限制x轴方向的旋转
        # driveAPI: UsdPhysics.DriveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, "rotX")
        # driveAPI.CreateTypeAttr("force")
        # driveAPI.CreateDampingAttr(self._rope_damping)
        # driveAPI.CreateStiffnessAttr(self._rope_stiffness)

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
    def _createRopes(self,scene:World.scene):
        """
        实现了pointInstancer和jointInstancer的定义和两者的连接
        """

        # linkLength = 2.0 * self._linkHalfLength - self._linkRadius #link长 2*0.03-0.015=0.045
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
            
            # capsule instancer
            instancerPath:Sdf.Path = xformPath.AppendChild("rigidBodyInstancer")
            self._rboInstancer:UsdGeom.PointInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath) #实例化PointInstancer容器
            # print(self._rboInstancer)

            capsulePath:Sdf.Path = instancerPath.AppendChild("capsule")
            cubePathR:Sdf.Path = instancerPath.AppendChild("dynamicCubeR")
            cubePathL:Sdf.Path = instancerPath.AppendChild("dynamicCubeL")

            
            self._createCapsule(capsulePath) # 创建胶囊体的几何外形，在此添加DynamicCube的几何外形！！！！
            
            meshIndices = [] #几何外形的索引，[0,0,0,...,0]改为[1,0,0,...,0](在Rel下的index)
            positions = [] # 几何体在参考坐标下的位置
            orientations = [] # 几何体在参考坐标下的姿态
            
              
                   
            # 更新第一个动力块儿的索引、坐标和姿态
            meshIndices.append(1)
            x=xStart
            positions.append(Gf.Vec3f(x,0,0))
            orientations.append(Gf.Quath(1.0))
            # print(Gf.Vec3f(x,y,z))
            self._dynamicCubeR:objects.DynamicCuboid=objects.DynamicCuboid(
                                    prim_path="/World/Rope0/rigidBodyInstancer/dynamicCubeR",
                                    name="DynamicCubeR",
                                    position=Gf.Vec3f(0,y,z),
                                    scale=np.array([0.02, 0.01, 0.01]),   #Gf.Vec3f(0.045, 0.03, 0.03)  np.array([0.045, 0.03, 0.03])
                                    color=np.array([1.0, 0, 0]))
            scene.add(self._dynamicCubeR)
            # print(self._dynamicCube)
            # print(self._world.scene.get_object("DynamicCube"))
            # print("create dunamicCube!!!!!!!")
           
            for linkInd in range(1,numLinks-1):
                meshIndices.append(0) 
                x = xStart + linkInd * linkLength #等间距更新x
                positions.append(Gf.Vec3f(x,0,0))# 几何体在参考坐标下的位置
                orientations.append(Gf.Quath(1.0))# 几何体在参考坐标下的姿态


            # 更新最后一个动力块儿的索引、坐标和姿态
            meshIndices.append(2)
            x=xStart + (numLinks-1) * linkLength
            positions.append(Gf.Vec3f(x,0,0))
            orientations.append(Gf.Quath(1.0))
            # print(Gf.Vec3f(x,y,z))
            self._dynamicCubeL:objects.DynamicCuboid=objects.DynamicCuboid(
                                    prim_path="/World/Rope0/rigidBodyInstancer/dynamicCubeL",
                                    name="DynamicCubeL",
                                    position=Gf.Vec3f(0,y,z),
                                    scale=np.array([0.02, 0.01, 0.01]),   #Gf.Vec3f(0.045, 0.03, 0.03)  np.array([0.045, 0.03, 0.03])
                                    color=np.array([1.0, 0, 0]))
            scene.add(self._dynamicCubeL)
            # 记录左右动力块儿和绳子中点(球心的位置)
            self._originPosition=np.array(  [[xStart,y,z],
                                            [0,y,z],
                                            [xStart + numLinks * linkLength,y,z]])
            self._goalR_position = self._originPosition[0]
            self._goalL_position = self._originPosition[2]
            # self._goalR_position=np.array([0.8,1.8,2.6])
            # self._goalL_position=np.array([0,2,2.6])


            ## 参数写入到容器 
            meshList:Usd.Relationship = self._rboInstancer.GetPrototypesRel()
            # add mesh reference to point instancer
            meshList.SetTargets([capsulePath,cubePathR,cubePathL])

            self._rboInstancer.GetProtoIndicesAttr().Set(meshIndices)
            self._rboInstancer.GetPositionsAttr().Set(positions)
            self._rboInstancer.GetOrientationsAttr().Set(orientations) 
            
            # joint instancer
            jointInstancerPath = xformPath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath) #实例化JointInstancer容器
            
            jointPath = jointInstancerPath.AppendChild("joint")
            self._createJoint(jointPath) # 创建未连接的joint参考实例
             
            meshIndices = [] #joint参考的索引(在Rel下的index)
            body0s = []
            body0indices = [] #每个joint关联实体的索引，在instancer里的下标 [0,1,2,...,n-1]
            localPos0 = [] # 和实体0的相对位置
            localRot0 = [] # 和实体0的相对姿态
            body1s = []
            body1indices = [] #每个joint关联实体的索引，在instancer里的下标 [1,2,3,...,n]
            localPos1 = []     # 和实体1的相对位置
            localRot1 = []    # 和实体1的相对姿态  
            body0s.append(instancerPath) #参考实体的路径，这里是个instancer,因此indices应该取得是instancer[indices]
            body1s.append(instancerPath)
            jointX = self._linkHalfLength
            # jointX = self._linkHalfLength - 0.5 * self._linkRadius
            for linkInd in range(numLinks - 1):
                meshIndices.append(0)
                
                body0indices.append(linkInd)
                body1indices.append(linkInd + 1)
                         
                localPos0.append(Gf.Vec3f(jointX, 0, 0)) 
                localPos1.append(Gf.Vec3f(-jointX, 0, 0)) 
                localRot0.append(Gf.Quath(1.0))
                localRot1.append(Gf.Quath(1.0))

            meshList = jointInstancer.GetPhysicsPrototypesRel()
            meshList.AddTarget(jointPath) # 参考关节实例的路径

            jointInstancer.GetPhysicsProtoIndicesAttr().Set(meshIndices)

            jointInstancer.GetPhysicsBody0sRel().SetTargets(body0s) #参考实体的路径，这里是个instancer,因此indices应该取得是instancer[indices]
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(body0indices)
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(localPos0)
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(localRot0)

            jointInstancer.GetPhysicsBody1sRel().SetTargets(body1s)#参考实体的路径，这里是个instancer,因此indices应该取得是instancer[indices]
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(body1indices)
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(localPos1)
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(localRot1)
    #### 关于控制的函数 #####
    def get_observations(self):
        cubeRposition,_ =self._dynamicCubeR.get_world_pose()
        cubeRvelocity=self._dynamicCubeR.get_linear_velocity()
        cubeLposition,_ =self._dynamicCubeL.get_world_pose()
        cubeLvelocity=self._dynamicCubeL.get_linear_velocity()
        # self._pointInstancer:UsdGeom.PointInstancer = self._stage.GetPrimAtPath("/World/Rope0/rigidBodyInstancer")
        timeCode=Sdf.TimeCode()
        timeCode.GetValue()
        instancerPosition = self._rboInstancer.GetPositionsAttr().Get(timeCode)
        observations={
            "pointInstancer":{
                "positions":instancerPosition
            },
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
        }
        return observations
    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        # '''
        # 每次物理走一步都检查是否达到目标位置，如果达到就停止，并且改变颜色
        # '''
        ### 需要添加L的逻辑
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
class Rep(tasks.BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._shot=False
        self._counter = 0
        self._position=None
        self._out_dir="D:/Documents/SoftwareDoc/isaac_sim/repData"
        return 
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self.stage:Usd.Stage = get_current_stage()  
        #print("the stage1 is:",self.stage1)                   # add the rope in the scene
        self._create()
        return
    def _create(self):
        # Create camera
        self._light = rep.create.light(
            position=(0, 1, 1.25), 
            look_at="/World/Rope0/rigidBodyInstancer/capsule",
            light_type="cylinder",
            intensity=10000)
        self._camera = rep.create.camera(
            position=(0, 3, 1.25),
            look_at="/World/Rope0/rigidBodyInstancer/capsule",
        )
        # Attach camera to render product
        self._rp = rep.create.render_product(self._camera, resolution=(1024, 1024))
        # self._rp = rep.create.render_product("/OmniverseKit_Persp", (1024, 1024)) 
        # self._rp = rep.create.render_product("/Replicator/Camera_Xform/Camera", (1024, 1024))
        self._writer = rep.WriterRegistry.get("BasicWriter")
        out_dir = self._out_dir
        print(f"Writing data to {out_dir}")
        self._writer.initialize(
            output_dir=out_dir, 
            rgb=True, 
            # semantic_segmentation=True, 
            pointcloud=True,
            pointcloud_include_unlabelled=False)
        self._writer.attach(self._rp)
        return
    # def pre_step(self, time_step_index: int, simulation_time: float) -> None:
    #     print("i go here!!!")
    #     if self.counter== 60:
    #         print("Got it!")
    #         rep.orchestrator.step_async(rt_subframes=8)
    #         rep.orchestrator.wait_until_complete_async()
    #         self.counter = 1
    #     else:
    #         self.counter += 1

    #     return super().pre_step(time_step_index, simulation_time)
    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        # print("i go here!!!")
        self._counter += 1
        if not self._shot and self._counter == 30:
            # print("Got it!")
            self._shot=True
            # 
            self._counter = 0
        elif self._shot :
            self._shot = False
        return super().pre_step(time_step_index, simulation_time)
    def is_done(self):
        return self._shot
    # def get_observations(self):
    #     return None

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # We add the task to the world here
        # world.add_task(MoveRope(name="my_first_task", world=world))
        world.add_task(MoveRope(name="my_first_task"))
        world.add_task(Rep(name="replicator"))
        capsule=rep.get.prim_at_path("/World/Rope0/rigidBodyInstancer/capsule")
        with capsule:
            rep.modify.semantics([('class', 'Rope')])


        return
     
    async def setup_post_load(self):
        self._world:World=self.get_world()
        self._ouputdata=[]
        self._out_dir="D:/Documents/SoftwareDoc/isaac_sim/outdata.csv"
        # # 关闭物块儿的外力响应
        self._dynamicCubeR:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeR")
        # # print(self._dynamicCubeR)
        # self._dynamicCubeR.disable_rigid_body_physics()
        self._dynamicCubeL:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeL")
        # # print(self._dynamicCubeL)
        # self._dynamicCubeL.disable_rigrep.orchestrator.step_async(rt_subframes=8)
            # rep.orchestrator.wait_until_complete_async()id_body_physics()
        # # print("get world ok!!")
        # self._dataRecord=threading.Thread(target=data_record)
        # self._dataRecord.start()
        
        self._world.add_physics_callback("move_close_to_target",callback_fn=self.moveCube)
        self._world.add_physics_callback("shot",callback_fn=self.shot)
        await self._world.play_async()
        return 
    # async def setup_pre_reset(self):
    #     self._dataRecord.join()
    #     return 
    def data_record(self):
        if self._ouputdata == None:
            return 
        # 将数据列表转换为 DataFrame
        df = pd.DataFrame(self._ouputdata)
        # 将 DataFrame 写入文本文件（CSV 格式）
        df.to_csv(self._out_dir, index=False)
        print(f"Data successfully written to {self._out_dir}.")
        
    def shot(self,step_size):
        time_step=self._world.current_time_step_index
        if self._world.is_done("replicator"):
            print(time_step)
            # print("shot called :",self._world.current_time_step_index)
            rep.orchestrator.step_async(rt_subframes=8)
            rep.orchestrator.wait_until_complete_async()
        current_observations=self._world.get_observations("my_first_task")
        # print("label geted :",self._world.current_time_step_index)
        self._ouputdata.append({"time_step":time_step,"value":current_observations["pointInstancer"]["positions"]})
        if time_step % 100==0:
            self.data_record()
    def moveCube(self,step_size):
       
        current_observations=self._world.get_observations("my_first_task")
        # print("pointInstancer positions:",current_observations["pointInstancer"]["positions"])
        # print(current_observations["jointForce"]["Y"])
        # 控制右滑块儿
        targetPositionR = current_observations["DynamicCubeR"]["targetPosition"]
        currentPositionR =current_observations["DynamicCubeR"]["position"]
        print("now positionR is:",currentPositionR)
        # 将速度归一化到定值，方向由位置差确定
        disVectorR=targetPositionR-currentPositionR
        disVectorRNorm=np.linalg.norm(disVectorR)
        # disVectorR0=current_observations["DynamicCubeR"]["distance"]
        # print(disVectorR0)
        # print(f"Now distance is:{disVectorR}")
        if self._world.is_done("my_first_task"):
            currentvelocityR=current_observations["DynamicCubeR"]["velocity"]
            velocityR = -currentvelocityR
        else :
             velocityR =0.5*disVectorR/disVectorRNorm
        self._dynamicCubeR.set_linear_velocity(velocity=velocityR)
        # print("now velocityR is:",velocityR)
    
        # 控制左滑块儿
        targetPositionL = current_observations["DynamicCubeL"]["targetPosition"]
        currentPositionL =current_observations["DynamicCubeL"]["position"]
        print("now positionL is:",currentPositionL)
        # 将速度归一化到定值，方向由位置差确定
        disVectorL=targetPositionL-currentPositionL
        disVectorLNorm=np.linalg.norm(disVectorL)
        # disVectorL0=current_observations["DynamicCubeL"]["distance"]
        # print(disVectorL0)
        # print(f"Now distance is:{disVectorL}")
        if  self._world.is_done("my_first_task"):
            currentvelocityL=current_observations["DynamicCubeR"]["velocity"]
            velocityL = -currentvelocityL
        else:
            velocityL = 0.5*disVectorL/disVectorLNorm
        self._dynamicCubeL.set_linear_velocity(velocity=velocityL)
        
        # print("now velocityL is:",velocityL)
        # 左右滑块儿到位，停止
        # if self._world.is_done("my_first_task") :
        #     self._world.pause()
        return
