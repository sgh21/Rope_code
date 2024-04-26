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
class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._rope_damping=0
        self._rope_stiffness=1000
        self._stage =None
        return

    def setup_scene(self):
        
        self._world:World = self.get_world()
        self._world.scene.add_default_ground_plane()
        self._stage = get_current_stage()
        # We add the task to the world here
        # world.add_task(MoveRope(name="my_first_task", world=world))
        # world.add_task(MoveRope(name="my_first_task"))
        return
    def _createCube(self,path:Sdf.Path,position:Gf.Vec3f):
        capsuleGeom:UsdGeom.Capsule = UsdGeom.Capsule.Define(self._stage, path)
        capsuleGeom.CreateHeightAttr(0.02) #胶囊体高
        capsuleGeom.CreateRadiusAttr(0.005) #半径
        capsuleGeom.CreateAxisAttr("X") #伸展轴向
        capsuleGeom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.4, 0.2, 0.1)])
        capsuleGeom.AddTranslateOp().Set(position)
        capsuleGeom.AddOrientOp().Set(Gf.Quatf(1.0))
        # 添加物理属性
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI:UsdPhysics.MassAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(5) #kg/m^3
        physxCollisionAPI:PhysxSchema.PhysxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        # Contact offset of a collision shape.
        # Default value -inf means default is picked by the simulation based on the shape extent.
        # Range: [maximum(0, restOffset), inf) Units: distance
        physxCollisionAPI.CreateContactOffsetAttr().Set(2.0*0.01*0.3)
        self._physicsMaterialPath:Sdf.Path = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        #print("The path is: ",self._physicsMaterialPath)
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material:UsdPhysics.MaterialAPI = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5) #0.5 Static friction coefficient. Unitless
        material.CreateDynamicFrictionAttr().Set(0.5) #0.5 Dynamic friction coefficient. Unitless.
        material.CreateRestitutionAttr().Set(0)
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
        jointX=0.01
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(jointX,0,0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-jointX,0,0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
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
            limitAPI.CreateLowAttr(-30)
            limitAPI.CreateHighAttr(30)
            # joint drives for rope dynamics:
            driveAPI: UsdPhysics.DriveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._rope_damping)
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)
            
    def _createRope(self):
        if self._stage != None:
            prim:PhysxSchema.PhysxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(self._stage.GetPrimAtPath("/World")) #获取当前的世界场景的句柄
            self._defaultPrimPath:Sdf.Path = prim.GetPath() #获取当前的世界场景的prim_path
            xformPath:Sdf.Path = self._defaultPrimPath.AppendChild(f"Rope") 
            xform:UsdGeom.Xform=UsdGeom.Xform.Define(self._stage, xformPath) #建立一根绳子的Xform参考
            xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 1))
            x=0
            prePath=Sdf.Path()
            for i in range(50):
                capsule:Sdf.Path=xformPath.AppendChild(f"capsule{i}")
                self._createCube(path=capsule,position=Gf.Vec3f(x,0,0))
                x+=0.02
                if not prePath.isEmpty:
                    self._createJoint(Body0Path=prePath,Body1Path=capsule)
                prePath=capsule
    async def setup_post_load(self):
        self._world:World=self.get_world()
        # 关闭物块儿的外力响应
        # self._dynamicCubeR:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeR")
        # # print(self._dynamicCubeR)
        # self._dynamicCubeR.disable_rigid_body_physics()
        # self._dynamicCubeL:objects.DynamicCuboid=self._world.scene.get_object("DynamicCubeL")
        # # print(self._dynamicCubeL)
        # self._dynamicCubeL.disable_rigid_body_physics()
        # print("get world ok!!")

        
        # self._world.add_physics_callback("move_close_to_target",callback_fn=self.moveCube)
        self._createRope()
        await self._world.play_async()
        return 