'''
demo
'''
import carb
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdShade,  Usd #omni.usd.libs
from pxr import UsdPhysics #omni.usd.schema.physics
from pxr import PhysxSchema #omni.usd.schema.physx
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils
import omni.physx.bindings._physx as physx_bindings

class RigidBodyRopesDemo(demo.Base):
    title = "Rigid-Body Ropes"
    category = demo.Categories.JOINTS
    short_description = "Rigid-body capsules with D6 joints (defined as joint instancer) as ropes."
    description = "The capsules are connected by D6 joints with two rotational degrees of freedom unlocked. Joint drives add damping and a bit of stiffness to the rope. Press play (space) to run the simulation."

    kit_settings = {
        physx_bindings.SETTING_MOUSE_PICKING_FORCE: 10.0,
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }
    
    params = { "num_ropes": demo.IntParam(3, 1, 100, 1),
        "rope_length": demo.IntParam(300, 100, 1000, 10)}

    def create(self, stage, num_ropes, rope_length):
        self._stage = stage
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False)

        # configure ropes:
        self._linkHalfLength = 3 # 关节间的间距的一半
        self._linkRadius = 0.5 * self._linkHalfLength #链接的凸起半径
        self._ropeLength = rope_length #绳子的长度
        self._numRopes = num_ropes #绳子的根数
        self._ropeSpacing = 15.0 #绳子间的间距
        self._ropeColor = demo.get_primary_color()
        self._coneAngleLimit = 110 #关节角限制？
        self._rope_damping = 10.0 #阻尼
        self._rope_stiffness = 1.0 #刚度
        # 绳子的初始位姿吧？
        # configure collider capsule:
        self._capsuleZ = 50.0
        self._capsuleHeight = 400.0
        self._capsuleRadius = 20.0 
        self._capsuleRestOffset = -2.0
        self._capsuleColor = demo.get_static_color()

        # physics options:
        self._contactOffset = 2.0
        self._physicsMaterialPath = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)

        self._createRopes()
        self._createColliderCapsule()

    def _createCapsule(self, path: Sdf.Path):
        '''
        定义了单个胶囊体的几何和物理特征
        '''
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path) #定义胶囊体形状，返回一个prim
        capsuleGeom.CreateHeightAttr(self._linkHalfLength) #胶囊体高
        capsuleGeom.CreateRadiusAttr(self._linkRadius) #半径
        capsuleGeom.CreateAxisAttr("X") #伸展轴向
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])
        # 添加物理属性
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(0.00005)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)

    def _createJoint(self, jointPath):
        '''
        定义了关节的类型和物理限制属性，还未指定连接实体
        '''        
        joint = UsdPhysics.Joint.Define(self._stage, jointPath) # 定义一个未连接的关节

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

    def _createColliderCapsule(self):
        capsulePath = self._defaultPrimPath.AppendChild("CapsuleCollider")
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, capsulePath)
        capsuleGeom.CreateHeightAttr(self._capsuleHeight)
        capsuleGeom.CreateRadiusAttr(self._capsuleRadius)
        capsuleGeom.CreateAxisAttr("Y")
        capsuleGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, self._capsuleZ))
        capsuleGeom.CreateDisplayColorAttr().Set([self._capsuleColor])

        # make the capsule high-quality render
        capsulePrim = capsuleGeom.GetPrim()
        capsulePrim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool, True).Set(True)
        capsulePrim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int, True).Set(2)

        UsdPhysics.CollisionAPI.Apply(capsulePrim)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsulePrim)
        physxCollisionAPI.CreateRestOffsetAttr().Set(self._capsuleRestOffset)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsulePrim, self._physicsMaterialPath)

    def _createRopes(self):
        """
        实现了pointInstancer和jointInstancer的定义和两者的连接
        """

        linkLength = 2.0 * self._linkHalfLength - self._linkRadius #link长 2*3-1.5=4.5
        numLinks = int(self._ropeLength / linkLength) #胶囊体个数
        xStart = -numLinks * linkLength * 0.5 #绳子x方向的起点，以原点对称分布
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        for ropeInd in range(self._numRopes):# 循环建立每根绳子
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}") 
            UsdGeom.Scope.Define(self._stage, scopePath) #建立一根绳子的Xform参考
            
            # capsule instancer
            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath) #实例化PointInstancer容器
            
            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath) # 创建胶囊体的几何外形，在此添加DynamicCube的几何外形！！！！
            
            meshIndices = [] #几何外形的索引，[0,0,0,...,0]改为[1,0,0,...,0](在Rel下的index)
            positions = [] # 几何体在参考坐标下的位置
            orientations = [] # 几何体在参考坐标下的姿态
            
            y = yStart + ropeInd * self._ropeSpacing # 确定某根绳子在y,z方向上的分布
            z = self._capsuleZ + self._capsuleRadius + self._linkRadius * 1.4            

            for linkInd in range(numLinks):
                meshIndices.append(0) 
                x = xStart + linkInd * linkLength #等间距更新x
                positions.append(Gf.Vec3f(x, y, z))# 几何体在参考坐标下的位置
                orientations.append(Gf.Quath(1.0))# 几何体在参考坐标下的姿态

            meshList = rboInstancer.GetPrototypesRel()
            # add mesh reference to point instancer
            meshList.AddTarget(capsulePath) #将参考mesh的路径添加到列表，应该在后边添加一个DynamicCube的路径！！！！

            rboInstancer.GetProtoIndicesAttr().Set(meshIndices)
            rboInstancer.GetPositionsAttr().Set(positions)
            rboInstancer.GetOrientationsAttr().Set(orientations) #参数写入到容器
            
            # joint instancer
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
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

            jointX = self._linkHalfLength - 0.5 * self._linkRadius
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