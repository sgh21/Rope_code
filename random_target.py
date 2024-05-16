import numpy as np
def generate_random_point_in_ellipsoid(center, params=[1, 1, 1]):
    x0, y0, z0 = center
    a, b, c = params
    while True:
        # Generate random points within the bounding box of the ellipsoid
        x = np.random.uniform(x0 - a, x0 + a)
        y = np.random.uniform(y0 - b, y0 + b)
        z = np.random.uniform(z0 - c, z0 + c)
        
        # Check if the point is inside the ellipsoid
        if ((x - x0) ** 2) / a ** 2 + ((y - y0) ** 2) / b ** 2 + ((z - z0) ** 2) / c ** 2 <= 1:
            return (x, y, z)

def generate_two_points_in_ellipsoid(center, params=[1, 1, 1],d=0.002):
    point1 = generate_random_point_in_ellipsoid(center, params)
    while True:
        point2 = generate_random_point_in_ellipsoid(center, params)
        # distance = np.linalg.norm(np.array(point1) - np.array(point2))
        distance=np.min(np.abs(np.array(point1)-np.array(point2)))
        
        # Ensure the points are not the same and the distance is at least d
        if distance >= d:
            return point1, point2
def randomTarget(originPoint=np.array([0,0,1.375]),radius=0.375):
  key=np.random.randint(0,100)
  if key<=80:
    # 随机球面上的两个点
    # 生成两个随机数组，保证R_x<0 L_x>0
    randomArrayR = np.random.uniform(low=-100,high=100,size=(3,))
    randomArrayR[0]=-np.abs(randomArrayR[0])
    randomArrayL = np.random.uniform(low=-100,high=100,size=(3,))
    randomArrayL[0]=np.abs(randomArrayL[0])

    # 将两个数组归一化到半径为r的球空间
    NormR=np.linalg.norm(randomArrayR)
    NormL=np.linalg.norm(randomArrayL)
    # print(f"NormL:{NormL},NormR:{NormR}")

    targetR=randomArrayR/NormR*radius
    targetL=randomArrayL/NormL*radius
    # print(f"NormTargetL:{np.linalg.norm(targetL)},NormTargetR:{np.linalg.norm(targetR)}")
    targetR=targetR+originPoint
    targetL=targetL+originPoint
  else :
    x=np.random.uniform(low=-0.375,high=0.375)
    z1=np.random.uniform(low=1.375,high=1.5)
    z2=np.random.uniform(low=1.375,high=1.5)
    targetR=np.array([x,0.015,z1])
    targetL=np.array([-x,-0.015,z2])
  # targetR,targetL=generate_two_points_in_ellipsoid(originPoint,[radius,radius/2,radius/2])
  return targetR,targetL


if __name__=="__main__":
  print(randomTarget())
  # testnp=np.array([1])
  pass
