import numpy as np

def randomTarget(originPoint=np.array([0,0,1.5]),radius=1.5):
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
  return targetR,targetL


if __name__=="__main__":
  print(randomTarget())
  # testnp=np.array([1])
  pass
