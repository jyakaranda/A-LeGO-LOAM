# A-LeGO-LOAM

## ImageProjection

这 outlier 也太多了（将近 1/3），而且原始数据中有很多点相邻太近（将近一半），不知道是不是雷达的问题

## LaserOdometry

robo_0529.bag 粗略测试，1 step 优化时长 7915ms(1.90ms/frame)(10 iterations)
![](./img/laserOdometry6.png)
![](./img/laserOdometry7.png)

2 step 优化时长 8888ms(2.13ms/frame)，效果更好(surf 5 iterations, corner 10 iterations)
![](./img/laserOdometry3.png)
![](./img/laserOdometry5.png)