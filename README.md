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

~~不知道为啥，occluded points 就是比 lego 多，也是无语了，而且真要是标记为 occluded 的话 corner feature 就太少了，匹配起来效果很差。~~无语了，原来是因为 cloud_msg 里的 segmentedCloudColInd 是 uint，进行算术运算再赋给 int 出了问题，然后 col_diff 就 gg 了。

## LaserMapping

注意回环后 map2odom 要及时更新