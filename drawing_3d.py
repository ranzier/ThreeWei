import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 假设输入端点格式是[x1,y1,z1,x2,y2,z2],下例为两条直线的坐标端点
# [[x1,y1,z1,x2,y2,z2], # 直线1
#  [a1,b1,c1,a2,b2,c2]] # 直线2

# 在此处输入坐标即可
line_coord = [
[0,0,0,9,9,9],
]

# 坐标系画直线
for i in range(0,len(line_coord)):
    a = line_coord[i]
    x = [a[0],a[3]]
    y = [a[1],a[4]]
    z = [a[2],a[5]]
    ax.plot(x,y,z)

# 设置坐标轴范围
ax.invert_yaxis()
ax.invert_zaxis()
ax.set_aspect('equal')
plt.show()