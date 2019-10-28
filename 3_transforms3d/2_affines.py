from transforms3d.affines import compose,decompose,decompose44

# 平移矩阵
T = [20,20,30]
# 旋转矩阵
R = [[1,0,0],[0,1,0],[0,0,1]]
# 缩放矩阵
Z = [2.0,3.0,4.0]

A = compose(T,R,Z)
print("组成的变换矩阵是：")
print(A)

T1,R1,Z1,S1 = decompose(A)
print("组成的平移矩阵是：")
print(T1)
print("组成的旋转矩阵是：")
print(R1)
print("组成的缩放矩阵是：")
print(Z1)
print("组成的。。矩阵是：")
print(S1)




