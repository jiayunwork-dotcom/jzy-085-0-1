"""平面刚架静力核算服务。

模块划分：
- transform    方向余弦与局部/整体坐标变换
- element      单元局部刚度矩阵与杆上荷载的等效节点荷载
- assembly     总刚组装、荷载向量组装与约束施加（行列缩减法）
- solver       线性方程组求解与奇异检测
- postprocess  杆端内力与支座反力回代
- validation   输入模型的语义校验
- service      把上述步骤串起来的求解流程
- main         FastAPI 接入层（唯一的 HTTP 入口）
- schemas      请求/响应数据模型
- errors       机器可读错误码与异常定义
"""
