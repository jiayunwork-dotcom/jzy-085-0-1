"""统一错误定义：机器可读的错误码 + 给人看的说明。"""

# 模型非法类错误
DUPLICATE_NODE_ID = "DUPLICATE_NODE_ID"          # 节点编号重复
DUPLICATE_ELEMENT_ID = "DUPLICATE_ELEMENT_ID"    # 杆件编号重复
DUPLICATE_SUPPORT = "DUPLICATE_SUPPORT"          # 同一节点重复定义支座
UNKNOWN_NODE = "UNKNOWN_NODE"                    # 引用了不存在的节点
UNKNOWN_ELEMENT = "UNKNOWN_ELEMENT"              # 引用了不存在的杆件
ZERO_LENGTH_ELEMENT = "ZERO_LENGTH_ELEMENT"      # 杆长为零
INVALID_SECTION = "INVALID_SECTION"              # 弹性模量/截面量非正或非有限
INSUFFICIENT_CONSTRAINTS = "INSUFFICIENT_CONSTRAINTS"  # 约束不足以消除刚体运动
DISCONNECTED_STRUCTURE = "DISCONNECTED_STRUCTURE"      # 结构分成互不相连的几块
EMPTY_MODEL = "EMPTY_MODEL"                      # 模型缺少节点或杆件
INVALID_SCHEMA = "INVALID_SCHEMA"                # 请求体不符合数据格式

# 求解类错误
SINGULAR_STIFFNESS = "SINGULAR_STIFFNESS"        # 总刚奇异/病态（机构、约束不足等）


class ModelError(Exception):
    """模型非法或无法求解时抛出，携带机器可读错误码与可读说明。"""

    def __init__(self, code: str, message: str):
        self.code = code
        self.message = message
        super().__init__(f"{code}: {message}")
