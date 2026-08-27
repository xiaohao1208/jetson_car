"""HTTP 请求中与 ROS 无关的严格数据校验。"""

import math


class RequestValueError(ValueError):
    """请求字段缺失、类型错误或包含非有限数值。"""


def boolean_field(body, name, *, default=None):
    """只接受 JSON 布尔值，避免字符串参与 Python 真值转换。"""

    value = body.get(name, default)
    if not isinstance(value, bool):
        raise RequestValueError(f"{name}必须是JSON布尔值")
    return value


def finite_float(body, name, *, default=None):
    """读取一个有限浮点字段；布尔值不能冒充数字。"""

    value = body.get(name, default)
    if isinstance(value, bool):
        raise RequestValueError(f"{name}必须是有限数值")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RequestValueError(f"{name}必须是有限数值") from error
    if not math.isfinite(result):
        raise RequestValueError(f"{name}必须是有限数值")
    return result
