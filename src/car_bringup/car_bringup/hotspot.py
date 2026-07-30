import ipaddress
import subprocess
import time


def _run(command):
    """执行无需 Shell 解析的系统命令并返回去除空白的标准输出"""
    return subprocess.run(
        command, check=True, capture_output=True, text=True
    ).stdout.strip()


def _wifi_interface(requested):
    """返回显式指定或 NetworkManager 自动发现的 WiFi 网卡"""
    if requested and requested != "auto":
        return requested
    output = _run(["nmcli", "-t", "-f", "DEVICE,TYPE", "device", "status"])
    for line in output.splitlines():
        fields = line.split(":")
        if len(fields) >= 2 and fields[1] == "wifi" and fields[0]:
            return fields[0]
    raise RuntimeError("未检测到NetworkManager管理的Wi-Fi网卡")


def _connection_exists(name):
    """检查 NetworkManager 中是否已经保存指定热点连接"""
    output = _run(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    return name in output.splitlines()


def _address_ready(interface, address):
    """检查热点网卡是否已经取得期望的 IPv4 网关地址"""
    expected = ipaddress.ip_interface(address).ip
    output = _run(["ip", "-4", "-o", "addr", "show", "dev", interface])
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 4 and fields[2] == "inet":
            if ipaddress.ip_interface(fields[3]).ip == expected:
                return True
    return False


def ensure_hotspot(
    interface,
    connection,
    ssid,
    password,
    address,
    channel,
    timeout,
):
    """创建或修正固定热点，并等待网关地址真正出现在本机"""
    device = _wifi_interface(interface)
    if not _connection_exists(connection):
        _run(
            [
                "nmcli", "connection", "add", "type", "wifi",
                "ifname", device, "con-name", connection, "ssid", ssid,
            ]
        )
    _run(
        [
            "nmcli", "connection", "modify", connection,
            "connection.interface-name", device,
            "connection.autoconnect", "no",
            "802-11-wireless.mode", "ap",
            "802-11-wireless.band", "bg",
            "802-11-wireless.channel", str(channel),
            "ipv4.method", "shared",
            "ipv4.addresses", address,
            "ipv6.method", "disabled",
            "wifi-sec.key-mgmt", "wpa-psk",
            "wifi-sec.psk", password,
        ]
    )
    _run(["nmcli", "connection", "up", connection, "ifname", device])
    deadline = time.monotonic() + max(1.0, timeout)
    while time.monotonic() < deadline:
        if _address_ready(device, address):
            return device
        time.sleep(0.25)
    raise RuntimeError(
        f"热点{connection}已启动，但{address}未在{device}上就绪"
    )
