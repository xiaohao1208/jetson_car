param(
    [string]$Root = $PSScriptRoot
)

$ErrorActionPreference = "Stop"

# 交接包完整摘要清单
$ChecksumFile = Join-Path $Root "SHA256SUMS"
if (-not (Test-Path -LiteralPath $ChecksumFile -PathType Leaf)) {
    throw "缺少 SHA256SUMS"
}

# 当前已经校验的文件数量
$VerifiedCount = 0
foreach ($Line in Get-Content -LiteralPath $ChecksumFile) {
    if ($Line -notmatch '^([0-9a-f]{64})  \./(.+)$') {
        throw "SHA256SUMS 格式无效，内容=$Line"
    }

    # 清单记录的期望摘要
    $ExpectedHash = $Matches[1]
    # 转换为 Windows 分隔符后的相对路径
    $RelativePath = $Matches[2].Replace('/', [IO.Path]::DirectorySeparatorChar)
    # 当前待校验文件的完整路径
    $FilePath = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $FilePath -PathType Leaf)) {
        throw "交接文件缺失，文件=$RelativePath"
    }

    # Windows 重新计算的实际摘要
    $ActualHash = (Get-FileHash -LiteralPath $FilePath -Algorithm SHA256).Hash.ToLower()
    if ($ActualHash -ne $ExpectedHash) {
        throw "交接文件摘要不一致，文件=$RelativePath"
    }
    $VerifiedCount += 1
}

Write-Host "交接包内部校验通过，文件数=$VerifiedCount"
