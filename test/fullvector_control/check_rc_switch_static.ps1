$ErrorActionPreference = "Stop"

$root = Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")

function Read-RepoFile($relativePath) {
    Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root $relativePath)
}

function Assert-Contains($text, $pattern, $message) {
    if ($text -notmatch $pattern) {
        throw $message
    }
}

function Assert-FileExists($relativePath, $message) {
    if (-not (Test-Path -LiteralPath (Join-Path $root $relativePath))) {
        throw $message
    }
}

$msgList = Read-RepoFile "msg\CMakeLists.txt"
$fvHeader = Read-RepoFile "src\modules\fullvector_control\fullvector_control.hpp"
$fvSource = Read-RepoFile "src\modules\fullvector_control\fullvector_control.cpp"
$fvYaml = Read-RepoFile "src\modules\fullvector_control\module.yaml"
$caHeader = Read-RepoFile "src\modules\control_allocator\ControlAllocator.hpp"
$caSource = Read-RepoFile "src\modules\control_allocator\ControlAllocator.cpp"

Assert-FileExists "msg\FullvectorControlStatus.msg" "missing FullvectorControlStatus uORB message"
Assert-Contains $msgList "FullvectorControlStatus\.msg" "FullvectorControlStatus.msg is not registered in msg/CMakeLists.txt"

Assert-Contains $fvHeader "fullvector_control_status\.h" "fullvector_control does not include the status topic header"
Assert-Contains $fvHeader "_fullvector_control_status_pub" "fullvector_control does not publish ownership status"
Assert-Contains $fvHeader "FV_RC_SW_EN" "fullvector_control missing RC switch enable parameter"
Assert-Contains $fvHeader "FV_RC_SW_CH" "fullvector_control missing RC switch channel parameter"
Assert-Contains $fvHeader "FV_RC_SW_THR" "fullvector_control missing RC switch threshold parameter"
Assert-Contains $fvHeader "FV_RC_SW_REV" "fullvector_control missing RC switch reverse parameter"
Assert-Contains $fvSource "publishFullvectorControlStatus" "fullvector_control missing status publish helper"
Assert-Contains $fvSource "evaluateNativeControllerRequest" "fullvector_control missing RC switch evaluation helper"
Assert-Contains $fvSource "publishNeutralTiltServos" "fullvector_control missing neutral servo handoff helper"
Assert-Contains $fvSource "native_requested" "fullvector_control never marks native controller requests"
Assert-Contains $fvSource "fullvector_active" "fullvector_control never marks active ownership"
Assert-Contains $fvSource "\u8235\u673a" "fullvector_control lacks Chinese servo safety comments"
Assert-Contains $fvYaml "FV_RC_SW_EN" "module.yaml missing FV_RC_SW_EN"
Assert-Contains $fvYaml "FV_RC_SW_CH" "module.yaml missing FV_RC_SW_CH"
Assert-Contains $fvYaml "FV_RC_SW_THR" "module.yaml missing FV_RC_SW_THR"
Assert-Contains $fvYaml "FV_RC_SW_REV" "module.yaml missing FV_RC_SW_REV"

Assert-Contains $caHeader "fullvector_control_status\.h" "control_allocator does not include the status topic header"
Assert-Contains $caHeader "_fullvector_control_status_sub" "control_allocator does not subscribe to fullvector status"
Assert-Contains $caSource "fullvector_status_fresh" "control_allocator missing freshness guard"
Assert-Contains $caSource "fullvector_control_status.fullvector_active" "control_allocator does not use status topic ownership"
Assert-Contains $caSource "\u9065\u63a7" "control_allocator lacks Chinese RC switch arbitration comments"

Write-Host "PASS fullvector RC switch static checks"
