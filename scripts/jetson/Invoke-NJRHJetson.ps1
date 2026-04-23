param(
  [ValidateSet(
    'start-runtime',
    'start-container',
    'restart-container',
    'stop-container',
    'shell',
    'status',
    'start-dashboard',
    'stop-dashboard',
    'dashboard-status',
    'dashboard-logs',
    'open-dashboard'
  )]
  [string]$Action = 'status',
  [string]$JetsonHost = 'nvidia@192.168.31.23',
  [string]$RemoteToolDir = '/tmp/njrh-runtime-tools',
  [string]$WorkspaceHost = '/home/nvidia/workspaces/njrh-v3/workspace1',
  [string]$WorkspaceContainer = '/workspaces/njrh-v3/workspace1',
  [string]$UpstreamWorkspaceHost = '/home/nvidia/workspaces/isaac_ros-dev',
  [string]$UpstreamWorkspaceContainer = '/workspaces/isaac_ros-dev',
  [string]$UpstreamWorkspaceAliasContainer = '/workspaces/isaac_ros-dev-upstream',
  [string]$ContainerName = 'NJRH-car',
  [string]$ImageName = 'njrh-car:latest',
  [string]$BaseImage = 'isaac_ros_dev-aarch64:latest',
  [int]$DashboardPort = 2048
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-JetsonAddress {
  param([string]$HostSpec)
  if ($HostSpec -match '@(.+)$') {
    return $Matches[1]
  }
  return $HostSpec
}

function Invoke-Remote {
  param(
    [string]$HostSpec,
    [string]$Command,
    [switch]$Interactive
  )

  if ($Interactive) {
    & ssh -t $HostSpec $Command
  }
  else {
    & ssh $HostSpec $Command
  }

  if ($LASTEXITCODE -ne 0) {
    throw "Remote command failed with exit code $LASTEXITCODE"
  }
}

function Publish-RemoteTool {
  param(
    [string]$HostSpec,
    [string]$RemoteDir
  )

  $scriptRoot = Split-Path -Parent $PSCommandPath
  $localTool = Join-Path $scriptRoot 'njrh_container.sh'
  if (-not (Test-Path $localTool)) {
    throw "Missing remote tool: $localTool"
  }

  Invoke-Remote -HostSpec $HostSpec -Command "mkdir -p '$RemoteDir'"
  & scp $localTool "${HostSpec}:${RemoteDir}/njrh_container.sh"
  if ($LASTEXITCODE -ne 0) {
    throw "scp failed with exit code $LASTEXITCODE"
  }
  Invoke-Remote -HostSpec $HostSpec -Command "chmod +x '$RemoteDir/njrh_container.sh'"

  $overlayRoot = Join-Path $scriptRoot 'runtime_overlay'
  if (-not (Test-Path $overlayRoot)) {
    throw "Missing runtime overlay: $overlayRoot"
  }
  Invoke-Remote -HostSpec $HostSpec -Command "rm -rf '$RemoteDir/runtime_overlay' && mkdir -p '$RemoteDir'"
  & scp -r $overlayRoot "${HostSpec}:${RemoteDir}/runtime_overlay"
  if ($LASTEXITCODE -ne 0) {
    throw "scp overlay failed with exit code $LASTEXITCODE"
  }
  Invoke-Remote -HostSpec $HostSpec -Command "bash -lc ""chmod +x '$RemoteDir/runtime_overlay/scripts/'*.sh '$RemoteDir/runtime_overlay/scripts/'*.py 2>/dev/null || true"""
}

$jetsonAddress = Get-JetsonAddress -HostSpec $JetsonHost

if ($Action -eq 'open-dashboard') {
  Start-Process "http://${jetsonAddress}:$DashboardPort"
  return
}

Publish-RemoteTool -HostSpec $JetsonHost -RemoteDir $RemoteToolDir

$remoteAction = switch ($Action) {
  'start-runtime' { 'start-runtime' }
  'start-container' { 'start' }
  'restart-container' { 'restart' }
  'stop-container' { 'stop' }
  'shell' { 'shell' }
  'status' { 'status' }
  'start-dashboard' { 'start-dashboard' }
  'stop-dashboard' { 'stop-dashboard' }
  'dashboard-status' { 'dashboard-status' }
  'dashboard-logs' { 'dashboard-logs' }
  default { throw "Unsupported action: $Action" }
}

$remoteEnv = @(
  "NJRH_WORKSPACE_HOST='$WorkspaceHost'",
  "NJRH_WORKSPACE_CONTAINER='$WorkspaceContainer'",
  "NJRH_UPSTREAM_WORKSPACE_HOST='$UpstreamWorkspaceHost'",
  "NJRH_UPSTREAM_WORKSPACE_CONTAINER='$UpstreamWorkspaceContainer'",
  "NJRH_UPSTREAM_WORKSPACE_ALIAS_CONTAINER='$UpstreamWorkspaceAliasContainer'",
  "NJRH_CONTAINER_NAME='$ContainerName'",
  "NJRH_IMAGE_NAME='$ImageName'",
  "NJRH_BASE_IMAGE='$BaseImage'",
  "NJRH_DASHBOARD_PORT='$DashboardPort'",
  "NJRH_DASHBOARD_HOST='$jetsonAddress'"
) -join ' '

$remoteCommand = "$remoteEnv bash '$RemoteToolDir/njrh_container.sh' $remoteAction"

if ($Action -eq 'shell') {
  Invoke-Remote -HostSpec $JetsonHost -Command $remoteCommand -Interactive
}
else {
  Invoke-Remote -HostSpec $JetsonHost -Command $remoteCommand
}
