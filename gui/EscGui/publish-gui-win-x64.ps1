Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectPath = Join-Path $scriptDir "src\Esc.Web\Esc.Web.csproj"
$publishProfile = "WinX64Launcher"
$outputDir = Join-Path $scriptDir "publish\win-x64"
$exePath = Join-Path $outputDir "Esc.Web.exe"

Write-Host "Publicando GUI en modo Release para win-x64..."
dotnet publish $projectPath -p:PublishProfile=$publishProfile

if (-not (Test-Path $exePath)) {
    throw "No se encontro Esc.Web.exe en $outputDir"
}

Write-Host ""
Write-Host "Listo. Ejecutable generado en:"
Write-Host "  $exePath"
Write-Host ""
Write-Host "Manten la carpeta publish\win-x64 completa junto al ejecutable."
Write-Host "Al abrirlo se levantara el servidor y se intentara abrir el navegador en http://localhost:5187."
