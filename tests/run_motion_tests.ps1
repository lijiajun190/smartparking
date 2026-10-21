param([string]$CompilerPath = 'tcc')
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path $PSScriptRoot -Parent
$outputDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executable = Join-Path $outputDir 'motion_tests.exe'
& $CompilerPath "-I$PSScriptRoot/mocks" "-I$projectDir/BSP" "-I$projectDir/App" `
    "$PSScriptRoot/test_motion.c" "$projectDir/BSP/bsp_stepper.c" `
    "$projectDir/App/safety.c" "$projectDir/App/parking_fsm.c" "$projectDir/App/billing.c" "$projectDir/App/health_watch.c" -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Motion tests failed' }
$recoveryExecutable = Join-Path $outputDir 'recovery_tests.exe'
& $CompilerPath "-I$projectDir/BSP" "-I$projectDir/App" `
    "$PSScriptRoot/test_recovery.c" "$projectDir/App/parking_store.c" `
    "$projectDir/App/health_watch.c" -o $recoveryExecutable
if ($LASTEXITCODE -ne 0) { throw 'Recovery test compilation failed' }
& $recoveryExecutable
if ($LASTEXITCODE -ne 0) { throw 'Recovery tests failed' }
$applicationExecutable = Join-Path $outputDir 'application_tests.exe'
& $CompilerPath "-I$projectDir/App" "$PSScriptRoot/test_application.c" `
    "$projectDir/App/billing.c" "$projectDir/App/console_command.c" -o $applicationExecutable
if ($LASTEXITCODE -ne 0) { throw 'Application test compilation failed' }
& $applicationExecutable
if ($LASTEXITCODE -ne 0) { throw 'Application tests failed' }
$ioExecutable = Join-Path $outputDir 'io_tests.exe'
& $CompilerPath "-I$PSScriptRoot/io_mocks" "-I$projectDir/BSP" "-I$projectDir/App" `
    "$PSScriptRoot/test_io.c" "$projectDir/BSP/bsp_console.c" "$projectDir/BSP/bsp_sim800.c" `
    "$projectDir/App/parking_service.c" "$projectDir/App/parking_fsm.c" `
    "$projectDir/App/console_command.c" "$projectDir/App/billing.c" -o $ioExecutable
if ($LASTEXITCODE -ne 0) { throw 'IO integration test compilation failed' }
& $ioExecutable
if ($LASTEXITCODE -ne 0) { throw 'IO integration tests failed' }
