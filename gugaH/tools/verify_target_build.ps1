$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$sdkRoot = "C:\ti\mspm0_sdk_2_10_00_04"
$sysconfig = "C:\ti\sysconfig_1.26.2\sysconfig_cli.bat"
$compiler = "C:\ti\ccs2100\ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS\bin\tiarmclang.exe"
$tempRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
$work = Join-Path $tempRoot ("gugah_build_" + [Guid]::NewGuid().ToString("N"))
$generated = Join-Path $work "syscfg"
$objects = Join-Path $work "obj"

New-Item -ItemType Directory -Path $generated, $objects | Out-Null

try {
    & $sysconfig `
        --product (Join-Path $sdkRoot ".metadata\product.json") `
        --script (Join-Path $projectRoot "empty_cpp.syscfg") `
        --output $generated `
        --compiler ticlang `
        --treatWarningsAsErrors
    if ($LASTEXITCODE -ne 0) { throw "SysConfig generation failed" }

    $common = @(
        "@$(Join-Path $generated 'device.opt')",
        "-march=thumbv6m", "-mcpu=cortex-m0plus",
        "-mfloat-abi=soft", "-mlittle-endian", "-mthumb",
        "-O2", "-Wall", "-Werror",
        "-I$projectRoot", "-I$generated",
        "-I$(Join-Path $sdkRoot 'source\third_party\CMSIS\Core\Include')",
        "-I$(Join-Path $sdkRoot 'source')"
    )

    $cppSources = Get-ChildItem $projectRoot -Recurse -Filter *.cpp
    foreach ($source in $cppSources) {
        $relative = $source.FullName.Substring($projectRoot.Length + 1)
        $name = ($relative -replace "[\\/]", "_") + ".o"
        & $compiler -c @common -o (Join-Path $objects $name) $source.FullName
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $relative" }
    }

    & $compiler -c @common `
        -o (Join-Path $objects "ti_msp_dl_config.o") `
        (Join-Path $generated "ti_msp_dl_config.c")
    if ($LASTEXITCODE -ne 0) { throw "Generated config compile failed" }

    & $compiler -c @common `
        -o (Join-Path $objects "startup.o") `
        (Join-Path $sdkRoot "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g351x_ticlang.c")
    if ($LASTEXITCODE -ne 0) { throw "Startup compile failed" }

    $output = Join-Path $work "gugaH.out"
    $linkArgs = @(
        "@$(Join-Path $generated 'device.opt')",
        "-march=thumbv6m", "-mcpu=cortex-m0plus",
        "-mfloat-abi=soft", "-mlittle-endian", "-mthumb",
        "-O2", "-Wall", "-Werror",
        "-Wl,-m$(Join-Path $work 'gugaH.map')",
        "-Wl,-i$(Join-Path $sdkRoot 'source')",
        "-Wl,-i$generated",
        "-Wl,-iC:\ti\ccs2100\ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS\lib",
        "-Wl,--diag_wrap=off", "-Wl,--display_error_number",
        "-Wl,--warn_sections", "-Wl,--rom_model",
        "-o", $output
    )
    $linkArgs += (Get-ChildItem $objects -Filter *.o |
        ForEach-Object { $_.FullName })
    $linkArgs += @(
        "-Wl,-l$(Join-Path $generated 'device_linker.cmd')",
        "-Wl,-l$(Join-Path $generated 'device.cmd.genlibs')",
        "-Wl,-llibc.a"
    )
    & $compiler @linkArgs
    if ($LASTEXITCODE -ne 0) { throw "Link failed" }

    $size = (Get-Item $output).Length
    Write-Host "PASS: $($cppSources.Count) C++ sources, -O2 -Wall -Werror, $size bytes"
}
finally {
    $resolvedWork = [IO.Path]::GetFullPath($work)
    if ($resolvedWork.StartsWith($tempRoot, [StringComparison]::OrdinalIgnoreCase)) {
        Remove-Item -LiteralPath $resolvedWork -Recurse -Force -ErrorAction SilentlyContinue
    }
}
