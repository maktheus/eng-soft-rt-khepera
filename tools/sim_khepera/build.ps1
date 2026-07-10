param(
    [string]$OutDir = "tools/sim_khepera/out_gl",
    [string]$Worlds = "tools/sim_khepera/worlds_1000.json",
    [switch]$Interactive
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$src = Join-Path $PSScriptRoot "khepera_gl.c"
$exe = Join-Path $PSScriptRoot "khepera_gl.exe"
$localGccDir = Join-Path $root ".tools\c-compiler\Library\bin"
$localGcc = Join-Path $localGccDir "x86_64-w64-mingw32-gcc.exe"

Push-Location $root
try {
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

    $gcc = Get-Command gcc -ErrorAction SilentlyContinue
    $clang = Get-Command clang -ErrorAction SilentlyContinue
    $cl = Get-Command cl -ErrorAction SilentlyContinue

    if (Test-Path $localGcc) {
        $env:Path = "$localGccDir;$env:Path"
        & $localGcc -O2 -Wall -Wextra $src -lopengl32 -lgdi32 -luser32 -lm -o $exe
    } elseif ($gcc) {
        & $gcc.Source -O2 -Wall -Wextra $src -lopengl32 -lgdi32 -luser32 -lm -o $exe
    } elseif ($clang) {
        & $clang.Source -O2 -Wall -Wextra $src -lopengl32 -lgdi32 -luser32 -o $exe
    } elseif ($cl) {
        & $cl.Source /nologo /O2 /W4 $src /Fe:$exe opengl32.lib gdi32.lib user32.lib
    } else {
        throw "Nenhum compilador C encontrado. Instale gcc/clang/cl ou rode: conda create -y -p .\.tools\c-compiler -c conda-forge m2w64-gcc"
    }

    if ($Interactive) {
        Start-Process -FilePath $exe -ArgumentList @($Worlds) -WorkingDirectory $root
    } else {
        & $exe --batch $OutDir $Worlds
    }
} finally {
    Pop-Location
}
