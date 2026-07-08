# Build And Test

Use these commands as the source of truth for local development and CI. Run commands from the indicated directory.

## Firmware

From `firmware/`:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Faster local Debug configure without Doxygen post-build generation:

```powershell
cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF
cmake --build --preset Debug
```

Release build:

```powershell
cmake --preset Release
cmake --build --preset Release
```

The firmware build expects Ninja, CMake, and `arm-none-eabi-gcc`.

## GUI

From the repo root:

```powershell
dotnet restore gui\EscGui\src\Esc.Web\Esc.Web.csproj
dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj
dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj
```

Run the GUI:

```powershell
dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj
```

Open `http://localhost:5187`.

Publish the Windows launcher:

```powershell
powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1
```

The expected output is `gui\EscGui\publish\win-x64\Esc.Web.exe` plus the required web assets in the same folder.

## Focused GUI Tests

```powershell
dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~ProtocolTests"
dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~BridgeTests"
```

## CI Parity

The default CI workflow runs:

- repository hygiene checks;
- firmware Debug configure/build with `ESC_BUILD_CODE_DOCS=OFF`;
- GUI restore/build/test on .NET `10.0.x`;
- protocol guard checks;
- SSD document structure checks.
