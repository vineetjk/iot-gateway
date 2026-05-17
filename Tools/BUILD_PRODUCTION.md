# Production Build Setup

## Steps in STM32CubeIDE:

### 1. Create Release Configuration
- Project → Build Configurations → Manage → New
- Name: `Release`, copy from `Debug`
- Click OK

### 2. Set Optimization (Release config)
- Project → Properties → C/C++ Build → Settings
- **Make sure "Release" is selected** in Configuration dropdown
- MCU GCC Compiler → Optimization → **Optimization level: -Os (Optimize for size)**
- MCU GCC Compiler → Optimization → Check: **-ffunction-sections**, **-fdata-sections**

### 3. Add PRODUCTION Define
- MCU GCC Compiler → Preprocessor → Add defined symbol: **PRODUCTION=1**
- Remove **DEBUG** from the list (if present)

### 4. Enable LTO (optional, extra 2-5KB saving)
- MCU GCC Compiler → Miscellaneous → Other flags: add **-flto**
- MCU GCC Linker → Miscellaneous → Other flags: add **-flto**

### 5. Linker Garbage Collection (should already be set)
- MCU GCC Linker → General → Check: **Discard unused sections (-Wl,--gc-sections)**

### 6. Build
- Set active configuration to Release (Project → Build Configurations → Set Active → Release)
- Project → Clean, then Build All
- Output: `Release/GATEWAY.bin`

## Expected Size Reduction:
| Build | Flash | RAM |
|-------|-------|-----|
| Debug (-O0, logs ON) | ~96 KB | ~19 KB |
| Release (-Os, logs OFF) | ~50-55 KB | ~17 KB |

## Switching Between Builds:
- Debug: for development/testing with full logging
- Release: for production deployment (smaller, faster, no debug output)

The CLI still works in production (commands like `show config`, `reboot`, etc.)
but Debug_Printf/Debug_Print calls are compiled out (zero flash cost).
