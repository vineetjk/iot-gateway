/**
 * build_config.h
 * =========================================================================
 * Production vs Debug build configuration.
 *
 * For production builds:
 *   1. Define PRODUCTION=1 in project preprocessor symbols
 *      (Project → Properties → C/C++ Build → Settings → Preprocessor)
 *   2. Set optimization to -Os
 *   3. Rebuild all
 *
 * Or just change the default below.
 * =========================================================================
 */
#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

/* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * BUILD MODE: Set via preprocessor define or change default here
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
#ifndef PRODUCTION
#define PRODUCTION  0    /* 0 = Debug (logs enabled), 1 = Production (logs stripped) */
#endif

/* ── Feature flags derived from build mode ── */
#if PRODUCTION
  #define DEBUG_LOG_ENABLED   0
  #define AT_LOG_ENABLED      0    /* AT command TX/RX logging */
#else
  #define DEBUG_LOG_ENABLED   1
  #define AT_LOG_ENABLED      1
#endif

#endif /* BUILD_CONFIG_H */
