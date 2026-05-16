/**
 * debug_cli.c
 * =========================================================================
 */
#include "debug_cli.h"
#include "main.h"
#include "config.h"
#include "modem_at.h"
#include "bootloader.h"
#include "rgb_led.h"
#include "w25q_spi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

static UART_HandleTypeDef *s_huart;
volatile uint8_t g_log_enabled = 0;  /* background logs OFF by default */
static volatile uint8_t s_in_dispatch = 0;

/* Ring buffer */
static volatile uint8_t  s_ring[CLI_RX_BUF_SIZE];
static volatile uint16_t s_head = 0, s_tail = 0;
static uint8_t           s_rxbyte;

/* Command assembly */
static char    s_cmd[CLI_CMD_BUF_SIZE];
static uint8_t s_len = 0;

#define BS  0x08
#define DEL 0x7F

/* ── Output ──────────────────────────────────────────────────────── */
void Debug_Print(const char *msg)
{
    if (!s_huart || !msg) return;
    if (!g_log_enabled && !s_in_dispatch) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)msg, strlen(msg), 500);
}

void Debug_Printf(const char *fmt, ...)
{
    if (!s_huart) return;
    if (!g_log_enabled && !s_in_dispatch) return;
    char buf[256];
    va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, strlen(buf), 500);
}

/* Direct print — always outputs regardless of log flag (for CLI responses) */
static void cli_print(const char *msg)
{
    if (!s_huart || !msg) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)msg, strlen(msg), 500);
}

static void cli_printf(const char *fmt, ...)
{
    char buf[256];
    va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    cli_print(buf);
}

static void echo(char c)
{ HAL_UART_Transmit(s_huart, (uint8_t *)&c, 1, 10); }

/* ── Helpers ─────────────────────────────────────────────────────── */
static void trim(char *s)
{
    uint8_t l = strlen(s);
    while (l && isspace((unsigned char)s[l-1])) s[--l] = 0;
    uint8_t st = 0;
    while (st < l && isspace((unsigned char)s[st])) st++;
    if (st) memmove(s, s+st, l-st+1);
}

static int ci_eq(const char *a, const char *b)
{
    while (*a && *b)
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return 0;
    return *a == 0 && *b == 0;
}

/* ── Ring buffer ─────────────────────────────────────────────────── */
void CLI_RxCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == s_huart->Instance) {
        uint16_t n = (s_head + 1U) % CLI_RX_BUF_SIZE;
        if (n != s_tail) { s_ring[s_head] = s_rxbyte; s_head = n; }
        HAL_UART_Receive_IT(s_huart, &s_rxbyte, 1);
    }
}

static int rbpop(uint8_t *b)
{
    if (s_tail == s_head) return 0;
    *b = s_ring[s_tail]; s_tail = (s_tail+1U)%CLI_RX_BUF_SIZE; return 1;
}

/* ── Init ────────────────────────────────────────────────────────── */
void CLI_Init(UART_HandleTypeDef *h)
{
    s_huart = h;
    HAL_UART_Receive_IT(s_huart, &s_rxbyte, 1);
    cli_print("\r\n\r\n");
    cli_print("========================================\r\n");
    cli_printf("  Delta IoT Gateway  FW %s\r\n", FW_VERSION_STR);
    cli_print("  Type 'help' | 'log on' for debug\r\n");
    cli_print("========================================\r\n\r\n> ");
}

/* ══════════════════════════════════════════════════════════════════
   COMMAND HANDLERS
   ══════════════════════════════════════════════════════════════════ */

static void cmd_help(void)
{
    Debug_Print(
    "\r\n┌──────────────────────────────────────────────────────────────┐\r\n"
    "│  DISPLAY                                                      │\r\n"
    "│    help | show config | show regs                             │\r\n"
    "│  DEVICE                                                       │\r\n"
    "│    set device_id <n>     set device_name <str>                │\r\n"
    "│  MODBUS                                                       │\r\n"
    "│    set modbus_baud <n>        9600/19200/38400/115200         │\r\n"
    "│    set modbus_parity <0|1|2>  0=None 1=Even 2=Odd            │\r\n"
    "│    set modbus_stopbits <1|2>                                  │\r\n"
    "│    set modbus_timeout <ms>    set poll_interval <ms>          │\r\n"
    "│  REGISTERS                                                    │\r\n"
    "│    reg add <slave> <addr> <fc> <type> <scale> <tag> [alarm]    │\r\n"
    "│         type: u16 i16 u32 i32 f32                            │\r\n"
    "│         example: reg add 1 0x0000 3 u16 0.1 freq_hz          │\r\n"
    "│         example: reg add 1 0x0010 3 u16 1.0 ovr_vtg alarm    │\r\n"
    "│    reg del <idx>     reg enable <idx> <0|1>     reg list      │\r\n"
    "│    reg alarm <idx> <0|1>   — mark register as alarm           │\r\n"
    "│  ALARM BITS                                                   │\r\n"
    "│    alarm add <reg_idx> <bit> <name>   alarm del <idx>         │\r\n"
    "│    alarm list                                                 │\r\n"
    "│         example: alarm add 5 0 OV  (reg[5] bit0 = OV)        │\r\n"
    "│  TELEMETRY                                                    │\r\n"
    "│    set publish_interval <ms>  set delta_mode <0|1>            │\r\n"
    "│    set delta_threshold <n>                                    │\r\n"
    "│  NETWORK                                                      │\r\n"
    "│    set modem_type <0|1>   0=SIMCom A7670C  1=Quectel EC200U  │\r\n"
    "│    set apn <str>          set broker <host>                   │\r\n"
    "│    set broker_port <n>    set mqtt_client <str>               │\r\n"
    "│    set mqtt_user <str>    set mqtt_pass <str>                 │\r\n"
    "│    set mqtt_topic <str>   set alarm_topic <str>                │\r\n"
    "│    set mqtt_qos <0-2>                                        │\r\n"
    "│  OTA                                                          │\r\n"
    "│    set ota_enabled <0|1>  set ota_url <url>                   │\r\n"
    "│  SPI FLASH                                                     │\r\n"
    "│    flash id       flash test       flash dump <addr> [len]    │\r\n"
    "│  SYSTEM                                                       │\r\n"
    "│    save   reset   reboot   fwupdate   led <0-255>             │\r\n"
    "└──────────────────────────────────────────────────────────────┘\r\n\r\n");
}

static void cmd_set(char *args)
{
    GatewayConfig_t *c = Config_GetMutable();
    char key[32]={0}; uint8_t ki=0;
    char *p = args;
    while (*p && !isspace((unsigned char)*p) && ki<31) key[ki++]=*p++;
    while (*p &&  isspace((unsigned char)*p)) p++;
    char *val = p;
    if (!*val) { Debug_Print("[ERR] Missing value\r\n"); return; }

    if      (ci_eq(key,"device_id"))        { c->device_id=(uint16_t)atoi(val); Debug_Printf("[OK] device_id=%u\r\n",c->device_id); }
    else if (ci_eq(key,"device_name"))      { strncpy(c->device_name,val,31); Debug_Printf("[OK] device_name=%s\r\n",c->device_name); }
    else if (ci_eq(key,"modbus_baud"))      {
        uint32_t b=(uint32_t)atol(val);
        if(b==9600||b==19200||b==38400||b==57600||b==115200){c->modbus_baud=b;Debug_Printf("[OK] baud=%lu\r\n",b);}
        else Debug_Print("[ERR] Use 9600/19200/38400/57600/115200\r\n");
    }
    else if (ci_eq(key,"modbus_parity"))    { uint8_t v=(uint8_t)atoi(val); if(v<=2){c->modbus_parity=v;Debug_Printf("[OK] parity=%u\r\n",v);}else Debug_Print("[ERR] 0/1/2\r\n"); }
    else if (ci_eq(key,"modbus_stopbits"))  { uint8_t v=(uint8_t)atoi(val); if(v==1||v==2){c->modbus_stop_bits=v;Debug_Printf("[OK] stopbits=%u\r\n",v);}else Debug_Print("[ERR] 1 or 2\r\n"); }
    else if (ci_eq(key,"modbus_timeout"))   { c->modbus_timeout_ms=(uint16_t)atoi(val); Debug_Printf("[OK] timeout=%u ms\r\n",c->modbus_timeout_ms); }
    else if (ci_eq(key,"poll_interval"))    { c->poll_interval_ms=(uint16_t)atoi(val); Debug_Printf("[OK] poll=%u ms\r\n",c->poll_interval_ms); }
    else if (ci_eq(key,"publish_interval")) { c->publish_interval_ms=(uint32_t)atol(val); Debug_Printf("[OK] publish=%lu ms\r\n",c->publish_interval_ms); }
    else if (ci_eq(key,"delta_mode"))       { c->delta_mode=atoi(val)?1:0; Debug_Printf("[OK] delta_mode=%s\r\n",c->delta_mode?"ON":"OFF"); }
    else if (ci_eq(key,"delta_threshold"))  { c->delta_threshold=(uint16_t)atoi(val); Debug_Printf("[OK] delta_thresh=%u\r\n",c->delta_threshold); }
    else if (ci_eq(key,"modem_type"))      {
        uint8_t v=(uint8_t)atoi(val);
        if(v<=1){c->modem_type=v;Debug_Printf("[OK] modem=%s — run 'save' then 'reboot'\r\n",v==1?"Quectel EC200U":"SIMCom A7670C");}
        else Debug_Print("[ERR] modem_type: 0=SIMCom A7670C, 1=Quectel EC200U\r\n");
    }
    else if (ci_eq(key,"apn"))             { strncpy(c->apn,val,31); Debug_Printf("[OK] apn=%s\r\n",c->apn); }
    else if (ci_eq(key,"broker"))          { strncpy(c->broker,val,63); Debug_Printf("[OK] broker=%s\r\n",c->broker); }
    else if (ci_eq(key,"broker_port"))     { c->broker_port=(uint16_t)atoi(val); Debug_Printf("[OK] port=%u\r\n",c->broker_port); }
    else if (ci_eq(key,"mqtt_client"))     { strncpy(c->mqtt_client_id,val,31); Debug_Printf("[OK] client=%s\r\n",c->mqtt_client_id); }
    else if (ci_eq(key,"mqtt_user"))       { strncpy(c->mqtt_user,val,31); Debug_Printf("[OK] user=%s\r\n",c->mqtt_user); }
    else if (ci_eq(key,"mqtt_pass"))       { strncpy(c->mqtt_pass,val,31); Debug_Print("[OK] pass updated\r\n"); }
    else if (ci_eq(key,"mqtt_topic"))      { strncpy(c->mqtt_topic,val,63); Debug_Printf("[OK] topic=%s\r\n",c->mqtt_topic); }
    else if (ci_eq(key,"alarm_topic"))    { strncpy(c->mqtt_alarm_topic,val,63); Debug_Printf("[OK] alarm_topic=%s\r\n",c->mqtt_alarm_topic); }
    else if (ci_eq(key,"mqtt_qos"))        { uint8_t v=(uint8_t)atoi(val); if(v<=2){c->mqtt_qos=v;Debug_Printf("[OK] qos=%u\r\n",v);}else Debug_Print("[ERR] 0/1/2\r\n"); }
    else if (ci_eq(key,"ota_enabled"))     { c->ota_enabled=atoi(val)?1:0; Debug_Printf("[OK] ota=%s\r\n",c->ota_enabled?"ON":"OFF"); }
    else if (ci_eq(key,"ota_url"))         { strncpy(c->ota_manifest_url,val,127); Debug_Printf("[OK] ota_url=%s\r\n",c->ota_manifest_url); }
    else                                   { Debug_Printf("[ERR] Unknown key: %s\r\n",key); }
}

static void cmd_reg(char *args)
{
    GatewayConfig_t *c = Config_GetMutable();
    char sub[16]={0}; uint8_t si=0;
    char *p=args;
    while(*p&&!isspace((unsigned char)*p)&&si<15) sub[si++]=*p++;
    while(*p&& isspace((unsigned char)*p)) p++;

    if (ci_eq(sub,"list")||ci_eq(sub,"ls")) {
        if (!c->num_regs){Debug_Print("[INFO] No registers.\r\n");return;}
        Debug_Print("\r\n IDX  EN  SLAVE  ADDR    FC  TYPE   SCALE    TAG\r\n");
        Debug_Print(" ---  --  -----  ------  --  -----  -------  ----------------\r\n");
        static const char *tn[]={"U16","I16","U32","I32","F32"};
        for(uint8_t i=0;i<c->num_regs;i++){
            const RegDef_t *r=&c->regs[i];
            Debug_Printf(" %-3u  %-2u  0x%02X   0x%04X  0x%02X %-5s  %-7.2f  %s\r\n",
                i,r->enabled,r->slave_addr,r->reg_addr,r->func_code,
                r->data_type<=DTYPE_FLOAT32?tn[r->data_type]:"?",r->scale,r->tag);
        }
        Debug_Printf("\r\n Total: %u/%u\r\n\r\n",c->num_regs,CONFIG_MAX_REGS);

    } else if (ci_eq(sub,"add")) {
        if (c->num_regs>=CONFIG_MAX_REGS){Debug_Printf("[ERR] Table full (%u)\r\n",CONFIG_MAX_REGS);return;}
        char t[7][32]={{0}};
        uint8_t tc=0; char *q=p;
        while(tc<7&&*q){
            uint8_t k=0;
            while(*q&&!isspace((unsigned char)*q)&&k<31) t[tc][k++]=*q++;
            while(*q&& isspace((unsigned char)*q)) q++;
            if(t[tc][0]) tc++;
        }
        if(tc<6){Debug_Print("[ERR] Usage: reg add <slave> <addr> <fc> <type> <scale> <tag> [alarm]\r\n");return;}

        uint8_t fc=(uint8_t)atoi(t[2]);
        if(fc!=3&&fc!=4){Debug_Print("[ERR] fc must be 3 or 4\r\n");return;}

        RegDataType_t dt;
        if     (ci_eq(t[3],"u16")) dt=DTYPE_UINT16;
        else if(ci_eq(t[3],"i16")) dt=DTYPE_INT16;
        else if(ci_eq(t[3],"u32")) dt=DTYPE_UINT32;
        else if(ci_eq(t[3],"i32")) dt=DTYPE_INT32;
        else if(ci_eq(t[3],"f32")) dt=DTYPE_FLOAT32;
        else{Debug_Print("[ERR] type: u16 i16 u32 i32 f32\r\n");return;}

        RegDef_t *r=&c->regs[c->num_regs];
        r->slave_addr=(uint8_t)atoi(t[0]);
        r->reg_addr  =(uint16_t)strtol(t[1],NULL,0);
        r->func_code =fc;
        r->data_type =dt;
        r->scale     =strtof(t[4],NULL);
        strncpy(r->tag,t[5],CONFIG_MAX_TAG_LEN-1);
        r->enabled=1;
        r->is_alarm=(tc>=7 && ci_eq(t[6],"alarm"))?1:0;
        c->num_regs++;
        Debug_Printf("[OK] reg[%u] added: 0x%02X:0x%04X fc=%u %s scale=%.3f tag=%s%s\r\n",
            c->num_regs-1,r->slave_addr,r->reg_addr,r->func_code,t[3],r->scale,r->tag,
            r->is_alarm?" [ALARM]":"");

    } else if (ci_eq(sub,"del")||ci_eq(sub,"delete")) {
        uint8_t idx=(uint8_t)atoi(p);
        if(idx>=c->num_regs){Debug_Printf("[ERR] Index %u out of range\r\n",idx);return;}
        for(uint8_t i=idx;i<c->num_regs-1;i++) c->regs[i]=c->regs[i+1];
        memset(&c->regs[--c->num_regs],0,sizeof(RegDef_t));
        Debug_Printf("[OK] reg[%u] deleted. Total=%u\r\n",idx,c->num_regs);

    } else if (ci_eq(sub,"enable")) {
        char is[8]={0},es[4]={0};
        sscanf(p,"%7s %3s",is,es);
        uint8_t idx=(uint8_t)atoi(is),en=atoi(es)?1:0;
        if(idx>=c->num_regs){Debug_Printf("[ERR] Index %u out of range\r\n",idx);return;}
        c->regs[idx].enabled=en;
        Debug_Printf("[OK] reg[%u] %s\r\n",idx,en?"enabled":"disabled");

    } else if (ci_eq(sub,"alarm")) {
        char is[8]={0},es[4]={0};
        sscanf(p,"%7s %3s",is,es);
        uint8_t idx=(uint8_t)atoi(is),al=atoi(es)?1:0;
        if(idx>=c->num_regs){Debug_Printf("[ERR] Index %u out of range\r\n",idx);return;}
        c->regs[idx].is_alarm=al;
        Debug_Printf("[OK] reg[%u] %s\r\n",idx,al?"marked as ALARM":"marked as PARAM");

    } else {
        Debug_Printf("[ERR] Unknown: '%s'. Use add/del/enable/alarm/list\r\n",sub);
    }
}

/* ── SPI flash test commands ─────────────────────────────────────── */
static void cmd_flash(char *args)
{
    char sub[16] = {0};
    uint8_t si = 0;
    char *p = args;
    while (*p && !isspace((unsigned char)*p) && si < 15) sub[si++] = *p++;
    while (*p &&  isspace((unsigned char)*p)) p++;

    if (ci_eq(sub, "id")) {
        uint32_t id = 0;
        W25Q_ReadID(&id);
        Debug_Printf("[FLASH] JEDEC ID: 0x%06lX", id);
        uint8_t mfr = (id >> 16) & 0xFF;
        uint8_t cap = id & 0xFF;
        const char *mfr_name = mfr==0xEF ? "Winbond" :
                               mfr==0x20 ? "XMC" :
                               mfr==0xC8 ? "GigaDevice" : "Unknown";
        if (cap >= 0x14 && cap <= 0x18)
            Debug_Printf("  (%s %uMbit)\r\n", mfr_name,
                         (unsigned)(1U << (cap - 0x10)) / 8 * 8);
        else
            Debug_Printf("  (%s, cap=0x%02X)\r\n", mfr_name, cap);

    } else if (ci_eq(sub, "test")) {
        /* Full write-read-verify test on sector 0 (first 4KB) */
        Debug_Print("[FLASH] === SPI Flash Test ===\r\n");

        /* Step 1: Read JEDEC ID */
        uint32_t id = 0;
        W25Q_ReadID(&id);
        Debug_Printf("[FLASH] JEDEC ID: 0x%06lX\r\n", id);
        uint8_t tmfr = (id >> 16) & 0xFF;
        if (tmfr != 0xEF && tmfr != 0x20 && tmfr != 0xC8) {
            Debug_Print("[FLASH] FAIL — unknown chip. Check wiring:\r\n");
            Debug_Print("        PA5=SCK  PA6=MISO  PA7=MOSI  PA8=CS\r\n");
            return;
        }
        Debug_Print("[FLASH] PASS — chip detected\r\n");

        /* Step 2: Erase sector 0 */
        Debug_Print("[FLASH] Erasing sector 0 (0x000000)...\r\n");
        W25Q_EraseSector(0x000000);

        /* Step 3: Verify erase — all bytes should be 0xFF */
        static uint8_t buf[256];
        W25Q_Read(0x000000, buf, 256);
        bool erase_ok = true;
        for (uint16_t i = 0; i < 256; i++) {
            if (buf[i] != 0xFF) { erase_ok = false; break; }
        }
        Debug_Printf("[FLASH] Erase verify: %s\r\n", erase_ok ? "PASS" : "FAIL");
        if (!erase_ok) return;

        /* Step 4: Write a test pattern */
        for (uint16_t i = 0; i < 256; i++) buf[i] = (uint8_t)(i & 0xFF);
        Debug_Print("[FLASH] Writing 256-byte pattern...\r\n");
        W25Q_WritePage(0x000000, buf, 256);

        /* Step 5: Read back and verify */
        static uint8_t rbuf[256];
        memset(rbuf, 0, 256);
        W25Q_Read(0x000000, rbuf, 256);
        bool write_ok = true;
        uint16_t fail_at = 0;
        for (uint16_t i = 0; i < 256; i++) {
            if (rbuf[i] != (uint8_t)(i & 0xFF)) {
                write_ok = false;
                fail_at = i;
                break;
            }
        }
        if (write_ok)
            Debug_Print("[FLASH] Write+Read verify: PASS\r\n");
        else
            Debug_Printf("[FLASH] Write+Read verify: FAIL at byte %u "
                         "(wrote 0x%02X, read 0x%02X)\r\n",
                         fail_at, (uint8_t)(fail_at & 0xFF), rbuf[fail_at]);

        /* Step 6: Clean up — erase sector again */
        W25Q_EraseSector(0x000000);
        Debug_Print("[FLASH] Sector 0 cleaned up\r\n");
        Debug_Print("[FLASH] === Test complete ===\r\n");

    } else if (ci_eq(sub, "dump")) {
        /* dump <addr> [len] — hex dump flash contents */
        uint32_t addr = (uint32_t)strtol(p, &p, 0);
        while (*p && isspace((unsigned char)*p)) p++;
        uint16_t len = *p ? (uint16_t)atoi(p) : 64;
        if (len > 256) len = 256;

        static uint8_t dbuf[256];
        W25Q_Read(addr, dbuf, len);
        Debug_Printf("[FLASH] Dump 0x%06lX (%u bytes):\r\n", addr, len);
        for (uint16_t i = 0; i < len; i += 16) {
            Debug_Printf("  %06lX: ", addr + i);
            for (uint8_t j = 0; j < 16 && (i + j) < len; j++)
                Debug_Printf("%02X ", dbuf[i + j]);
            Debug_Print("\r\n");
        }

    } else {
        Debug_Print("Usage:\r\n");
        Debug_Print("  flash id          — read JEDEC chip ID\r\n");
        Debug_Print("  flash test        — full erase/write/read self-test\r\n");
        Debug_Print("  flash dump <addr> [len]  — hex dump (max 256)\r\n");
    }
}

/* ── Alarm bit commands ─────────────────────────────────────────── */
static void cmd_alarm(char *args)
{
    GatewayConfig_t *c = Config_GetMutable();
    char sub[16] = {0};
    uint8_t si = 0;
    char *p = args;
    while (*p && !isspace((unsigned char)*p) && si < 15) sub[si++] = *p++;
    while (*p &&  isspace((unsigned char)*p)) p++;

    if (ci_eq(sub, "list") || ci_eq(sub, "ls")) {
        if (!c->num_alarm_bits) { Debug_Print("[INFO] No alarm bits defined.\r\n"); return; }
        Debug_Print("\r\n IDX  REG  BIT  NAME\r\n");
        Debug_Print(" ---  ---  ---  -------\r\n");
        for (uint8_t i = 0; i < c->num_alarm_bits; i++) {
            const AlarmBitDef_t *a = &c->alarm_bits[i];
            Debug_Printf(" %-3u  %-3u  %-3u  %s\r\n",
                         i, a->reg_idx, a->bit_pos, a->name);
        }
        Debug_Printf("\r\n Total: %u/%u\r\n\r\n", c->num_alarm_bits, CONFIG_MAX_ALARM_BITS);

    } else if (ci_eq(sub, "add")) {
        if (c->num_alarm_bits >= CONFIG_MAX_ALARM_BITS) {
            Debug_Printf("[ERR] Table full (%u)\r\n", CONFIG_MAX_ALARM_BITS);
            return;
        }
        /* Parse: alarm add <reg_idx> <bit> <name> */
        char t[3][16] = {{0}};
        uint8_t tc = 0;
        char *q = p;
        while (tc < 3 && *q) {
            uint8_t k = 0;
            while (*q && !isspace((unsigned char)*q) && k < 15) t[tc][k++] = *q++;
            while (*q &&  isspace((unsigned char)*q)) q++;
            if (t[tc][0]) tc++;
        }
        if (tc < 3) {
            Debug_Print("[ERR] Usage: alarm add <reg_idx> <bit> <name>\r\n");
            Debug_Print("      example: alarm add 5 0 OV\r\n");
            return;
        }

        uint8_t ri = (uint8_t)atoi(t[0]);
        uint8_t bp = (uint8_t)atoi(t[1]);
        if (ri >= c->num_regs) {
            Debug_Printf("[ERR] reg_idx %u out of range (max %u)\r\n", ri, c->num_regs - 1);
            return;
        }
        if (!c->regs[ri].is_alarm) {
            Debug_Printf("[ERR] reg[%u] is not an alarm register. Use 'reg alarm %u 1' first\r\n", ri, ri);
            return;
        }
        if (bp > 15) {
            Debug_Print("[ERR] bit must be 0-15\r\n");
            return;
        }

        AlarmBitDef_t *a = &c->alarm_bits[c->num_alarm_bits];
        a->reg_idx = ri;
        a->bit_pos = bp;
        strncpy(a->name, t[2], CONFIG_ALARM_NAME_LEN - 1);
        a->name[CONFIG_ALARM_NAME_LEN - 1] = '\0';
        c->num_alarm_bits++;
        Debug_Printf("[OK] alarm[%u] added: reg[%u] bit %u = \"%s\"\r\n",
                     c->num_alarm_bits - 1, ri, bp, a->name);

    } else if (ci_eq(sub, "del") || ci_eq(sub, "delete")) {
        uint8_t idx = (uint8_t)atoi(p);
        if (idx >= c->num_alarm_bits) {
            Debug_Printf("[ERR] Index %u out of range\r\n", idx);
            return;
        }
        for (uint8_t i = idx; i < c->num_alarm_bits - 1; i++)
            c->alarm_bits[i] = c->alarm_bits[i + 1];
        memset(&c->alarm_bits[--c->num_alarm_bits], 0, sizeof(AlarmBitDef_t));
        Debug_Printf("[OK] alarm[%u] deleted. Total=%u\r\n", idx, c->num_alarm_bits);

    } else {
        Debug_Print("Usage:\r\n");
        Debug_Print("  alarm add <reg_idx> <bit> <name>  — define alarm bit\r\n");
        Debug_Print("  alarm del <idx>                   — remove alarm bit\r\n");
        Debug_Print("  alarm list                        — show all alarm bits\r\n");
    }
}

/* ── Dispatch ────────────────────────────────────────────────────── */
static void dispatch(char *line)
{
    trim(line);
    if (!strlen(line)) return;
    char cmd[32]={0}; uint8_t ci_=0;
    char *p=line;
    while(*p&&!isspace((unsigned char)*p)&&ci_<31) cmd[ci_++]=*p++;
    while(*p&& isspace((unsigned char)*p)) p++;

    if      (ci_eq(cmd,"help")||ci_eq(cmd,"?")) cmd_help();
    else if (ci_eq(cmd,"log")) {
        if (ci_eq(p,"on"))       { g_log_enabled = 1; cli_print("[LOG] Logging ON\r\n"); }
        else if (ci_eq(p,"off")) { g_log_enabled = 0; cli_print("[LOG] Logging OFF\r\n"); }
        else cli_printf("[LOG] Currently %s. Usage: log on | log off\r\n", g_log_enabled?"ON":"OFF");
    }
    else if (ci_eq(cmd,"mbtest")) {
        extern UART_HandleTypeDef huart3;
        uint8_t test[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};

        Debug_Printf("[MBTEST] USART3 BRR=0x%04X (expect 0x1D4C for 9600@72MHz)\r\n",
                     (unsigned)USART3->BRR);
        Debug_Printf("[MBTEST] USART3 CR1=0x%04X SR=0x%04X\r\n",
                     (unsigned)USART3->CR1, (unsigned)USART3->SR);

        /* Force DE high */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart3, test, 8, 1000);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        Debug_Printf("[MBTEST] Sent 8 bytes, HAL status=%d\r\n", (int)st);
    }
    else if (ci_eq(cmd,"uid")) {
        /* Try standard UID address */
        uint32_t u0 = *(volatile uint32_t *)(0x1FFFF7E8UL);
        uint32_t u1 = *(volatile uint32_t *)(0x1FFFF7ECUL);
        uint32_t u2 = *(volatile uint32_t *)(0x1FFFF7F0UL);
        Debug_Printf("[UID] @0x1FFFF7E8: %08lX %08lX %08lX\r\n",
                     (unsigned long)u0, (unsigned long)u1, (unsigned long)u2);

        /* Check flash size register */
        uint16_t flash_kb = *(volatile uint16_t *)(0x1FFFF7E0UL);
        Debug_Printf("[UID] Flash size reg: %u KB\r\n", flash_kb);

        /* DBGMCU_IDCODE — reveals real chip identity */
        uint32_t idcode = *(volatile uint32_t *)(0xE0042000UL);
        Debug_Printf("[UID] DBGMCU_IDCODE=0x%08lX (DEV_ID=0x%03lX REV=0x%04lX)\r\n",
                     (unsigned long)idcode,
                     (unsigned long)(idcode & 0xFFFUL),
                     (unsigned long)((idcode >> 16) & 0xFFFFUL));

        /* Derive ID */
        uint32_t h  = u0 ^ u1 ^ u2;
        uint16_t id = (uint16_t)((h ^ (h >> 16)) & 0xFFFFU);
        Debug_Printf("[UID] Folded ID=0x%04X  Config device_id=0x%04X\r\n",
                     id, Config_Get()->device_id);

        if (u0 == 0 && u1 == 0 && u2 == 0)
            Debug_Print("[UID] WARNING: All zeros — likely clone chip. Use 'set device_id <n>' manually.\r\n");
    }
    else if (ci_eq(cmd,"i2cscan")) {
        extern I2C_HandleTypeDef hi2c1;
        Debug_Print("[I2C] Scanning...\r\n");
        uint8_t found=0;
        for (uint8_t a=1; a<128; a++) {
            if (HAL_I2C_IsDeviceReady(&hi2c1, a<<1, 2, 10) == HAL_OK) {
                Debug_Printf("  Found device at 0x%02X\r\n", a);
                found++;
            }
        }
        if (!found) Debug_Print("  No devices found! Check wiring/pullups.\r\n");
        else Debug_Printf("  %u device(s) found.\r\n", found);
    }
    else if (ci_eq(cmd,"show")) {
        if     (strstr(p,"config")) Config_Print();
        else if(strstr(p,"alarm"))  cmd_alarm((char*)"list");
        else if(strstr(p,"reg"))    cmd_reg((char*)"list");
        else    Debug_Print("[ERR] show config | show regs | show alarms\r\n");
    }
    else if (ci_eq(cmd,"set"))    cmd_set(p);
    else if (ci_eq(cmd,"reg"))    cmd_reg(p);
    else if (ci_eq(cmd,"alarm"))  cmd_alarm(p);
    else if (ci_eq(cmd,"flash"))  cmd_flash(p);
    else if (ci_eq(cmd,"save"))   { Debug_Print("[CFG] Saving...\r\n"); Config_Save(); }
    else if (ci_eq(cmd,"reset"))  { Config_LoadDefaults(); Debug_Print("[OK] Defaults loaded (not saved)\r\n"); }
    else if (ci_eq(cmd,"reboot")) { Debug_Print("[SYS] Rebooting...\r\n"); HAL_Delay(500); NVIC_SystemReset(); }
    else if (ci_eq(cmd,"led")) {
        if (!*p) {
            cli_printf("[LED] Brightness: %u/255\r\n", RGB_GetBrightness());
        } else {
            int val = atoi(p);
            if (val >= 0 && val <= 255) {
                RGB_SetBrightness((uint8_t)val);
                cli_printf("[LED] Brightness set to %u\r\n", val);
            } else {
                cli_print("[ERR] led <0-255>  (0=off, 255=max)\r\n");
            }
        }
    }
    else if (ci_eq(cmd,"at")) {
        /* AT passthrough — uses the GSM driver's ring buffer RX path */
        if (!*p) { cli_print("Usage: at <command>  (e.g. at AT+CSQ)\r\n"); }
        else {
            cli_printf("[AT>] %s\r\n", p);
            AT_BeginCmd(p);  /* flushes ring, sends cmd+\r\n */
            /* Poll ring buffer for OK/ERROR/timeout, print accumulated response */
            uint32_t deadline = HAL_GetTick() + 5000U;
            int r;
            do { r = AT_Poll("OK", deadline); } while (r == 0);
            const char *acc = AT_GetAcc();
            if (acc[0]) cli_printf("[AT<] %s\r\n", acc);
            else        cli_print("[AT<] (no response)\r\n");
        }
    }
    else if (ci_eq(cmd,"fwupdate")) {
        Debug_Print("[SYS] Entering serial firmware update mode...\r\n");
        Debug_Print("[SYS] Run 'python fw_update.py' on PC now.\r\n");
        HAL_Delay(500);
        BL_TriggerSerialUpdate();  /* does not return */
    }
    else    Debug_Printf("[ERR] Unknown command '%s'. Type 'help'\r\n",cmd);
}

/* ── Process (call in main loop) ─────────────────────────────────── */
void CLI_Process(void)
{
    uint8_t b;
    while (rbpop(&b)) {
        if (b=='\r'||b=='\n') {
            cli_print("\r\n");
            if (s_len) {
                s_cmd[s_len]=0;
                s_in_dispatch = 1;
                dispatch(s_cmd);
                s_in_dispatch = 0;
                s_len=0;
            }
            cli_print("> ");
        } else if (b==BS||b==DEL) {
            if (s_len) {
                s_len--;
                HAL_UART_Transmit(s_huart,(uint8_t*)"\x08 \x08",3,10);
            }
        } else if (b>=0x20&&b<0x7F&&s_len<CLI_CMD_BUF_SIZE-1) {
            s_cmd[s_len++]=(char)b;
            echo((char)b);
        }
    }
}
