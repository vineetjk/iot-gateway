/**
 * ota_manager.c
 * =========================================================================
 * OTA manager implementation.
 * =========================================================================
 */
#include "ota_manager.h"
#include "modem_hal.h"
#include "rgb_led.h"
#include "debug_cli.h"
#include "display_ui.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── BL shared API ───────────────────────────────────────────────── */
void BL_TriggerOTA(void)
{
    *(volatile uint32_t *)OTA_FLAG_ADDR = OTA_FLAG_MAGIC;
    __DSB();
    NVIC_SystemReset();
}

void BL_TriggerSerialUpdate(void)
{
    *(volatile uint32_t *)BL_SERIAL_FLAG_ADDR = BL_SERIAL_FLAG_MAGIC;
    __DSB();
    NVIC_SystemReset();
}

void BL_ConfirmBoot(void)
{
    OtaMeta_t m;
    W25Q_Read(SPI_FLASH_OTA_META_ADDR,(uint8_t*)&m,sizeof(m));
    if (m.magic!=OTA_META_MAGIC||m.first_boot_done) return;
    m.first_boot_done=1;
    uint32_t crc=0xFFFFFFFFUL;
    const uint8_t *d=(const uint8_t*)&m; uint32_t l=sizeof(OtaMeta_t)-4U;
    while(l--){ crc^=*d++; for(uint8_t b=0;b<8;b++) crc=(crc&1)?((crc>>1)^0xEDB88320UL):(crc>>1); }
    m.meta_crc32=crc^0xFFFFFFFFUL;
    W25Q_EraseSector(SPI_FLASH_OTA_META_ADDR);
    W25Q_Write(SPI_FLASH_OTA_META_ADDR,(uint8_t*)&m,sizeof(m));
    Debug_Print("[OTA] Boot confirmed — rollback released\r\n");
}

bool BL_IsFirstBootAfterOTA(void)
{
    OtaMeta_t m;
    W25Q_Read(SPI_FLASH_OTA_META_ADDR,(uint8_t*)&m,sizeof(m));
    return (m.magic==OTA_META_MAGIC && m.first_boot_done==0);
}

/* ── Init ────────────────────────────────────────────────────────── */
void OTA_Init(SPI_HandleTypeDef *h, GPIO_TypeDef *cp, uint16_t cn)
{
    W25Q_Init(h,cp,cn);
    Debug_Print(W25Q_Detect()?"[OTA] W25Q64 OK\r\n":"[OTA] W25Q64 NOT FOUND!\r\n");
}

/* ── Tiny JSON string extractor (no heap) ────────────────────────── */
static bool jget(const char *json, const char *key, char *out, uint16_t ol)
{
    char search[48]; snprintf(search,sizeof(search),"\"%s\":",key);
    const char *p=strstr(json,search); if(!p) return false;
    p+=strlen(search); while(*p==' ') p++;
    if(*p=='"') {
        p++; uint16_t i=0;
        while(*p&&*p!='"'&&i<ol-1U) out[i++]=*p++;
        out[i]='\0';
    } else {
        uint16_t i=0;
        while(*p&&*p!=','&&*p!='}'&&*p!=' '&&i<ol-1U) out[i++]=*p++;
        out[i]='\0';
    }
    return true;
}

static uint32_t parse_version(const char *s)
{
    uint32_t M=0,m=0,p=0;
    if(strchr(s,'.')) sscanf(s,"%lu.%lu.%lu",&M,&m,&p);
    else return (uint32_t)atol(s);
    return (M<<16)|(m<<8)|p;
}

static uint32_t crc32_buf(const uint8_t *d, uint32_t l)
{
    uint32_t c=0xFFFFFFFFUL;
    while(l--){ c^=*d++; for(uint8_t b=0;b<8;b++) c=(c&1)?((c>>1)^0xEDB88320UL):(c>>1); }
    return c^0xFFFFFFFFUL;
}

static void write_meta(OtaMeta_t *m)
{
    m->meta_crc32=crc32_buf((const uint8_t*)m,sizeof(OtaMeta_t)-4U);
    W25Q_EraseSector(SPI_FLASH_OTA_META_ADDR);
    W25Q_Write(SPI_FLASH_OTA_META_ADDR,(uint8_t*)m,sizeof(OtaMeta_t));
}

/* ── Server version check ────────────────────────────────────────── */
bool OTA_GetServerVersion(uint32_t *ver)
{
    const GatewayConfig_t *cfg=Config_Get();
    static char mbuf[512]; uint16_t mlen=0;
    if(Modem_HttpGet(cfg->ota_manifest_url,mbuf,sizeof(mbuf),&mlen)!=GSM_OK) return false;
    char vs[16]={0};
    if(!jget(mbuf,"latest_version",vs,sizeof(vs))) return false;
    *ver=parse_version(vs); return true;
}

/* ── Check and update ────────────────────────────────────────────── */
OtaResult_t OTA_CheckAndUpdate(void)
{
    const GatewayConfig_t *cfg=Config_Get();
    static char mbuf[512]; uint16_t mlen=0;

    Debug_Print("[OTA] Fetching manifest...\r\n");
    if(Modem_HttpGet(cfg->ota_manifest_url,mbuf,sizeof(mbuf),&mlen)!=GSM_OK) {
        Debug_Print("[OTA] Cannot reach server\r\n"); return OTA_RESULT_NO_SERVER;
    }

    char vs[16]={0}; if(!jget(mbuf,"latest_version",vs,sizeof(vs))) return OTA_RESULT_NO_SERVER;
    uint32_t srv_ver=parse_version(vs);
    Debug_Printf("[OTA] Device=0x%08lX Server=0x%08lX (%s)\r\n",cfg->fw_version,srv_ver,vs);
    if(srv_ver<=cfg->fw_version){ Debug_Print("[OTA] Up to date\r\n"); return OTA_RESULT_UP_TO_DATE; }

    char fw_url[128]={0},fw_size_s[16]={0},fw_crc_s[16]={0};
    jget(mbuf,"fw_url",  fw_url,    sizeof(fw_url));
    jget(mbuf,"fw_size", fw_size_s, sizeof(fw_size_s));
    jget(mbuf,"fw_crc32",fw_crc_s,  sizeof(fw_crc_s));
    uint32_t fw_size=(uint32_t)atol(fw_size_s);
    uint32_t fw_crc =(uint32_t)strtoul(fw_crc_s,NULL,0);

    if(!fw_size||fw_size>SPI_FLASH_SLOT_SIZE){
        Debug_Printf("[OTA] Bad fw_size=%lu\r\n",fw_size); return OTA_RESULT_DOWNLOAD_FAIL;
    }
    Debug_Printf("[OTA] Downloading %lu bytes from: %s\r\n",fw_size,fw_url);
    Display_ShowUpdating(0, fw_size);

    /* Disconnect MQTT before HTTP download to avoid resource conflict */
    Modem_MqttDisconnect();
    Modem_MqttSetUp(false);
    HAL_Delay(500U);

    W25Q_EraseRange(SPI_FLASH_SLOT_A_ADDR,fw_size);

    __attribute__((aligned(4))) static uint8_t chunk[256];
    uint32_t done=0, crc_run=0xFFFFFFFFUL;

    while(done<fw_size){
        uint32_t csz=((fw_size-done)>sizeof(chunk))?sizeof(chunk):(fw_size-done);
        uint16_t got=0;
        if(Modem_HttpGetRange(fw_url,done,done+csz-1U,chunk,&got)!=GSM_OK){
            Debug_Printf("[OTA] Download fail at offset %lu\r\n",done);
            Display_ShowError(ERR_OTA_DOWNLOAD);
            return OTA_RESULT_DOWNLOAD_FAIL;
        }
        W25Q_Write(SPI_FLASH_SLOT_A_ADDR+done,chunk,got);
        const uint8_t *p=chunk; uint32_t n=got;
        while(n--){ crc_run^=*p++; for(uint8_t b=0;b<8;b++) crc_run=(crc_run&1)?((crc_run>>1)^0xEDB88320UL):(crc_run>>1); }
        done+=got;
        if(done%8192==0||done==fw_size) {
            Debug_Printf("[OTA] %lu/%lu bytes\r\n",done,fw_size);
            Display_ShowUpdating(done, fw_size);
        }
    }
    crc_run^=0xFFFFFFFFUL;

    if(crc_run!=fw_crc){
        Debug_Printf("[OTA] CRC fail: got=0x%08lX exp=0x%08lX\r\n",crc_run,fw_crc);
        Display_ShowError(ERR_OTA_VERIFY);
        return OTA_RESULT_VERIFY_FAIL;
    }
    Debug_Print("[OTA] CRC OK\r\n");

    OtaMeta_t meta; memset(&meta,0,sizeof(meta));
    meta.magic          = OTA_META_MAGIC;
    meta.fw_version     = srv_ver;
    meta.fw_size        = fw_size;
    meta.fw_crc32       = fw_crc;
    meta.slot_addr      = SPI_FLASH_SLOT_A_ADDR;
    meta.status         = OTA_STATUS_DOWNLOADED;
    meta.first_boot_done= 0;
    meta.boot_attempts  = 0;
    strncpy(meta.fw_url,fw_url,127);
    write_meta(&meta);

    Debug_Print("[OTA] Metadata written — rebooting to bootloader\r\n");
    HAL_Delay(500U);
    BL_TriggerOTA();    /* does not return */
    return OTA_RESULT_UPDATED;
}

/* Progress callback */
static void ota_progress(uint32_t done, uint32_t total)
{
    Debug_Printf("[OTA] %lu/%lu\r\n", done, total);
    Display_ShowUpdating(done, total);
}

/* ── Core MQTT-triggered OTA (shared by JSON and CSV entry points) ── */
static OtaResult_t ota_download(const char *fw_url, uint32_t fw_size,
                                 uint32_t fw_crc, uint32_t ver)
{
    if (!fw_url[0] || !fw_size) {
        Debug_Print("[OTA] Missing url/size\r\n");
        return OTA_RESULT_DOWNLOAD_FAIL;
    }
    if (fw_size > SPI_FLASH_SLOT_SIZE) {
        Debug_Printf("[OTA] Bad size=%lu\r\n", fw_size);
        return OTA_RESULT_DOWNLOAD_FAIL;
    }

    Debug_Printf("[OTA] Downloading %lu bytes\r\n", fw_size);
    Debug_Printf("[OTA] URL: %s\r\n", fw_url);

    /* Orange LED = OTA in progress */
    RGB_YELLOW();
    Display_ShowUpdating(0, fw_size);

    /* Disconnect MQTT before HTTP download */
    Modem_MqttDisconnect();
    Modem_MqttSetUp(false);
    HAL_Delay(500U);

    /* Erase flash */
    W25Q_EraseRange(SPI_FLASH_SLOT_A_ADDR, fw_size);

    /* Stream download directly to SPI flash */
    uint32_t written = 0;
    GsmResult_t r = Modem_HttpDownloadToFlash(fw_url, SPI_FLASH_SLOT_A_ADDR,
                                              fw_size, &written, ota_progress);
    if (r != GSM_OK || written < fw_size) {
        Debug_Printf("[OTA] Download fail: %lu/%lu\r\n", written, fw_size);
        Display_ShowError(ERR_OTA_DOWNLOAD);
        return OTA_RESULT_DOWNLOAD_FAIL;
    }

    /* CRC verify by reading back from flash */
    uint32_t crc_run = 0xFFFFFFFFUL;
    static uint8_t vbuf[256];
    uint32_t voff = 0;
    while (voff < fw_size) {
        uint16_t vlen = ((fw_size - voff) > 256U) ? 256U : (uint16_t)(fw_size - voff);
        W25Q_Read(SPI_FLASH_SLOT_A_ADDR + voff, vbuf, vlen);
        const uint8_t *p = vbuf; uint32_t n = vlen;
        while (n--) { crc_run ^= *p++; for (uint8_t b = 0; b < 8; b++) crc_run = (crc_run & 1) ? ((crc_run >> 1) ^ 0xEDB88320UL) : (crc_run >> 1); }
        voff += vlen;
    }
    crc_run ^= 0xFFFFFFFFUL;

    if (fw_crc && crc_run != fw_crc) {
        Debug_Printf("[OTA] CRC fail: got=0x%08lX exp=0x%08lX\r\n", crc_run, fw_crc);
        Display_ShowError(ERR_OTA_VERIFY);
        return OTA_RESULT_VERIFY_FAIL;
    }
    Debug_Print("[OTA] CRC OK\r\n");

    OtaMeta_t meta; memset(&meta, 0, sizeof(meta));
    meta.magic           = OTA_META_MAGIC;
    meta.fw_version      = ver;
    meta.fw_size         = fw_size;
    meta.fw_crc32        = crc_run;
    meta.slot_addr       = SPI_FLASH_SLOT_A_ADDR;
    meta.status          = OTA_STATUS_DOWNLOADED;
    meta.first_boot_done = 0;
    meta.boot_attempts   = 0;
    strncpy(meta.fw_url, fw_url, 127);
    write_meta(&meta);

    Debug_Print("[OTA] Metadata written — rebooting to bootloader\r\n");
    HAL_Delay(500U);
    BL_TriggerOTA();    /* does not return */
    return OTA_RESULT_UPDATED;
}

/* ── JSON entry point (legacy / manifest-poll) ── */
OtaResult_t OTA_HandleMqttCommand(const char *payload)
{
    char cmd_str[16]={0};
    if (!jget(payload, "cmd", cmd_str, sizeof(cmd_str))) return OTA_RESULT_NO_SERVER;
    if (strcmp(cmd_str, "ota") != 0) return OTA_RESULT_NO_SERVER;

    char fw_url[128]={0}, sz_s[16]={0}, crc_s[16]={0}, ver_s[16]={0};
    jget(payload, "url",     fw_url, sizeof(fw_url));
    jget(payload, "size",    sz_s,   sizeof(sz_s));
    jget(payload, "crc32",   crc_s,  sizeof(crc_s));
    jget(payload, "version", ver_s,  sizeof(ver_s));

    return ota_download(fw_url, (uint32_t)atol(sz_s),
                        (uint32_t)strtoul(crc_s, NULL, 0),
                        ver_s[0] ? parse_version(ver_s) : 0);
}

/* ── CSV entry point: ota,<url>,<size>,<crc32>[,<version>] ── */
OtaResult_t OTA_HandleMqttCsv(const char *url, const char *size_s,
                                const char *crc_s, const char *ver_s)
{
    return ota_download(url, (uint32_t)atol(size_s),
                        (uint32_t)strtoul(crc_s, NULL, 0),
                        ver_s[0] ? parse_version(ver_s) : 0);
}
