/*
 * ESP01 APRS - Web Arayüzlü Yapım (BMP280 Entegre)
 * -------------------------------------------------------------------
 * İlk açılışta (veya kayıtlı WiFi olmadığında) AP modu ile
 *   "ESP01_APRS" adlı ağ kurar → Kullanıcı bağlanır → 192.168.4.1
 * Web arayüzünden:
 *   - WiFi ağları eklenir / silinir
 *   - APRS callsign, passcode, konum, mesajlar ayarlanır
 *   - BMP280 Sensör sıcaklık ve basınç bilgileri "Bağlantı Durumu" sekmesinde gösterilir.
 *   - BMP280 verileri, statüs mesajına otomatik olarak eklenir.
 *   - Manuel APRS mesajı gönderilir (Kimden, Kime, Mesaj)
 *   - Gelen APRS mesajları görüntülenir ve yönetilir.
 * Ayarlar kaydedilince cihaz yeniden başlar, bu kez STA modunda
 * kayıtlı ağlara bağlanır ve APRS veri gönderir.
 *
 * ESP-01 + BMP280 uyarlaması
 * ESP-01 I2C Pinleri:
 *   - GPIO0: SDA (Normal çalışma için HIGH olmalı, pull-up gerekebilir)
 *   - GPIO2: SCL (Normal çalışma için HIGH olmalı, pull-up gerekebilir)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// BMP280 için kütüphaneler
#include <Wire.h>             // I2C iletişimi için
#include <Adafruit_Sensor.h>  // Adafruit sensör kütüphaneleri için temel
#include <Adafruit_BMP280.h>  // BMP280 barometrik basınç ve sıcaklık sensörü

// ─────────────────────────────────────────────
//  EEPROM Yapısı (toplam 1024 byte)
// ─────────────────────────────────────────────
// [0..3]      Magic "APRS"
// ── WiFi (4 ağ × 96 byte = 384) ──
// [4..35]     SSID_0 (32)   [36..99]   PASS_0 (64)
// [100..131]  SSID_1 (32)   [132..195] PASS_1 (64)
// [196..227]  SSID_2 (32)   [228..291] PASS_2 (64)
// [292..323]  SSID_3 (32)   [324..387] PASS_3 (64)
// [388]       WiFi count (1 byte)
// ── APRS (389…618) ──
// [389..398]  mycall       (10)
// [399..406]  aprspass     ( 8)
// [407..414]  symbol_str   ( 8)
// [415..446]  comment      (32)
// [447..488]  message      (42)
// [489..552]  aprshost     (64)
// [553..584]  latitude     (32)
// [585..616]  longitude    (32)
// [617..618]  aprsport     ( 2)  ← uint16 big-endian
// ─────────────────────────────────────────────

static const int  EEPROM_SIZE   = 1024;
static const int  MAGIC_ADDR    = 0;
static const char MAGIC[]       = "APRS";

// ── WiFi adres sabitleri ──
static const int  WIFI_SSID_LEN = 32;
static const int  WIFI_PASS_LEN = 64;
static const int  WIFI_BLOCK    = WIFI_SSID_LEN + WIFI_PASS_LEN; // 96
static const int  WIFI_BASE     = 4;
static const int  MAX_WIFI      = 4;
static const int  WIFI_CNT_ADDR = WIFI_BASE + (MAX_WIFI * WIFI_BLOCK); // 388

// ── APRS adres sabitleri ──
static const int  APRS_CALL_LEN     = 10;
static const int  APRS_PASS_LEN     = 8;
static const int  APRS_SYMBOL_LEN   = 8;
static const int  APRS_COMMENT_LEN  = 32;
static const int  APRS_MSG_LEN      = 42; // APRS status/mesaj alanı max 67 karakter, bizim EEPROM 42
static const int  APRS_HOST_LEN     = 64;
static const int  APRS_LAT_LEN      = 32;
static const int  APRS_LON_LEN      = 32;
static const int  APRS_PORT_LEN     = 2;


static const int  APRS_CALL_ADDR    = 389;
static const int  APRS_PASS_ADDR    = APRS_CALL_ADDR    + APRS_CALL_LEN; // 389 + 10 = 399
static const int  APRS_SYMBOL_ADDR  = APRS_PASS_ADDR    + APRS_PASS_LEN; // 399 + 8  = 407
static const int  APRS_COMMENT_ADDR = APRS_SYMBOL_ADDR  + APRS_SYMBOL_LEN; // 407 + 8  = 415
static const int  APRS_MSG_ADDR     = APRS_COMMENT_ADDR + APRS_COMMENT_LEN; // 415 + 32 = 447
static const int  APRS_HOST_ADDR    = APRS_MSG_ADDR     + APRS_MSG_LEN; // 447 + 42 = 489
static const int  APRS_LAT_ADDR     = APRS_HOST_ADDR    + APRS_HOST_LEN; // 489 + 64 = 553
static const int  APRS_LON_ADDR     = APRS_LAT_ADDR     + APRS_LAT_LEN; // 553 + 32 = 585
static const int  APRS_PORT_ADDR    = APRS_LON_ADDR     + APRS_LON_LEN; // 585 + 32 = 617


// ─────────────────────────────────────────────
//  Runtime yapıları
// ─────────────────────────────────────────────
struct WiFiEntry {
  String ssid;
  String pass;
};

struct APRSConfig {
  String mycall;      // TA3OER-4
  String aprspass;    // passcode
  String symbol_str;  // sembol (örn "/I")
  String comment;     // yorum
  String message;     // status mesaj
  String aprshost;    // sunucu (default: france.aprs2.net)
  String latitude;    // enlem  (örn "4019.25N")
  String longitude;   // boylam (örn "02624.15E")
  uint16_t aprsport;  // port   (default: 14580)
};

static WiFiEntry  savedWifi[MAX_WIFI];
static int        savedWifiCount = 0;
static APRSConfig aprsConf;

// BMP280 sensör nesnesi
// ESP-01 için I2C pinleri Wire.begin(SDA, SCL) olarak ayarlanacak.
// BMP280'in varsayılan I2C adresi 0x77'dir. Eğer sensörünüz 0x76 kullanıyorsa,
// setup() içinde bmp.begin(0x76) olarak değiştirmeniz gerekebilir.
Adafruit_BMP280 bmp;

// Sensörün başlatılıp başlatılamadığını takip etmek için
bool bmp280_initialized = false;

// Gelen mesajları saklamak için yapı ve dizi
struct ReceivedAPRSMessage {
  String sender;
  String message;
  String timestamp; // Mesajın alındığı zaman (saniye cinsinden)
  bool   read;      // Okundu/Okunmadı durumu
};

const int MAX_INCOMING_MESSAGES = 20; // RAM'de tutulacak maksimum mesaj sayısı
ReceivedAPRSMessage incomingMessages[MAX_INCOMING_MESSAGES];
int incomingMessageCount = 0; // Gerçekte kaç mesaj var
int nextMessageIndex = 0;     // Yeni mesajın yazılacağı döngüsel buffer indexi

// APRS-IS bağlantısı için client
WiFiClient aprsClient;
// Gelen veri tamponu
String aprsReadBuffer = "";
unsigned long lastAPRSClientConnectAttempt = 0;
const long APRS_CLIENT_RECONNECT_INTERVAL = 30000; // 30 saniyede bir bağlantı denemesi


// ─────────────────────────────────────────────
//  WebServer
// ─────────────────────────────────────────────
ESP8266WebServer server(80);

// ─────────────────────────────────────────────
//  EEPROM yardımcı
// ─────────────────────────────────────────────
void eepromWriteString(int addr, const String &s, int maxLen) {
  int len = s.length();
  if (len >= maxLen) len = maxLen - 1; // Null sonlandırma için yer bırak
  for (int i = 0; i < len; i++)  EEPROM.write(addr + i, s[i]);
  for (int i = len; i < maxLen; i++) EEPROM.write(addr + i, 0); // Kalanı sıfırla
}

String eepromReadString(int addr, int maxLen) {
  String s;
  for (int i = 0; i < maxLen; i++) {
    char c = EEPROM.read(addr + i);
    if (c == 0) break; // Null karakteri görünce dur
    s += c;
  }
  return s;
}

// ─────────────────────────────────────────────
//  Config yükleme / kaydetme
// ─────────────────────────────────────────────
bool loadConfig() {
  EEPROM.begin(EEPROM_SIZE); // EEPROM'u başlat
  // magic kontrol
  for (int i = 0; i < 4; i++) {
    if (EEPROM.read(MAGIC_ADDR + i) != MAGIC[i]) return false;
  }

  savedWifiCount = EEPROM.read(WIFI_CNT_ADDR);
  if (savedWifiCount > MAX_WIFI) savedWifiCount = 0; // Geçersiz sayım

  for (int i = 0; i < savedWifiCount; i++) {
    int base = WIFI_BASE + i * WIFI_BLOCK;
    savedWifi[i].ssid = eepromReadString(base,                WIFI_SSID_LEN);
    savedWifi[i].pass = eepromReadString(base + WIFI_SSID_LEN, WIFI_PASS_LEN);
  }

  aprsConf.mycall     = eepromReadString(APRS_CALL_ADDR,    APRS_CALL_LEN);
  aprsConf.aprspass   = eepromReadString(APRS_PASS_ADDR,    APRS_PASS_LEN);
  aprsConf.symbol_str = eepromReadString(APRS_SYMBOL_ADDR,  APRS_SYMBOL_LEN);
  aprsConf.comment    = eepromReadString(APRS_COMMENT_ADDR, APRS_COMMENT_LEN);
  aprsConf.message    = eepromReadString(APRS_MSG_ADDR,     APRS_MSG_LEN);
  aprsConf.aprshost   = eepromReadString(APRS_HOST_ADDR,    APRS_HOST_LEN);
  aprsConf.latitude   = eepromReadString(APRS_LAT_ADDR,     APRS_LAT_LEN);
  aprsConf.longitude  = eepromReadString(APRS_LON_ADDR,     APRS_LON_LEN);
  // port: big-endian uint16
  aprsConf.aprsport   = ((uint16_t)EEPROM.read(APRS_PORT_ADDR) << 8)
                      |  (uint16_t)EEPROM.read(APRS_PORT_ADDR + 1);
  if (aprsConf.aprsport == 0) aprsConf.aprsport = 14580; // default APRS-IS port
  return true;
}

void saveConfig() {
  // magic
  for (int i = 0; i < 4; i++) EEPROM.write(MAGIC_ADDR + i, MAGIC[i]);

  // wifi listesi
  EEPROM.write(WIFI_CNT_ADDR, savedWifiCount);
  for (int i = 0; i < MAX_WIFI; i++) {
    int base = WIFI_BASE + i * WIFI_BLOCK;
    if (i < savedWifiCount) {
      eepromWriteString(base,                savedWifi[i].ssid, WIFI_SSID_LEN);
      eepromWriteString(base + WIFI_SSID_LEN, savedWifi[i].pass, WIFI_PASS_LEN);
    } else {
      // Kayıtlı olmayanları temizle
      eepromWriteString(base,                "",  WIFI_SSID_LEN);
      eepromWriteString(base + WIFI_SSID_LEN, "", WIFI_PASS_LEN);
    }
  }

  // APRS
  eepromWriteString(APRS_CALL_ADDR,    aprsConf.mycall,     APRS_CALL_LEN);
  eepromWriteString(APRS_PASS_ADDR,    aprsConf.aprspass,    APRS_PASS_LEN);
  eepromWriteString(APRS_SYMBOL_ADDR,  aprsConf.symbol_str,  APRS_SYMBOL_LEN);
  eepromWriteString(APRS_COMMENT_ADDR, aprsConf.comment,    APRS_COMMENT_LEN);
  eepromWriteString(APRS_MSG_ADDR,     aprsConf.message,    APRS_MSG_LEN);
  eepromWriteString(APRS_HOST_ADDR,    aprsConf.aprshost,   APRS_HOST_LEN);
  eepromWriteString(APRS_LAT_ADDR,     aprsConf.latitude,   APRS_LAT_LEN);
  eepromWriteString(APRS_LON_ADDR,     aprsConf.longitude,  APRS_LON_LEN);
  // port: big-endian uint16
  EEPROM.write(APRS_PORT_ADDR,     (aprsConf.aprsport >> 8) & 0xFF);
  EEPROM.write(APRS_PORT_ADDR + 1,  aprsConf.aprsport       & 0xFF);

  EEPROM.commit(); // Değişiklikleri EEPROM'a yaz
}

// ─────────────────────────────────────────────
//  URL-decode
// ─────────────────────────────────────────────
// Web sunucusundan gelen URL encode edilmiş veriyi decode eder.
String urlDecode(const String &src) {
  String res;
  char c;
  for (size_t i = 0; i < src.length(); i++) {
    c = src[i];
    if (c == '+') { res += ' '; continue; } // '+' boşluk demektir
    if (c == '%' && i + 2 < src.length()) { // '%XX' hex değeri
      char h = 0;
      for (int j = 1; j < 3; j++) {
        h <<= 4;
        char x = src[i + j];
        if (x >= '0' && x <= '9') h += x - '0';
        else if (x >= 'a' && x <= 'f') h += x - 'a' + 10;
        else if (x >= 'A' && x <= 'F') h += x - 'A' + 10;
      }
      res += h;
      i += 2; // '%' ve iki hex karakteri atla
    } else {
      res += c;
    }
  }
  return res;
}

// ─────────────────────────────────────────────
//  HTML – ortak parçalar
// ─────────────────────────────────────────────
static const char HTML_HEAD[] PROGMEM =
R"(<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP01 MİNİ APRS HAVA İSTASYONU</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh;padding:16px}
.wrap{max-width:520px;margin:0 auto}
h1{text-align:center;font-size:1.4rem;color:#38bdf8;margin-bottom:4px;letter-spacing:1px}
.sub{text-align:center;font-size:.75rem;color:#64748b;margin-bottom:18px}
nav{display:flex;justify-content:center;gap:6px;margin-bottom:18px;flex-wrap:wrap}
nav a{background:#1e293b;border:1px solid #334155;color:#94a3b8;padding:6px 14px;border-radius:6px;text-decoration:none;font-size:.82rem;transition:.2s}
nav a:hover,nav a.act{background:#38bdf8;color:#0f172a;border-color:#38bdf8;font-weight:600}
.card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:18px;margin-bottom:14px}
.card h2{font-size:.95rem;color:#38bdf8;margin-bottom:10px;border-bottom:1px solid #334155;padding-bottom:6px}
label{display:block;font-size:.78rem;color:#94a3b8;margin-bottom:3px;margin-top:10px}
input,select{width:100%;padding:8px 10px;border-radius:6px;border:1px solid #475569;background:#0f172a;color:#e2e8f0;font-size:.85rem}
input:focus{outline:none;border-color:#38bdf8}
.row{display:flex;gap:8px}
.row input{flex:1}
.btn{display:inline-block;padding:7px 16px;border-radius:6px;border:none;cursor:pointer;font-size:.82rem;font-weight:600;transition:.2s}
.btn-primary{background:#38bdf8;color:#0f172a}.btn-primary:hover{background:#7dd3fc}
.btn-danger{background:#ef4444;color:#fff}.btn-danger:hover{background:#dc2626}
.btn-success{background:#22c55e;color:#fff}.btn-success:hover{background:#16a34a}
.btn-warn{background:#f59e0b;color:#0f172a}.btn-warn:hover{background:#fbbf24}
.btn-full{width:100%;margin-top:14px}
.wifi-item{display:flex;align-items:center;justify-content:space-between;background:#0f172a;border-radius:6px;padding:7px 10px;margin-bottom:6px}
.wifi-item span{font-size:.82rem}
.status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
.dot-green{background:#22c55e}.dot-red{background:#ef4444}.dot-yellow{background:#f59e0b}
.info-row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #334155;font-size:.82rem}
.info-row:last-child{border:none}
.info-row span:first-child{color:#64748b}
.info-row span:last-child{color:#e2e8f0;font-weight:500}
.note{font-size:.72rem;color:#64748b;margin-top:6px;font-style:italic}
.scan-list{max-height:140px;overflow-y:auto;margin-top:6px}
.scan-item{background:#0f172a;border-radius:5px;padding:5px 8px;margin-bottom:3px;font-size:.78rem;cursor:pointer;border:1px solid transparent;transition:.15s}
.scan-item:hover{border-color:#38bdf8}
.hidden{display:none}
.wifi-item.highlight-unread {
    border-left: 3px solid #f59e0b; /* Sarı vurgu */
}
</style>
</head>
<body><div class="wrap">
<h1>👤 ESP01 MİNİ APRS HAVA İSTASYONU</h1>
<p class="sub">Kablosuz APRS Veri Gönderici</p>
)";

static const char HTML_FOOT[] PROGMEM = R"(</div></body></html>)";

// Nav bar oluştırma yardımcısı  (active: 0=Ana 1=WiFi 2=APRS 3=Mesaj 4=Gelen Mesajlar)
String buildNav(int active) {
  String n = "<nav>";
  n += "<a href=\"/\""    + String(active==0 ? " class=\"act\"" : "") + ">Ana Sayfa</a>";
  n += "<a href=\"/wifi\"" + String(active==1 ? " class=\"act\"" : "") + ">WiFi</a>";
  n += "<a href=\"/aprs\"" + String(active==2 ? " class=\"act\"" : "") + ">APRS</a>";
  n += "<a href=\"/message\"" + String(active==3 ? " class=\"act\"" : "") + ">Mesaj Gönder</a>";
  n += "<a href=\"/inbox\"" + String(active==4 ? " class=\"act\"" : "") + ">Gelen Mesajlar (" + String(incomingMessageCount) + ")</a>";
  n += "</nav>";
  return n;
}

// ─────────────────────────────────────────────
//  Sayfalar
// ─────────────────────────────────────────────

// --- Ana Sayfa (/) ---
void handleIndex() {
  String page;
  page.reserve(2500); // Hafıza tahsisini biraz artırdık
  page += FPSTR(HTML_HEAD);

  // Nav
  page += buildNav(0);

  // BMP280 verilerini oku (sadece sensör başlatıldıysa)
  float sicaklik = NAN;
  float basinc_hPa = NAN;
  if (bmp280_initialized) {
    sicaklik = bmp.readTemperature();
    basinc_hPa = bmp.readPressure() / 100.0F; // Pa'dan hPa'ya çevir
  }

  // ─── Durum kartı ───
  page += "<div class=\"card\"><h2>📶 Bağlantı Durumu</h2>";
  bool connected = (WiFi.status() == WL_CONNECTED);
  page += "<div class=\"info-row\"><span>Durum</span><span>";
  if (connected) {
    page += "<span class=\"status-dot dot-green\"></span>Bağlı";
  } else {
    page += "<span class=\"status-dot dot-red\"></span>Bağlı Değil (AP Mod)";
  }
  page += "</span></div>";

  if (connected) {
    page += "<div class=\"info-row\"><span>Ağ Adı</span><span>" + WiFi.SSID() + "</span></div>";
    page += "<div class=\"info-row\"><span>IP Adresi</span><span>" + WiFi.localIP().toString() + "</span></div>";
    page += "<div class=\"info-row\"><span>Signal</span><span>" + String(WiFi.RSSI()) + " dBm</span></div>";
  } else {
    page += "<div class=\"info-row\"><span>AP Adı</span><span>ESP01_APRS</span></div>";
    page += "<div class=\"info-row\"><span>AP IP</span><span>192.168.4.1</span></div>";
  }

  // BMP280 Sensör Durumu ve Verileri
  page += "<div class=\"info-row\"><span>BMP280 Sensör</span><span>";
  if (bmp280_initialized) {
      page += "<span class=\"status-dot dot-green\"></span>Algılandı";
      // Sıcaklık ve basınç değerlerini ekle
      if (!isnan(sicaklik) && !isnan(basinc_hPa)) {
          char sensorDataBuffer[50]; // Yeterli büyüklükte bir buffer
          snprintf(sensorDataBuffer, sizeof(sensorDataBuffer), " (T:%.1f°C, P:%.0fhPa)", sicaklik, basinc_hPa);
          page += String(sensorDataBuffer);
      } else {
          page += " (Veri okunamadı)";
      }
  } else {
      page += "<span class=\"status-dot dot-red\"></span>Algılanmadı";
  }
  page += "</span></div>";

  page += "</div>"; // end card

  // ─── APRS Özet ───
  page += "<div class=\"card\"><h2>📈 APRS Özet</h2>";
  page += "<div class=\"info-row\"><span>Callsign</span><span>"
          + (aprsConf.mycall.isEmpty()     ? String("— Ayarlanmadı —") : aprsConf.mycall)     + "</span></div>";
  page += "<div class=\"info-row\"><span>Passcode</span><span>"
          + (aprsConf.aprspass.isEmpty()   ? String("— Ayarlanmadı —") : String("****"))       + "</span></div>";
  page += "<div class=\"info-row\"><span>Sembol</span><span>"
          + (aprsConf.symbol_str.isEmpty() ? String("— Ayarlanmadı —") : aprsConf.symbol_str) + "</span></div>";
  page += "<div class=\"info-row\"><span>Enlem</span><span>"
          + (aprsConf.latitude.isEmpty()   ? String("— Ayarlanmadı —") : aprsConf.latitude)   + "</span></div>";
  page += "<div class=\"info-row\"><span>Boylam</span><span>"
          + (aprsConf.longitude.isEmpty()  ? String("— Ayarlanmadı —") : aprsConf.longitude)  + "</span></div>";
  page += "<div class=\"info-row\"><span>Yorum</span><span>"
          + (aprsConf.comment.isEmpty()    ? String("— Yok —")             : aprsConf.comment)     + "</span></div>";
  page += "<div class=\"info-row\"><span>Mesaj</span><span>"
          + (aprsConf.message.isEmpty()    ? String("— Yok —")             : aprsConf.message)     + "</span></div>";
  page += "<div class=\"info-row\"><span>Sunucu</span><span>"
          + (aprsConf.aprshost.isEmpty()   ? String("france.aprs2.net")                : aprsConf.aprshost)    + "</span></div>";
  page += "<div class=\"info-row\"><span>Port</span><span>" + String(aprsConf.aprsport) + "</span></div>";
  page += "</div>"; // end card

  // ─── Kayıtlı WiFi sayısı ───
  page += "<div class=\"card\"><h2>📶 Kayıtlı Ağlar</h2>";
  if (savedWifiCount == 0) {
    page += "<p class=\"note\">Henüz kayıtlı ağ yok. WiFi sayfasından ekleyin.</p>";
  } else {
    for (int i = 0; i < savedWifiCount; i++) {
      page += "<div class=\"wifi-item\"><span><span class=\"status-dot dot-yellow\"></span>" +
              savedWifi[i].ssid + "</span></div>";
    }
  }
  page += "</div>"; // end card

  // ─── Hafıza Durumu Kartı (Güncellenmiş) ───
  page += "<div class=\"card\"><h2>💾 Hafıza Durumu</h2>";
  page += "<div class=\"info-row\"><span>Free Heap (RAM)</span><span>" + String(ESP.getFreeHeap()) + " bayt</span></div>";
  page += "<div class=\"info-row\"><span>Flash Boyutu</span><span>" + String(ESP.getFlashChipSize() / 1024) + " KB</span></div>";
  page += "<div class=\"info-row\"><span>Sketch Boyutu</span><span>" + String(ESP.getSketchSize() / 1024) + " KB</span></div>";
  page += "<div class=\"info-row\"><span>Kalan Flash</span><span>" + String((ESP.getFlashChipSize() - ESP.getSketchSize()) / 1024) + " KB</span></div>";

  page += "</div>"; // end card

  // ─── Yeniden Başlat ───
  page += "<form method='POST' action='/restart'>"
          "<button class='btn btn-warn btn-full' type='submit'>🔄 Yeniden Başlat</button></form>";

  page += FPSTR(HTML_FOOT);
  server.send(200, "text/html; charset=utf-8", page);
}

// --- WiFi Sayfası (/wifi) ---
void handleWifi() {
  String page;
  page.reserve(3072); // Hafıza tahsisi optimize etmek için
  page += FPSTR(HTML_HEAD);

  page += buildNav(1);

  // ─── Kayıtlı ağlar ───
  page += "<div class=\"card\"><h2>📶 Kayıtlı Wi-Fi Ağları</h2>";
  if (savedWifiCount == 0) {
    page += "<p class=\"note\">Kayıtlı ağ yok.</p>";
  }
  for (int i = 0; i < savedWifiCount; i++) {
    page += "<div class=\"wifi-item\">"
            "<span><span class=\"status-dot dot-yellow\"></span>" + savedWifi[i].ssid + "</span>"
            "<form method='POST' action='/wifi/delete' style='display:inline'>"
            "<input type='hidden' name='idx' value='" + String(i) + "'>"
            "<button class='btn btn-danger' type='submit'>Sil</button></form>"
            "</div>";
  }
  page += "</div>"; // end card

  // ─── Yakında ağlar (scan) ───
  page += "<div class=\"card\"><h2>🔍 Yakın Çevredeki Ağlar</h2>";
  int n = WiFi.scanNetworks();
  if (n > 0) {
    page += "<div class=\"scan-list\">";
    for (int i = 0; i < n; i++) {
      String ssid_i = WiFi.SSID(i);
      // tek tırnaklara kaçış (JS onclick için)
      String ssid_safe = ssid_i;
      ssid_safe.replace("'", "\\'");
      String enc = (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " 🔓" : " 🔒";
      page += "<div class=\"scan-item\" onclick=\"fillScan('" + ssid_safe + "')\">" +
              ssid_i + " (" + String(WiFi.RSSI(i)) + " dBm)" + enc + "</div>";
    }
    page += "</div>";
  } else {
    page += "<p class=\"note\">Ağ bulunamadı.</p>";
  }
  page += "</div>"; // end card

  // ─── Yeni ağ ekleme formu ───
  page += "<div class=\"card\"><h2>+ Yeni Ağ Ekle</h2>"
          "<label>Ağ Adı (SSID)</label>"
          "<input type='text' id='ssid' name='ssid' autocomplete='off'>"
          "<label>Şifre</label>"
          "<input type='password' id='pass' name='pass' autocomplete='off'>"
          "<form method='POST' action='/wifi'>"
          "<input type='hidden' id='ssid_h' name='ssid' value=''>"
          "<input type='hidden' id='pass_h' name='pass' value=''>"
          "<button class='btn btn-primary btn-full' type='submit' onclick='return submitWifi()'>Kaydet &amp; Yeniden Başlat</button></form>"
          "<p class=\"note\">Ağı kaydettikten sonra cihaz yeniden başlayacak ve kayıtlı ağlara bağlanacak.</p>"
          "</div>"; // end card

  // JS – scan itemine tıkayınca SSID dolsun
  page += "<script>"
          "function fillScan(s){document.getElementById('ssid').value=s;}"
          "function submitWifi(){"
          "  var s=document.getElementById('ssid').value;"
          "  var p=document.getElementById('pass').value;"
          "  if(!s){alert('Ağ adı girelim');return false;}"
          "  document.getElementById('ssid_h').value=s;"
          "  document.getElementById('pass_h').value=p;"
          "  return true;}"
          "</script>";

  page += FPSTR(HTML_FOOT);
  server.send(200, "text/html; charset=utf-8", page);
}

// --- APRS Sayfası (/aprs) ---
void handleAprs() {
  String page;
  page.reserve(3072); // Hafıza tahsisi optimize etmek için
  page += FPSTR(HTML_HEAD);

  page += buildNav(2);

  // ─── Kimlik kartı ───
  page += "<div class=\"card\"><h2>📈 Kimlik</h2>"
          "<form method='POST' action='/aprs'>"
          "<label>Callsign  <small>(Örn: TA3OER-4)</small></label>"
          "<input type='text' name='mycall' value='" + aprsConf.mycall + "' maxlength='9' autocomplete='off'>"

          "<label>Passcode  <small>(5 rakam, http://www.aprs-is.net/passcode.aspx)</small></label>"
          "<input type='text' name='aprspass' value='" + aprsConf.aprspass + "' maxlength='5' inputmode='numeric' autocomplete='off'>";

  // ─── Konum kartı ───
  page += "<hr style='border-color:#334155;margin:14px 0'>"
          "<label style='color:#38bdf8;font-size:.82rem;margin-top:0'>🌐 Konum</label>"

          "<label>Enlem  <small>(Örn: 4019.25N)</small></label>"
          "<input type='text' name='latitude' value='" + aprsConf.latitude + "' maxlength='12' autocomplete='off'>"

          "<label>Boylam  <small>(Örn: 02624.15E)</small></label>"
          "<input type='text' name='longitude' value='" + aprsConf.longitude + "' maxlength='13' autocomplete='off'>"

          "<label>Sembol  <small>(/ + karakter, örn: /I /O /T /r)</small></label>"
          "<input type='text' name='symbol_str' value='" + aprsConf.symbol_str + "' maxlength='2' autocomplete='off'>";

  // ─── Metin kartı ───
  page += "<hr style='border-color:#334155;margin:14px 0'>"
          "<label style='color:#38bdf8;font-size:.82rem;margin-top:0'>📝 Metinler</label>"

          "<label>Yorum  <small>(Comment – 31 kar, sensör bilgileri eklenebilir)</small></label>"
          "<input type='text' name='comment' value='" + aprsConf.comment + "' maxlength='31' autocomplete='off'>"

          "<label>Mesaj  <small>(Status – 41 kar, boş bırakılabilir)</small></label>"
          "<input type='text' name='message' value='" + aprsConf.message + "' maxlength='41' autocomplete='off'>";

  // ─── Sunucu kartı ───
  page += "<hr style='border-color:#334155;margin:14px 0'>"
          "<label style='color:#38bdf8;font-size:.82rem;margin-top:0'>🌐 Sunucu</label>"

          "<label>APRS-IS Host  <small>(default: france.aprs2.net)</small></label>"
          "<input type='text' name='aprshost' value='" + aprsConf.aprshost + "' maxlength='63' autocomplete='off'>"

          "<label>Port  <small>(default: 14580)</small></label>"
          "<input type='text' name='aprsport' value='" + String(aprsConf.aprsport) + "' maxlength='5' inputmode='numeric' autocomplete='off'>"

          "<button class='btn btn-success btn-full' type='submit'>Kaydet</button></form>"

          "<p class=\"note\">"
          "Sembol: /I=İstasyon &nbsp; /O=Balon &nbsp; /T=Fırtına &nbsp; /r=Antenna<br>"
          "Konum: <strong>DDMM.MMN</strong> (enlem) + <strong>DDDMM.MME</strong> (boylam)"
          "</p></div>"; // end card

  page += FPSTR(HTML_FOOT);
  server.send(200, "text/html; charset=utf-8", page);
}

// --- Mesaj Gönderme Sayfası (/message) ---
void handleMessage() {
  String page;
  page.reserve(2048); // Hafıza tahsisi
  page += FPSTR(HTML_HEAD);

  page += buildNav(3); // '3' yeni mesaj sayfası için aktif navigasyon

  page += "<div class=\"card\"><h2>✉️ Manuel Mesaj Gönder</h2>";

  if (aprsConf.mycall.isEmpty() || aprsConf.aprspass.isEmpty()) {
    page += "<p class=\"note\" style='color:#ef4444;font-weight:600'>APRS Callsign veya Passcode ayarlanmamış! Mesaj gönderilemez.</p>";
  }

  page += "<form method='POST' action='/message'>";
  page += "<label>Kimden (MyCall)</label>";
  page += "<input type='text' value='" + (aprsConf.mycall.isEmpty() ? String("Ayarlanmadı") : aprsConf.mycall) + "' readonly>"; // Sadece okunur MyCall

  page += "<label>Kime (To Call)</label>";
  page += "<input type='text' name='to_call' maxlength='9' autocomplete='off' required>"; // Maks. 9 karakter, APRS standardı

  page += "<label>Mesaj (Max 67 Karakter)</label>";
  page += "<input type='text' name='manual_message' maxlength='67' autocomplete='off' required>"; // Maks. 67 karakter

  if (aprsConf.mycall.isEmpty() || aprsConf.aprspass.isEmpty()) {
      page += "<button class='btn btn-primary btn-full' type='submit' disabled>Gönder</button>"; // Callsign yoksa devre dışı
  } else {
      page += "<button class='btn btn-primary btn-full' type='submit'>Gönder</button>";
  }

  page += "<p class=\"note\">Gönderdiğiniz mesaj APRS-IS ağı üzerinden belirtilen Callsign'a ulaşacaktır.</p>";
  // YENİ EKLENEN SATIR BAŞLANGICI
  page += "<p class=\"note\"><strong>#APRSThursday'a katılmak için:</strong><br>Kime: <code>ANSRVR</code> yazıp, Mesaj: <code>CQ HOTG Merhaba Dunya, 73</code> gönderin.</p>";
  page += "<p class=\"note\">Bu mesaj, HOTG ( “Hams On The Go” anlamına gelir ) adlı ANSRVR grubuna katılmış olan tüm çağrı işaretlerine gönderilecektir. Ayrıca, 12 saatlik süre boyunca gruba gönderilen tüm mesajları da alırsınız.</p>";
  page += "<p class=\"note\"><strong>Ağdan Ayrılma (Unsubscribe)</strong> yerel APRS RF ağındaki trafiği azaltmak için #APRSThursday ağından çıkın (artık sonraki mesajları almazsınız). Bunun için şu mesajı gönderin: <br>Kime: <code>ANSRVR</code> yazıp, Mesaj: <code>U HOTG</code> gönderin.</p>";
  page += "<p class=\"note\"><strong>#APRSThursday için alternatif bir kayıt yöntemi:</strong><br>Kime: <code>APRSPH</code> yazıp, Mesaj: <code>HOTG Merhaba Dunya, 73</code> gönderin.</p>";
  page += "<p class=\"note\">Bu yöntem, #APRSThursday APRS RF ağındaki yükü azaltmak isteyen kullanıcılar içindir.</p>";
  
  // YENİ EKLENEN SATIR SONUÇ
  page += "</div>"; // end card
  page += FPSTR(HTML_FOOT);
  server.send(200, "text/html; charset=utf-8", page);
}

// --- Gelen Mesajlar Sayfası (/inbox) ---
void handleInbox() {
  String page;
  page.reserve(4096); // Daha fazla veri gösterebileceği için reserve'i artırabiliriz
  page += FPSTR(HTML_HEAD);

  page += buildNav(4); // '4' gelen mesajlar sayfası için aktif navigasyon

  page += "<div class=\"card\"><h2>📥 Gelen Mesajlar</h2>";
  if (incomingMessageCount == 0) {
    page += "<p class=\"note\">Henüz gelen mesaj yok.</p>";
  } else {
    // Mesajları sondan başa doğru göster (en yeni en üstte)
    for (int i = 0; i < incomingMessageCount; i++) {
        // Döngüsel buffer'da doğru indeksi bul
        int idx = (nextMessageIndex - 1 - i + MAX_INCOMING_MESSAGES) % MAX_INCOMING_MESSAGES;
        ReceivedAPRSMessage msg = incomingMessages[idx];

        // Okunmamış mesajları vurgula
        String itemClass = msg.read ? "wifi-item" : "wifi-item highlight-unread";
        String dotClass = msg.read ? "dot-green" : "dot-yellow";

        page += "<div class=\"" + itemClass + "\">";
        page += "<span><span class=\"status-dot " + dotClass + "\"></span>";
        page += "<strong>" + msg.sender + "</strong> (" + msg.timestamp + ")<br>";
        page += msg.message;
        page += "</span>";
        // Okunmadıysa 'Okundu' olarak işaretleme butonu
        if (!msg.read) {
          page += "<form method='POST' action='/inbox/read' style='display:inline'>";
          page += "<input type='hidden' name='idx' value='" + String(idx) + "'>";
          page += "<button class='btn btn-success btn-sm' type='submit' style='margin-left:8px'>Okundu</button>";
          page += "</form>";
        }
        page += "</div>";
    }
  }
  // Tüm mesajları temizleme butonu
  page += "<form method='POST' action='/inbox/clear'>";
  page += "<button class='btn btn-danger btn-full' type='submit' style='margin-top:10px;'>Tümünü Temizle</button>";
  page += "</form>";
  page += "</div>"; // end card
  page += FPSTR(HTML_FOOT);
  server.send(200, "text/html; charset=utf-8", page);
}


// ─────────────────────────────────────────────
//  POST Handlers
// ─────────────────────────────────────────────
void handlePostWifi() {
  if (savedWifiCount >= MAX_WIFI) {
    server.send(400, "text/plain; charset=utf-8", "Maksimum 4 ağ kayedilebilir. Önce birini silin.");
    return;
  }
  String ssid = urlDecode(server.arg("ssid"));
  String pass = urlDecode(server.arg("pass"));
  if (ssid.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "SSID boş!");
    return;
  }
  // Aynı SSID zaten kayıtlı mı kontrol et
  for (int i = 0; i < savedWifiCount; i++) {
    if (savedWifi[i].ssid == ssid) {
      server.send(400, "text/plain; charset=utf-8", "Bu ağ zaten kayıtlı!");
      return;
    }
  }

  savedWifi[savedWifiCount].ssid = ssid;
  savedWifi[savedWifiCount].pass = pass;
  savedWifiCount++;
  saveConfig();
  Serial.println("[WiFi] Ağ eklendi: " + ssid);
  // redirect + yeniden başla
  server.sendHeader("Location", "/"); // Ana sayfaya yönlendir
  server.send(302, "text/html; charset=utf-8", "<p>Yeniden başlatılıyor…</p>");
  delay(500);
  ESP.restart();
}

void handlePostWifiDelete() {
  int idx = server.arg("idx").toInt();
  if (idx >= 0 && idx < savedWifiCount) {
    Serial.println("[WiFi] Silindi: " + savedWifi[idx].ssid);
    for (int i = idx; i < savedWifiCount - 1; i++) savedWifi[i] = savedWifi[i + 1];
    savedWifiCount--;
    // Son elemanı temizle (EEPROM'da çöp kalmaması için)
    savedWifi[savedWifiCount].ssid = "";
    savedWifi[savedWifiCount].pass = "";
    saveConfig();
  }
  server.sendHeader("Location", "/wifi"); // WiFi sayfasına geri yönlendir
  server.send(302); // Sadece yönlendirme mesajı gönder
}

void handlePostAprs() {
  aprsConf.mycall     = urlDecode(server.arg("mycall"));
  aprsConf.aprspass   = urlDecode(server.arg("aprspass"));
  aprsConf.symbol_str = urlDecode(server.arg("symbol_str"));
  aprsConf.comment    = urlDecode(server.arg("comment"));
  aprsConf.message    = urlDecode(server.arg("message"));
  aprsConf.latitude   = urlDecode(server.arg("latitude"));
  aprsConf.longitude  = urlDecode(server.arg("longitude"));

  // host – boş kalırsa default
  String hostTmp = urlDecode(server.arg("aprshost"));
  aprsConf.aprshost = hostTmp.isEmpty() ? String("france.aprs2.net") : hostTmp;
  if (aprsConf.aprshost.length() > APRS_HOST_LEN - 1) aprsConf.aprshost = aprsConf.aprshost.substring(0, APRS_HOST_LEN - 1); // Max uzunluk

  // port – boş veya geçersiz → default 14580
  String portTmp = urlDecode(server.arg("aprsport"));
  uint16_t p = portTmp.toInt();
  aprsConf.aprsport = (p > 0 && p < 65535) ? p : 14580; // Geçerli port aralığı kontrolü

  saveConfig();
  Serial.println("[APRS] Kaydetildi – Callsign: " + aprsConf.mycall
                 + "  Host: " + aprsConf.aprshost
                 + ":" + String(aprsConf.aprsport));
  server.sendHeader("Location", "/aprs"); // APRS sayfasına geri yönlendir
  server.send(302);
}

void handlePostMessage() {
  if (aprsConf.mycall.isEmpty() || aprsConf.aprspass.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "APRS Callsign veya Passcode ayarlanmamış! Mesaj gönderilemez.");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    server.send(400, "text/plain; charset=utf-8", "WiFi bağlantısı yok. Mesaj gönderilemez.");
    return;
  }

  String toCall = urlDecode(server.arg("to_call"));
  String manualMessage = urlDecode(server.arg("manual_message"));

  if (toCall.isEmpty() || manualMessage.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "Kime veya Mesaj boş bırakılamaz!");
    return;
  }

  // toCall ve manualMessage için uzunluk kontrolleri
  if (toCall.length() > APRS_CALL_LEN - 1) toCall = toCall.substring(0, APRS_CALL_LEN - 1); // APRS Callsign max 9 karakter
  if (manualMessage.length() > 67) manualMessage = manualMessage.substring(0, 67); // APRS Mesaj max 67 karakter

  Serial.println("[APRS Mesaj] Gönderiliyor -> Kimden: " + aprsConf.mycall + ", Kime: " + toCall + ", Mesaj: " + manualMessage);

  // APRS mesajını gönderen yeni fonksiyonu çağır
  bool success = sendAPRSMessage(aprsConf.mycall, aprsConf.aprspass, toCall, manualMessage);

  if (success) {
    String page;
    page.reserve(2048);
    page += FPSTR(HTML_HEAD);
    page += buildNav(3); // Mesaj Gönder sayfası aktif
    page += "<div class=\"card\"><h2>✉️ Manuel Mesaj Gönder</h2>";
    page += "<div style='background:#d4edda; border:1px solid #c3e6cb; border-radius:4px; padding:15px; margin-bottom:15px; color:#155724;'>";
    page += "<h3>✓ Mesaj başarıyla gönderildi!</h3>";
    page += "<p><b>Kime:</b> " + toCall + "</p>";
    page += "<p><b>Mesaj:</b> " + manualMessage + "</p>";

    // APRSThursday özel bilgi (eğer ANSRVR'a gönderildiyse)
    if (toCall.equalsIgnoreCase("ANSRVR")) {
        page += "<div style='margin-top:15px; padding:15px; background:#cfe2ff; border:1px solid #0d6efd; border-radius:3px;'>";
        page += "<h4 style='margin:0 0 10px 0; color:#084298;'>📡 APRSThursday Katılım</h4>";
        page += "<p style='margin:5px 0;'>ANSRVR'a mesajınız iletildi!</p>";
        if (manualMessage.startsWith("CQ HOTG") || manualMessage.startsWith("cq hotg")) {
            page += "<p style='margin:5px 0; color:#0f5132;'>✓ Mesajınız doğru formatta (<code>CQ HOTG</code>)</p>";
        } else {
            page += "<p style='margin:5px 0; color:#664d03; background:#fff3cd; padding:8px; border-radius:3px;'>";
            page += "⚠ Not: APRSThursday mesajları '<code>CQ HOTG</code>' ile başlamalı.<br>";
            page += "Örnek: <code>CQ HOTG Merhaba Dunya, 73</code></p>";
        }
        page += "</div>";
    }
    page += "<p style='margin-top:15px;'>";
    page += "<a href='/message' class='btn btn-warn'>← Yeni Mesaj Gönder</a> ";
    page += "<a href='/inbox' class='btn btn-primary'>Gelen Kutusu →</a>";
    page += "</p>";
    page += "</div>"; // end success box
    page += FPSTR(HTML_FOOT);
    server.send(200, "text/html; charset=utf-8", page);
  } else {
    String page;
    page.reserve(2048);
    page += FPSTR(HTML_HEAD);
    page += buildNav(3); // Mesaj Gönder sayfası aktif
    page += "<div class=\"card\"><h2>✉️ Manuel Mesaj Gönder</h2>";
    page += "<div style='background:#f8d7da; border:1px solid #f5c6cb; border-radius:4px; padding:15px; margin-bottom:15px; color:#721c24;'>";
    page += "<h3>✖ Mesaj gönderilirken hata oluştu!</h3>";
    page += "<p>Lütfen daha sonra tekrar deneyin veya ayarlarınızı kontrol edin.</p>";
    page += "<p><a href='/message' class='btn btn-warn'>Tekrar Dene</a></p>";
    page += "</div>"; // end error box
    page += FPSTR(HTML_FOOT);
    server.send(500, "text/html; charset=utf-8", page);
  }
  // Geri dönüşte bir miktar beklemek iyi olabilir, mesaj gönderiminin tamamlanmasını sağlamak için.
  delay(1000);
}

void handlePostInboxRead() {
  int idx = server.arg("idx").toInt();
  // Geçerli mesaj sayısından az ve sender'ı boş olmayan (yani var olan) bir mesajı kontrol et
  // nextMessageIndex'i kullanarak döngüsel buffer'ın mantığını da dikkate almalıyız.
  // Bu durumda, sadece `idx`'in buffer sınırları içinde olup olmadığını kontrol etmek daha kolay.
  if (idx >= 0 && idx < MAX_INCOMING_MESSAGES && !incomingMessages[idx].sender.isEmpty()) {
      incomingMessages[idx].read = true;
      Serial.println("[Mesaj Kutu] Mesaj okundu olarak işaretlendi: " + incomingMessages[idx].sender);
  }
  server.sendHeader("Location", "/inbox"); // Gelen mesajlar sayfasına geri yönlendir
  server.send(302);
}

void handlePostInboxClear() {
  // Tüm mesajları temizle
  for (int i = 0; i < MAX_INCOMING_MESSAGES; i++) {
    incomingMessages[i].sender = "";
    incomingMessages[i].message = "";
    incomingMessages[i].timestamp = "";
    incomingMessages[i].read = true;
  }
  incomingMessageCount = 0;
  nextMessageIndex = 0;
  Serial.println("[Mesaj Kutu] Tüm mesajlar temizlendi.");
  server.sendHeader("Location", "/inbox"); // Gelen mesajlar sayfasına geri yönlendir
  server.send(302);
}


void handlePostRestart() {
  server.send(200, "text/html; charset=utf-8", "<h2>Yeniden başlatılıyor…</h2>");
  delay(500);
  ESP.restart();
}

// ─────────────────────────────────────────────
//  WiFi bağlantı yönetimi
// ─────────────────────────────────────────────
bool tryConnectWifi() {
  if (savedWifiCount == 0) return false;
  WiFi.mode(WIFI_STA); // Station moduna geç
  WiFi.disconnect(true); // Önceki bağlantıları ve ayarları unut
  delay(300);

  // Scan yap, kayıtlı ağlardan güçlüsünü seç
  Serial.println("[WiFi] Ağlar taranıyor...");
  int n = WiFi.scanNetworks();
  int bestIdx  = -1;
  int bestRSSI = -100; // En kötü RSSI değeri
  if (n == 0) {
      Serial.println("[WiFi] Hiç ağ bulunamadı.");
  } else {
      Serial.print("[WiFi] Tarama tamamlandı, " + String(n) + " ağ bulundu.");
      for (int s = 0; s < n; s++) {
        String scanned = WiFi.SSID(s);
        int rssi = WiFi.RSSI(s);
        Serial.printf("  - %s (%d dBm)\n", scanned.c_str(), rssi);
        for (int i = 0; i < savedWifiCount; i++) {
          if (scanned == savedWifi[i].ssid && rssi > bestRSSI) {
            bestIdx  = i;
            bestRSSI = rssi;
          }
        }
      }
  }
  WiFi.scanDelete(); // Taramayı temizle

  if (bestIdx < 0) {
    Serial.println("[WiFi] Kayıtlı ağlardan hiçbiri çevrede bulunamadı.");
    return false;
  }

  Serial.println("[WiFi] Bağlanılıyor: " + savedWifi[bestIdx].ssid + " (RSSI: " + String(bestRSSI) + " dBm)");
  WiFi.begin(savedWifi[bestIdx].ssid.c_str(), savedWifi[bestIdx].pass.c_str());

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) { // 15 saniye zaman aşımı
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Başarıyla bağlandı → IP: " + WiFi.localIP().toString());
    return true;
  }
  Serial.println("[WiFi] Bağlantı başarısız. SSID: " + savedWifi[bestIdx].ssid);
  return false;
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP01_APRS", "aprs1234"); // AP şifresi: aprs1234
  delay(500); // AP'nin tam olarak başlaması için bekle
  Serial.println("[AP] Başlatıldı → IP: " + WiFi.softAPIP().toString() + " (Şifre: aprs1234)");
}

// ─────────────────────────────────────────────
//  APRS Gönderme Yardımcı Fonksiyonu
// ─────────────────────────────────────────────
// Manuel bir APRS mesajı gönderir.
bool sendAPRSMessage(const String& fromCall, const String& pass, const String& toCall, const String& message) {
  if (fromCall.isEmpty() || pass.isEmpty() || toCall.isEmpty() || message.isEmpty()) {
    Serial.println("[APRS Mesaj] Eksik bilgi: Kimden/Kime/Mesaj boş!");
    // Eksik bilgi durumunda hata mesajı döndür. Çağıran fonksiyon bunu işleyecek.
    // Web arayüzünde gösterilecek hata mesajı handlePostMessage'da oluşturulmalı.
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[APRS Mesaj] WiFi bağlantısı yok, mesaj gönderilemez.");
    return false;
  }

  String host = aprsConf.aprshost.isEmpty() ? String("france.aprs2.net") : aprsConf.aprshost;
  uint16_t port = (aprsConf.aprsport > 0) ? aprsConf.aprsport : 14580;

  // ============================================
  // APRSThursday Özel Kontrol
  // ============================================
  bool isAPRSThursday = false;
  if (toCall.equalsIgnoreCase("ANSRVR")) { // Büyük/küçük harf duyarsız karşılaştırma
    isAPRSThursday = true;
    Serial.println("[APRSThursday] ANSRVR'a mesaj gönderiliyor - APRSThursday modu aktif");
    if (!message.startsWith("CQ HOTG") && !message.startsWith("cq hotg")) {
      Serial.println("[APRSThursday] UYARI: Mesaj 'CQ HOTG' ile başlamıyor!");
    }
  }

  // ============================================
  // APRS Mesaj Formatı - Hedef Çağrı İşareti Padding
  // ============================================
  String toCall_formatted = toCall;

  // APRS standardı: Hedef çağrı işareti TAM 9 KARAKTER olmalı
  // Kısa ise boşluk ekle (padding)
  while (toCall_formatted.length() < 9) {
    toCall_formatted += " ";
  }

  // 9 karakterden uzunsa kes (ANSRVR = 6 karakter, padding ile 9 olur)
  if (toCall_formatted.length() > 9) {
    toCall_formatted = toCall_formatted.substring(0, 9);
  }

  // ============================================
  // Mesaj ID Oluştur
  // ============================================
  static int msgID = 1; // msgID statik olmalı ki her çağrıda değeri korunsun
  String msgIDstr = String(msgID++);
  if (msgID > 999) msgID = 1; // 1-999 arası döngü

  // ============================================
  // APRS Mesaj Paketi Oluştur
  // ============================================
  // Format: FROM_CALL>APRS,TCPIP*::TO_CALL  :Mesaj{msgID}
  //                                  ^^^^^^^
  //                               9 karakter (boşluklarla doldurulmuş)

  String aprsMsg = fromCall + ">APRS,TCPIP*::" + toCall_formatted + ":" + message + "{" + msgIDstr + "}";

  Serial.println("[Manuel Mesaj] APRS Paketi: " + aprsMsg);
  Serial.println("[Debug] toCall orijinal: '" + toCall + "' (uzunluk: " + String(toCall.length()) + ")");
  Serial.println("[Debug] toCall formatted: '" + toCall_formatted + "' (uzunluk: " + String(toCall_formatted.length()) + ")");


  // ============================================
  // APRS-IS Sunucusuna Bağlan
  // ============================================
  WiFiClient client;
  Serial.print("[APRS Mesaj] APRS-IS sunucusuna bağlanılıyor: " + host + ":" + String(port) + " ... ");
  if (!client.connect(host.c_str(), port)) {
    Serial.println("BAĞLANTI HATASI!");
    return false;
  }
  Serial.println("BAĞLANDI.");

  // ============================================
  // APRS-IS Login
  // ============================================
  String loginLine = "user " + fromCall + " pass " + pass + " vers ESP01_APRS_V4"; // Versiyon numarasını güncelleyebiliriz
  client.println(loginLine);
  Serial.println("[APRS Mesaj] Login: " + loginLine);
  delay(300); // Sunucunun yanıt vermesi için kısa bir bekleme

  // Login yanıtını kontrol etmek, daha sağlam bir sistem için faydalı olabilir,
  // ancak basit bir ESP01 için her zaman gerekli olmayabilir.
  // Burada orijinal kodun yapısına sadık kalarak sadece gönderim yapıyoruz.

  // ============================================
  // APRS Mesajını Gönder
  // ============================================
  client.println(aprsMsg);
  Serial.println("[APRS Mesaj] Paket: " + aprsMsg);

  delay(2000); // Paketlerin gönderilmesi ve sunucu tarafından işlenmesi için bekle
  client.stop(); // Bağlantıyı kapat
  Serial.println("[APRS Mesaj] Gönderim tamamlandı. Bağlantı kesildi.");
  return true;
}


// ─────────────────────────────────────────────
//  APRS gönderme (otomatik konum/durum raporları)
// ─────────────────────────────────────────────
void sendAPRS() {
  if (aprsConf.mycall.isEmpty() || aprsConf.aprspass.isEmpty()) {
    Serial.println("[APRS] Callsign veya passcode yok. APRS gönderimi atlandı.");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[APRS] WiFi bağlantısı yok. APRS gönderimi atlandı.");
      return;
  }


  // BMP280 verilerini oku
  float sicaklik = NAN; // Not a Number
  float basinc_hPa = NAN;
  // float yukseklik_m = NAN; // Rakımı hesaplamak için deniz seviyesi basıncı gerekli.

  if (bmp280_initialized) { // Sensör başarıyla başlatıldıysa veri oku
    sicaklik = bmp.readTemperature();
    basinc_hPa = bmp.readPressure() / 100.0F; // Pa'dan hPa'ya çevir
    // yukseklik_m = bmp.readAltitude(1013.25); // 1013.25 hPa deniz seviyesi basıncı
    Serial.printf("[BMP280] Sıcaklık: %.2f °C, Basınç: %.2f hPa\n", sicaklik, basinc_hPa);
  } else {
     Serial.println("[BMP280] Sensör başlatılamadı veya bulunamadı. Veri okunamadı.");
  }


  // host / port defaults
  String host = aprsConf.aprshost.isEmpty() ? String("france.aprs2.net") : aprsConf.aprshost;
  uint16_t port = (aprsConf.aprsport > 0) ? aprsConf.aprsport : 14580;

  WiFiClient client;
  Serial.print("[APRS Mesaj] APRS-IS sunucusuna bağlanılıyor: " + host + ":" + String(port) + " ... ");
  if (!client.connect(host.c_str(), port)) {
    Serial.println("BAĞLANTI HATASI!");
    return;
  }
  Serial.println("BAĞLANDI.");

  // ── Login ──
  String loginLine = "user " + aprsConf.mycall + " pass " + aprsConf.aprspass + " vers ESP01_APRS_V4";
  client.println(loginLine);
  Serial.println("[APRS] Login: " + loginLine);
  delay(300); // Sunucunun yanıt vermesi için kısa bir bekleme

  // ── Position report ──
  // Format:  =DDMM.MMN/DDDMM.MME<symbol><comment>
  if (!aprsConf.latitude.isEmpty() && !aprsConf.longitude.isEmpty()) {
    String sym = aprsConf.symbol_str.isEmpty() ? String("/I") : aprsConf.symbol_str;
    char symTable = sym.length() > 0 ? sym[0] : '/';
    char symCode  = sym.length() > 1 ? sym[1] : 'I';

    String fullComment = aprsConf.comment;

    // Sensör verilerini yoruma ekle (eğer okunduysa ve mevcut yoruma sığıyorsa)
    if (bmp280_initialized && !isnan(sicaklik) && !isnan(basinc_hPa)) {
        char tempSensorBuf[25]; // Sensör verisi için geçici buffer
        // Yorum alanı 32 karakterle sınırlıdır. Mevcut yorumu kontrol et
        int maxLenForSensorData = APRS_COMMENT_LEN - 1 - fullComment.length(); // Boşluk ve null için 1 eksik
        if (maxLenForSensorData > 0) {
            // Sensör verisini formatla
            int charsWritten = snprintf(tempSensorBuf, sizeof(tempSensorBuf), " T:%.1fC P:%.0fhPa", sicaklik, basinc_hPa);
            if (charsWritten < maxLenForSensorData) { // Eğer sensör verisi yoruma sığıyorsa
                fullComment += String(tempSensorBuf);
            } else if (fullComment.isEmpty()) { // Yorum boşsa ve sensör verisi kısaysa, direkt sensör verisini kullan
                snprintf(tempSensorBuf, sizeof(tempSensorBuf), "T:%.1fC P:%.0fhPa", sicaklik, basinc_hPa);
                fullComment = String(tempSensorBuf).substring(0, min(charsWritten, APRS_COMMENT_LEN - 1)); // Sınırı aşma
            }
        }
    }

    String posLine = aprsConf.mycall + ">APLERT,TCPIP*,qAC:="
                   + aprsConf.latitude
                   + symTable
                   + aprsConf.longitude
                   + symCode + " "
                   + fullComment;   // Güncellenmiş yorum alanı
    client.println(posLine);
    Serial.println("[APRS] Konum: " + posLine);
  }

  // ── Status / mesaj ──
  // BMP280 sıcaklık ve basınç bilgilerini status mesajın arkasına ekle.
  // Eğer bmp280 okunamadıysa status mesajın arkasına "BMP280 OKUNAMADI" ekle.
  String fullStatusMessage = aprsConf.message; // Kullanıcının girdiği mevcut mesaj

  // Sensör verilerini veya hata mesajını statüs mesajına ekle
  if (bmp280_initialized) {
      if (!isnan(sicaklik) && !isnan(basinc_hPa)) {
          char sensorDataBuffer[30]; // Sensör verisi için yeterli buffer
          // Mevcut mesaj boşsa başa 'Sensor Data:' ekleyebiliriz
          if (fullStatusMessage.isEmpty()) {
              snprintf(sensorDataBuffer, sizeof(sensorDataBuffer), "T:%.1fC P:%.0fhPa", sicaklik, basinc_hPa);
          } else {
              // Mevcut mesaj varsa sonuna ekle
              snprintf(sensorDataBuffer, sizeof(sensorDataBuffer), " | T:%.1fC P:%.0fhPa", sicaklik, basinc_hPa);
          }
          fullStatusMessage += String(sensorDataBuffer);
      } else {
          // Sensör algılandı ama veri okunamadı
          if (fullStatusMessage.isEmpty()) {
              fullStatusMessage = "BMP280 OKUNAMADI (Veri Hatası)";
          } else {
              fullStatusMessage += " | BMP280 OKUNAMADI (Veri Hatası)";
          }
      }
  } else {
      // Sensör algılanmadı bile
      if (fullStatusMessage.isEmpty()) {
          fullStatusMessage = "BMP280 OKUNAMADI";
      } else {
          fullStatusMessage += " | BMP280 OKUNAMADI";
      }
  }

  // APRS mesaj alanı maksimum 67 karakterdir (APRS_MSG_LEN = 42 EEPROM limiti,
  // ancak gönderimde 67'ye kadar kullanılabilir). Bizim EEPROM sınırımız 42 olduğu için
  // String'i bu limite göre kırpıyoruz.
  if (fullStatusMessage.length() > (APRS_MSG_LEN - 1)) { // APRS_MSG_LEN, EEPROM'daki max uzunluk.
                                                         // APRS IS için teorik 67, ama bizim kayıtlı config'imiz 41+1.
      fullStatusMessage = fullStatusMessage.substring(0, APRS_MSG_LEN - 1);
  }


  if (!fullStatusMessage.isEmpty()) { // Sensör verisi ekli olsa bile boş kalabilir, kontrol edelim.
    String msgLine = aprsConf.mycall + ">APLERT,TCPIP*,qAC:>" + fullStatusMessage;
    client.println(msgLine);
    Serial.println("[APRS] Status: " + msgLine);
  }

  delay(2000); // Paketlerin gönderilmesi ve sunucu tarafından işlenmesi için bekle
  client.stop(); // Bağlantıyı kapat
  Serial.println("[APRS] Gönderim tamamlandı. Bağlantı kesildi.");
}


// ─────────────────────────────────────────────
//  APRS Gelen Kutusu Yönetimi
// ─────────────────────────────────────────────
// Gelen mesajları tampona ekler
void addIncomingMessage(const String& sender, const String& message) {
  ReceivedAPRSMessage newMessage;
  newMessage.sender = sender;
  newMessage.message = message;
  // Basit bir zaman damgası, gerçek zaman için RTC veya NTP gerekir
  // currentTime'ı epoch saniye olarak alıp formatlama daha iyi olabilir
  unsigned long secondsSinceBoot = millis() / 1000;
  String timeString;
  if (secondsSinceBoot < 60) timeString = String(secondsSinceBoot) + "s önce";
  else if (secondsSinceBoot < 3600) timeString = String(secondsSinceBoot / 60) + "dk önce";
  else if (secondsSinceBoot < 86400) timeString = String(secondsSinceBoot / 3600) + "sa önce";
  else timeString = String(secondsSinceBoot / 86400) + "gün önce";

  newMessage.timestamp = timeString;
  newMessage.read = false; // Yeni mesaj başlangıçta okunmamış

  incomingMessages[nextMessageIndex] = newMessage;
  nextMessageIndex = (nextMessageIndex + 1) % MAX_INCOMING_MESSAGES;
  if (incomingMessageCount < MAX_INCOMING_MESSAGES) {
      incomingMessageCount++;
  }
  Serial.println("[Mesaj Kutu] Yeni mesaj eklendi: " + sender + " -> " + message);
}

// Gelen APRS paketlerini okur ve mesajları ayrıştırır
void processIncomingAPRSData(String& data) {
  // Örnek paket: TA3OER-3>APIN21,TCPIP*,qAC,T2QUEBEC::TA3OER-4 :test test test{3
  Serial.println("[Parser] Paketi ayrıştırmaya başla: " + data);

  String senderCall;
  String targetCall;
  String actualMessage;

  // 1. Gönderen Callsign'ı bul (ilk '>' karakterine kadar)
  int greaterThanIdx = data.indexOf('>');
  if (greaterThanIdx == -1) {
    Serial.println("[Parser] Hata: '>' karakteri bulunamadı.");
    return;
  }
  senderCall = data.substring(0, greaterThanIdx);
  senderCall.trim();
  Serial.println("[Parser] Sender Call: " + senderCall);

  // 2. Mesajın başladığı ana ayracı bul (ilk Path sonrası ':')
  // Yani 'FROM>PATH:TO_CALL:MESSAGE' veya 'FROM>PATH::TO_CALL :MESSAGE' formunda

  // ÖNEMLİ DÜZELTME:
  // Kendi gönderdiğimiz paketin formatı: FROM>PATH,PATH*,PATH::TO :MESAJ
  // Bu durumda, path kısmı içinde de ':' olabileceği için, mesaj ayracını bulmak için
  // '::' ayracını veya en son ':' karakterini arayarak ilerlememiz daha mantıklı.
  // İlk '::' ayracının konumunu bulalım.
  int doubleColonIdx = data.indexOf("::", greaterThanIdx); // '>' sonrası '::' arar

  if (doubleColonIdx != -1) {
      // Özel formatımız mevcut: FROM>PATH::TO :MESAJ
      // Mesaj segmenti '::' karakterlerinden sonra başlayacak.
      String messageSegment = data.substring(doubleColonIdx + 2); // "::" karakterlerini atla
      messageSegment.trim();
      Serial.println("[Parser] Message Segment (Payload - Özel Format): " + messageSegment);

      int messageContentSeparator = messageSegment.indexOf(" :"); // TO_CALL ile mesajı ayırır (boşluklu iki nokta üst üste)
      if (messageContentSeparator != -1) {
          targetCall = messageSegment.substring(0, messageContentSeparator);
          actualMessage = messageSegment.substring(messageContentSeparator + 2); // " :" kısmını atla
          Serial.println("[Parser] Özel format (boşluklu) ayrıldı.");
      } else {
          // Eğer " :" bulunamazsa, belki sadece "::TO_CALL:MESSAGE" formatı vardır.
          messageContentSeparator = messageSegment.indexOf(":");
          if (messageContentSeparator != -1) {
              targetCall = messageSegment.substring(0, messageContentSeparator);
              actualMessage = messageSegment.substring(messageContentSeparator + 1);
              Serial.println("[Parser] Özel format (boşluksuz) ayrıldı.");
          } else {
              Serial.println("[Parser] Hata: Özel format ayracı bulunamadı (TO:MESSAGE veya TO :MESSAGE).");
              return; // Tanımsız format, atla
          }
      }
  } else {
      // Standart format: FROM>PATH:TO_CALL:MESSAGE
      // Bu durumda, '>' karakterinden sonraki SON ':' karakteri mesajı ayırır.
      int lastColonInPacket = data.lastIndexOf(':');
      if (lastColonInPacket == -1 || lastColonInPacket <= greaterThanIdx) { // '>' dan sonra hiç ':' yoksa veya geçersizse
          Serial.println("[Parser] Hata: Standart format için geçerli ':' ayracı bulunamadı.");
          return;
      }
      String messageSegment = data.substring(lastColonInPacket + 1); // Mesaj içeriği
      String targetAndPathSegment = data.substring(greaterThanIdx + 1, lastColonInPacket); // TO_CALL ve Path kısmı

      messageSegment.trim();
      targetAndPathSegment.trim();
      Serial.println("[Parser] Message Segment (Payload - Standart Format): " + messageSegment);
      Serial.println("[Parser] Target/Path Segment (Standart Format): " + targetAndPathSegment);

      // TO_CALL'ı bulmak için Path kısmını da temizlemeliyiz.
      // Basitçe: "APLERT,TCPIP*,qAC,T2FRANCE:TO_CALL" -> TO_CALL'ı almak
      int colonInTargetSegment = targetAndPathSegment.indexOf(':');
      if (colonInTargetSegment != -1) { // Path varsa
        targetCall = targetAndPathSegment.substring(colonInTargetSegment + 1);
      } else { // Sadece TO_CALL varsa
        targetCall = targetAndPathSegment;
      }
      actualMessage = messageSegment; // Mesaj içeriği zaten doğrudan payload'tan geliyor
      Serial.println("[Parser] Standart format ayrıldı.");
  }


  targetCall.trim();
  actualMessage.trim();
  Serial.println("[Parser] Final Target Call: " + targetCall);
  Serial.println("[Parser] Final Actual Message: " + actualMessage);

  // 4. Mesajın bize (aprsConf.mycall) gelip gelmediğini kontrol et
  if (targetCall.equalsIgnoreCase(aprsConf.mycall)) {
      Serial.println("[Parser] Mesaj BİZE ait! Mesajı ekle.");
      addIncomingMessage(senderCall, actualMessage);
  } else {
    Serial.println("[Parser] Mesaj BAŞKASINA ait (Hedef: " + targetCall + "). Atlandı.");
  }
}


// APRS-IS bağlantısını kontrol eder ve gelen veriyi okur
void checkAPRSInbox() {
  // APRS yapılandırılmamışsa veya WiFi yoksa bağlantı kurma
  if (aprsConf.mycall.isEmpty() || aprsConf.aprspass.isEmpty() || WiFi.status() != WL_CONNECTED) {
    if (aprsClient.connected()) {
      aprsClient.stop();
      Serial.println("[APRS Gelen] APRS ayarı veya WiFi yok, APRS-IS bağlantısı kesildi.");
    }
    return;
  }

  // APRS-IS bağlantısı koparsa yeniden bağlanmayı dene
  if (!aprsClient.connected()) {
    unsigned long now = millis();
    if (now - lastAPRSClientConnectAttempt >= APRS_CLIENT_RECONNECT_INTERVAL) {
      Serial.print("[APRS Gelen] APRS-IS bağlantısı kesildi veya hiç kurulmadı, yeniden bağlanılıyor...");
      aprsClient.stop(); // Önceki bağlantıyı kapat (varsa)

      String host = aprsConf.aprshost.isEmpty() ? String("france.aprs2.net") : aprsConf.aprshost;
      uint16_t port = (aprsConf.aprsport > 0) ? aprsConf.aprsport : 14580;

      if (aprsClient.connect(host.c_str(), port)) {
        Serial.println(" BAĞLANDI.");
        String loginLine = "user " + aprsConf.mycall + " pass " + aprsConf.aprspass + " vers ESP01_APRS_RX_V4";
        aprsClient.println(loginLine);
        // İsteğe bağlı: Filtre belirleyebiliriz (örn. m/callsign)
        // Eğer filtrelenirse sadece bize gelen mesajlar sunucu tarafından gönderilir.
        // client.println("#filter m/" + aprsConf.mycall);
        Serial.println("[APRS Gelen] Login: " + loginLine);
        aprsReadBuffer = ""; // Tamponu temizle
      } else {
        Serial.println(" BAĞLANTI HATASI!");
      }
      lastAPRSClientConnectAttempt = now;
    }
    return;
  }

  // Gelen verileri oku
  while (aprsClient.available()) {
    char c = aprsClient.read();
    aprsReadBuffer += c;

    if (c == '\n') { // Satır sonu karakteri (paket sonu)
      aprsReadBuffer.trim(); // Baştaki ve sondaki boşlukları temizle
      if (aprsReadBuffer.length() > 0 && !aprsReadBuffer.startsWith("#")) { // Yorum satırlarını atla
        Serial.println("[APRS Gelen] Ham Paket: " + aprsReadBuffer);
        processIncomingAPRSData(aprsReadBuffer);
      }
      aprsReadBuffer = ""; // Tamponu temizle
    }
    // Buffer taşması önlemi, çok uzun satırlar veya bozuk veriler için
    if (aprsReadBuffer.length() > 200) { // Ortalama APRS paketi 200 karakterden azdır
        Serial.println("[APRS Gelen] Tampon taşması, satır atlandı.");
        aprsReadBuffer = "";
    }
  }
}


// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  delay(2000); // Seri monitörün başlaması için bekle
  Serial.begin(115200);
  Serial.println("\n===== ESP01 APRS v4 – Web Arayüzlü (BMP280 Entegre) =====");

  // --- ESP01 Flash Bilgileri ---
  Serial.println("\n--- ESP01 Flash Bilgileri ---");
  Serial.print("Toplam Flash Boyutu: ");
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println(" KB");
  Serial.print("Mevcut Sketch (Kod) Boyutu: ");
  Serial.print(ESP.getSketchSize() / 1024);
  Serial.println(" KB");
  Serial.print("Kalan Flash (Teorik): ");
  Serial.print((ESP.getFlashChipSize() - ESP.getSketchSize()) / 1024); // Kalan teorik flash
  Serial.println(" KB (Bu, sadece gösterge amaçlıdır ve bölümlendirmeye göre değişir.)");
  Serial.println("-----------------------------");


  // BMP280 Sensörünü Başlatma
  // ESP-01 için I2C SDA=GPIO0, SCL=GPIO2
  Wire.begin(0, 2);
  Serial.print("[BMP280] Sensör başlatılıyor (I2C Adres 0x76)...");
  // Eğer sensörünüzün I2C adresi 0x77 ise: if (!bmp.begin(0x77)) olarak değiştirin.
  if (!bmp.begin(0x76)) {
    Serial.println(" BAĞLANTI HATASI! Sensörü kontrol edin veya adresini doğrulayın.");
    bmp280_initialized = false;
  } else {
    Serial.println(" BAŞARILI.");
    bmp280_initialized = true;
    // Sensör ayarlarını yapabilirsiniz (isteğe bağlı, güç tüketimi/doğruluk dengesi için)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Normal çalışma modu
                     Adafruit_BMP280::SAMPLING_X2,     // Sıcaklık oversampling (x2)
                     Adafruit_BMP280::SAMPLING_X16,    // Basınç oversampling (x16)
                     Adafruit_BMP280::FILTER_X16,      // IIR filtreleme (x16)
                     Adafruit_BMP280::STANDBY_MS_500); // 500ms bekleme süresi
  }


  // Config yükle
  bool hasConfig = loadConfig();
  Serial.println("[Config] Magic: " + String(hasConfig ? "OK" : "YOK (varsayılan ayarlar)"));
  Serial.println("[Config] Kayıtlı WiFi sayısı: " + String(savedWifiCount));

  // WiFi bağlanmaya çalış
  bool connected = false;
  if (hasConfig && savedWifiCount > 0) {
    connected = tryConnectWifi();
  }

  // Bağlantı yok → AP modu
  if (!connected) {
    startAP();
  }

  // Web server rota kayıtları
  server.on("/",            HTTP_GET,  handleIndex);
  server.on("/wifi",        HTTP_GET,  handleWifi);
  server.on("/aprs",        HTTP_GET,  handleAprs);
  server.on("/message",     HTTP_GET,  handleMessage);
  server.on("/inbox",       HTTP_GET,  handleInbox);

  server.on("/wifi",        HTTP_POST, handlePostWifi);
  server.on("/wifi/delete", HTTP_POST, handlePostWifiDelete);
  server.on("/aprs",        HTTP_POST, handlePostAprs);
  server.on("/message",     HTTP_POST, handlePostMessage);
  server.on("/inbox/read",  HTTP_POST, handlePostInboxRead);
  server.on("/inbox/clear", HTTP_POST, handlePostInboxClear);
  server.on("/restart",     HTTP_POST, handlePostRestart);


  server.begin();
  Serial.println("[Web] Server hazır (port 80)");
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
static unsigned long lastAPRS = 0;
//static const long    APRS_INTERVAL = 180000L; // 3 dakika (3 * 60 * 1000 ms)
static const long    APRS_INTERVAL = 900000L; // 15 dakika (15 * 60 * 1000 ms)
//static const long    APRS_INTERVAL = 1800000L; // 30 dakika (30 * 60 * 1000 ms)

void loop() {
  server.handleClient(); // Web isteklerini işle

  // APRS-IS gelen kutusunu kontrol et
  checkAPRSInbox();

  // STA modda bağlı isek APRS gönder (her APRS_INTERVAL dak)
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long now = millis();
    // İlk çalıştırmada veya belirlenen aralıktan sonra gönder
    if (now - lastAPRS >= APRS_INTERVAL || lastAPRS == 0) {
      sendAPRS();
      lastAPRS = now; // Son gönderme zamanını güncelle
    }
  }
}
