"""TepeGöz - Drone Yönetim Sistemi Konfigürasyonu

Drone fiziksel ayarları, görev kontrol parametreleri, 
ROS kamera entegrasyonu ve durum mesajları.
"""

# ==============================================================================
# DRON AYARLARI
# ==============================================================================
TAKEOFF_ALTITUDE = 5               # Kalkış yüksekliği (metre)
CRITICAL_BATTERY_LEVEL = 100         # Batarya seviyesi bu değerin altına düştüğünde dron geri döner (%)
MAX_TOUR_COUNT = 1                  # Bir görevin kaç kez tekrarlanacağı
SQUARE_SIZE = 20                    # Dronun gözlem yapacağı alanın boyutu (metre)
DRONE_SPEED = 90                    # Dronun hızı (cm/s)
CONNECTION_TIMEOUT = 20             # Dron bağlantısı için maksimum bekleme süresi (saniye)
DRONE_CONNECTION_RETRY_COUNT = 3    # Bağlantı deneme sayısı

# ==============================================================================
# GÖREV KONTROL PARAMETRELERİ
# ==============================================================================
MAX_SPEED_M_S = 4.0                 # Maksimum hız (m/s)
CONTROL_INTERVAL_S = 0.15           # PID kontrol döngüsü aralığı (saniye)
CELL_REACHED_THRESHOLD_M = 1.2      # Hedef merkeze kabul eşiği (metre)
FINE_APPROACH_THRESHOLD_M = 7.0     # Yavaşlama için mesafe eşiği (metre)
FINE_APPROACH_SCALE = 0.35          # Yakınlaşma hızı ölçeği
CENTER_CONFIRM_TOLERANCE_M = 1.5    # Merkez doğrulama için sapma toleransı (metre)
CENTER_CONFIRM_HOLD_S = 0.2         # Merkez doğrulama için bekleme süresi (saniye)
FINE_APPROACH_HOLD_S = 0.5          # Merkezde onay için kısa bekleme süresi (saniye)
PER_CELL_TIMEOUT_S = 60.0           # Bir hücreye ulaşma için maksimum süre (saniye)
RETRY_LIMIT = 2                     # Hücre atlamadan önce deneme sayısı
GPS_MEDIAN_WINDOW = 7               # GPS medyan filtreleme pencere boyutu
STUCK_MOVED_THRESHOLD_M = 0.12      # Takılma tespiti için hareket eşiği (metre)
STUCK_TIMEOUT_S = 8.0               # Takılma tespiti için zaman aşımı (saniye)
PID_KP = 0.9
PID_KI = 0.02
PID_KD = 0.12

# ==============================================================================
# ÇOKLU DRON VE SWİTCHER AYARLARI
# ==============================================================================
RTL_LANDING_TIMEOUT_S = 30.0        # Kritik bataryadan sonra RTL iniş bekleme süresi (saniye)
MONITOR_INTERVAL_S = 1.0            # Batarya monitörü kontrol döngüsü aralığı (saniye)

# ==============================================================================
# KAMERA VE ROS AYARLARI
# ==============================================================================
# Her port için ROS topic'ini belirten sözlük.
CAMERA_TOPICS = {
    5760: "/webcam/image_raw",
    5761: "/webcam/image_raw",
    5762: "/webcam/image_raw",
    5763: "/webcam/image_raw",
    5770: "/webcam_2/image_raw_2",
    5771: "/webcam_2/image_raw_2",
    5772: "/webcam_2/image_raw_2",
    5773: "/webcam_2/image_raw_2"
}

LOG_THROTTLE_SEC = 0.5              # Logların ne sıklıkta yazılacağını belirler (saniye)
JPEG_QUALITY = 80                   # JPEG görüntü sıkıştırma kalitesi (0-100 arası)

# ==============================================================================
# GÖRÜNTÜ İŞLEME VE YER TUTUCU AYARLARI
# ==============================================================================
PLACEHOLDER_IMAGE_SIZE = (640, 480) # Kamera akışı olmadığında gösterilen yer tutucu resmin boyutu
FONT_SCALE = 0.6                    # Yer tutucu resimdeki yazı fontunun boyutu
FONT_THICKNESS = 1                  # Yer tutucu resimdeki yazı kalınlığı

# ==============================================================================
# YAPAY ZEKA (YOLO) AYARLARI
# ==============================================================================
YOLO_MODEL_PATH = "./TepeGoz-main/models/yolov8n.pt"
YOLO_CONF_THRESHOLD = 0.25  # Güven eşiği - Düşürüldü (0.35→0.25) uzak hedefler için
YOLO_IOU_THRESHOLD = 0.40   # IoU eşiği - Düşürüldü (0.45→0.40) daha esnek overlap

# YOLO Tracking Ayarları
# LAP (Linear Assignment Problem) kütüphanesi gerektirir
# pip3 install lap --no-cache-dir
YOLO_TRACKING_ENABLED = True          # Tracking özelliği aktif/pasif
YOLO_TRACKER_TYPE = "bytetrack.yaml"  # Tracker tipi: bytetrack.yaml veya botsort.yaml
YOLO_TRACK_PERSIST = True             # Track ID'leri frame'ler arası koru
YOLO_TRACK_BUFFER = 60                # Kayıp nesne için buffer frame sayısı - Artırıldı (30→60)

# ==============================================================================
# ÇARPMA GÖREVİ AYARLARI
# ==============================================================================
COLLISION_SCREEN_THRESHOLD = 0.40   # Ekranın %40'ı kaplıysa çarpma gerçekleşmiş sayılır
COLLISION_FORWARD_SPEED = 2.0       # Takip sırasında sabit ileri hız (m/s)
HUMAN_LOST_TIMEOUT = 5.0            # İnsan kaybolduktan sonra son hareketi sürdürme süresi (saniye) - Artırıldı
APPROACH_MIN_SCREEN_COVERAGE = 0.001  # %0.1 - Bu değerin altındaysa agresif yaklaşma modu
APPROACH_BOOST_SPEED = 4.0          # Hedef çok küçükse daha hızlı yaklaş (m/s)
RESUME_SCAN_AFTER_LOST = True       # Hedef kaybolunca taramaya geri dön (True) veya RTL yap (False)

# 360° Tarama Ayarları
ROTATION_YAW_INCREMENT = 45         # Her adımda dönüş açısı (derece) - 360° tarama için
ROTATION_YAW_SPEED = 20             # Dönüş hızı (derece/saniye) - Drone yaw rate hızı
ROTATION_DIRECTION = 1              # Dönüş yönü (1=saat yönü, -1=saat yönü tersi)

# ==============================================================================
# PID KONTROL PARAMETRELERİ (TRACKING)
# ==============================================================================
# YAW (Yatay Dönüş) PID Parametreleri
TRACKING_YAW_PID_TAU = 0.1          # Filtreleme zaman sabiti - Düşük = Daha hızlı tepki
TRACKING_YAW_PID_KP = 1.5           # Proportional gain - Yüksek = Daha agresif dönüş
TRACKING_YAW_PID_KI = 0.01          # Integral gain - Steady-state hata düzeltme
TRACKING_YAW_PID_KD = 0.08          # Derivative gain - Titreşim azaltma
TRACKING_YAW_PID_INT_MAX = 0.01     # Integrator maksimum sınır - Windup önleme
TRACKING_YAW_PID_INT_MIN = -0.01    # Integrator minimum sınır
TRACKING_YAW_PID_OUT_MAX = 3.14159  # Çıkış maksimum (π rad/s)
TRACKING_YAW_PID_OUT_MIN = -3.14159 # Çıkış minimum (-π rad/s)

# PITCH (Dikey Hareket) PID Parametreleri
TRACKING_PITCH_PID_TAU = 0.1        # Filtreleme zaman sabiti
TRACKING_PITCH_PID_KP = 2.5         # Proportional gain - Dikey hareket hassasiyeti
TRACKING_PITCH_PID_KI = 0.03        # Integral gain
TRACKING_PITCH_PID_KD = 0.08        # Derivative gain
TRACKING_PITCH_PID_INT_MAX = 0.05   # Integrator maksimum sınır
TRACKING_PITCH_PID_INT_MIN = -0.05  # Integrator minimum sınır
TRACKING_PITCH_PID_OUT_MAX = 10.0   # Çıkış maksimum (m/s)
TRACKING_PITCH_PID_OUT_MIN = -10.0  # Çıkış minimum (m/s)

# ==============================================================================
# KAMERA OPTİK PARAMETRELERİ
# ==============================================================================
CAMERA_FOCAL_LENGTH = 3.37          # Efektif odak uzaklığı (mm) - Kamera lens özellikleri
CAMERA_SENSOR_WIDTH = 4208          # Kamera sensör genişliği (piksel) - Fiziksel sensör boyutu
CAMERA_RESOLUTION_WIDTH = 1920      # Görüntü çözünürlüğü genişliği (piksel) - İşlenen görüntü boyutu
CAMERA_PIXEL_SIZE_UM = 1.12         # Piksel boyutu (mikrometre) - Sensör piksel pitch

# ==============================================================================
# GÖRSEL TAKİP PARAMETRELERİ
# ==============================================================================
TRACKING_TARGET_OFFSET_Y_RATIO = 0.75      # Hedef nokta Y ofseti (0.75 = göğüs seviyesi, (y2 + 3*y1)/4)
TRACKING_SCREEN_OFFSET_Y_RATIO = 0.0625    # Ekran merkezi Y ofseti (1/16 = img_height/16)
TRACKING_LATERAL_CORRECTION_GAIN = 1.5     # Yatay düzeltme kazancı - theta_x'i vy'ye çevirme katsayısı
TRACKING_VERTICAL_FALLBACK_GAIN = 0.008    # Derinlik kamerası yoksa dikey hareket kazancı (m/s per piksel)

# ==============================================================================
# FRAME BUFFERING PARAMETRELERİ
# ==============================================================================
FRAME_BUFFER_TIMEOUT_FRAMES = 20    # Frame buffer timeout (frame sayısı) - 20 frame = ~1 saniye @20fps

# ==============================================================================
# UYGULAMA SUNUCU VE DURUM MESAJLARI
# ==============================================================================
APP_HOST = '0.0.0.0'
APP_PORT = 5000
CONNECTION_STATUS_CONNECTED = "connected"
CONNECTION_STATUS_NOT_CONNECTED = "not_connected"

MISSION_STATUS_MESSAGES = {
    "NOT_CONNECTED": "Bağlanmayı bekliyor...",
    "CONNECTING": "Drona bağlanılıyor...",
    "CONNECTION_ERROR": "Bağlantı hatası. Tekrar deneyin.",
    "RETRYING": "Bağlantı başarısız. Yeniden deneniyor... (Deneme {current}/{total})",
    "ALL_RETRIES_FAILED": "Tüm bağlantı denemeleri başarısız oldu.",
    "CONNECTED": "Dron bağlandı. Görev için hazır.",
    "AREA_SET": "Gözlem alanı başarıyla ayarlandı. ({rows}x{cols} grid)",
    "STARTING": "Görev başlatılıyor...",
    "INVALID_TYPE": "Geçersiz görev tipi: {mission_type}",
    "ALREADY_ACTIVE": "Görev zaten aktif.",
    "NO_AREA": "Lütfen önce bir gözlem alanı belirleyin.",
    "STOPPED": "Görev durduruldu. Dron ana konuma dönüyor.",
    "NO_ACTIVE": "Aktif bir görev yok.",
    "TAKING_OFF": "Kalkış yapılıyor. Hedef yükseklik: {altitude} metre.",
    "ARMING": "Motorlar hazırlanıyor...",
    "TAKEOFF_SUCCESS": "Hedef yüksekliğe ulaşıldı: {altitude} metre.",
    "BEGIN_MISSION_FLIGHT": "Hedef yüksekliğe ulaşıldı, gözlem alanına doğru ilerleniyor.",
    "NAVIGATING_TO_CELL": "Gözlem noktasına gidiyor: Hücre [{row},{col}].",
    "APPROACHING": "Hücre ({from_row},{from_col}) -> Hücre ({to_row},{to_col}) noktasına yaklaşıyor. Mesafe: {distance:.1f} m, Hız: {speed:.1f} m/s.",
    "VISITING": "Hücre [{row},{col}] gözlemleniyor. Merkezde stabil kalınıyor.",
    "CELL_CONFIRMED": "Hücre gözlemi tamamlandı: [{row},{col}].",
    "CELL_SKIPPED": "Hücre gözlemi atlandı: [{row},{col}].",
    "STUCK": "Dron takıldı. Kaçış manevrası yapılıyor.",
    "TIMEOUT": "Hücreye ulaşma zaman aşımı. Sonraki hücreye geçiliyor.",
    "CRITICAL_BATTERY": "Kritik batarya seviyesi! RTL (ana konuma dönüş) başlatıldı.",
    "MISSION_COMPLETE": "Görev tamamlandı. Dron ana konuma dönüyor.",
    "MISSION_ERROR": "Görev sırasında beklenmedik bir hata oluştu: {error}",
    "SWITCHING_DRONE": "Batarya bitmek üzere. Görev diğer drona devrediliyor...",
    "DRONE_IS_BACK": "Dron ana konuma döndü ve şarj oluyor.",
    "RESUMING": "Görev kaldığı yerden devam ettiriliyor.",
    "HANDOVER_MISSION_TYPE": "Görevin tipi devrediliyor: {mission_type}",
    "SWITCH_TO_NEXT": "Bir sonraki drona geçiş yapılıyor...",
    "NEXT_DRONE_SELECTED": "Yeni dron ({port}) seçildi. Görev devam ettiriliyor.",
    "MONITOR_STARTED": "[DroneSwitcher] İzleme başlatıldı.",
    "MONITOR_STOPPED": "[DroneSwitcher] İzleme durduruldu.",
     "RTL_STARTED": "[DroneSwitcher] Aktif dron RTL moduna geçirildi.",
    "MONITOR_ERROR": "[DroneSwitcher] İzleme sırasında hata oluştu: {error}",
    "RESUMING_FROM_POINT": "Kaldığı noktadan devam ediliyor: {point}",
    "NO_LAST_POINT": "Son görev noktası bulunamadı, baştan başlanıyor.",
    "HUMAN_DETECTED": "İnsan tespit edildi! Takip ve çarpma modu başlatılıyor.",
    "TRACKING_HUMAN": "İnsan takip ediliyor. Ekran kaplaması: {coverage:.1f}%",
    "COLLISION_DETECTED": "Çarpma gerçekleşti! Ekran kaplaması: {coverage:.1f}%",
    "HUMAN_LOST": "Hedef kaybedildi. Son hareket sürdürülüyor...",
    "ROTATING_360": "Tek hücre taraması: 360° dönüş yapılıyor.",
    "SCANNING_AREA": "Alan taraması yapılıyor. Hücre: [{row},{col}]"
}