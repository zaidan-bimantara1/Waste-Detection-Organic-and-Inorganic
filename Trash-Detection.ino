
#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_camera.h>
#include <zaidan-bimantara1-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// ==================================================
// KONFIGURASI PIN
// ==================================================
#define TRIG_PIN 13
#define ECHO_PIN 14
#define SERVO_PIN 2

// ==================================================
// KONFIGURASI SENSOR DAN SERVO
// ==================================================
#define JARAK_DETEKSI 15       // Jarak deteksi sampah (cm)
#define SUDUT_KIRI 0
#define SUDUT_KANAN 90
#define SUDUT_TENGAH 45

#define CONFIDENCE_MIN 0.60f   // Kepercayaan minimal 60%
#define WAKTU_BUKA 2000        // Lama servo membuka (ms)

// ==================================================
// PIN KAMERA AI THINKER ESP32-CAM
// ==================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Ukuran gambar kamera sebelum resize
#define CAM_WIDTH  320
#define CAM_HEIGHT 240

// ==================================================
// VARIABEL GLOBAL
// ==================================================
Servo servo;
uint8_t *snapshot_buf = nullptr;
bool kameraSiap = false;

// ==================================================
// INISIALISASI KAMERA
// ==================================================
bool inisialisasiKamera() {
    camera_config_t config = {};

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d0 = Y2_GPIO_NUM;

    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Gagal menginisialisasi kamera!");
        return false;
    }

    Serial.println("Kamera berhasil diinisialisasi.");
    return true;
}

// ==================================================
// MEMBACA JARAK HC-SR04
// ==================================================
float bacaJarak() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(3);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Timeout 30 ms agar program tidak menunggu terlalu lama
    unsigned long durasi = pulseIn(ECHO_PIN, HIGH, 30000);

    if (durasi == 0) {
        return -1.0f;  // Tidak ada pantulan yang terbaca
    }

    return durasi * 0.0343f / 2.0f;
}

// ==================================================
// MENGUBAH DATA GAMBAR UNTUK EDGE IMPULSE
// ==================================================
static int ambilDataGambar(
    size_t offset,
    size_t length,
    float *out_ptr
) {
    size_t pixel_ix = offset * 3;

    for (size_t i = 0; i < length; i++) {
        // RGB888 diubah ke format packed RGB yang diperlukan EI
        out_ptr[i] =
            (snapshot_buf[pixel_ix + 2] << 16) |
            (snapshot_buf[pixel_ix + 1] << 8) |
             snapshot_buf[pixel_ix];

        pixel_ix += 3;
    }

    return 0;
}

// ==================================================
// PENGAMBILAN DAN RESIZE GAMBAR
// ==================================================
bool ambilGambar() {
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        Serial.println("Gagal mengambil gambar!");
        return false;
    }

    bool berhasil = fmt2rgb888(
        fb->buf,
        fb->len,
        PIXFORMAT_JPEG,
        snapshot_buf
    );

    esp_camera_fb_return(fb);

    if (!berhasil) {
        Serial.println("Gagal mengubah gambar ke RGB!");
        return false;
    }

    // Resize/crop gambar menjadi 96 x 96 piksel.
    // Fungsi ini mengikuti alur contoh kamera Edge Impulse.
    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        CAM_WIDTH,
        CAM_HEIGHT,
        snapshot_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT
    );

    return true;
}

// ==================================================
// KLASIFIKASI SAMPAH
// ==================================================
String deteksiSampah() {
    if (!ambilGambar()) {
        return "unknown";
    }

    ei::signal_t signal;
    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT;

    signal.get_data = &ambilDataGambar;

    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR err = run_classifier(
        &signal,
        &result,
        false
    );

    if (err != EI_IMPULSE_OK) {
        Serial.printf("Klasifikasi gagal: %d\n", err);
        return "unknown";
    }

    float skorOrganik = 0;
    float skorOnorganik = 0;
    float skorUnknown = 0;

    Serial.println("\n--- HASIL KLASIFIKASI ---");

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        const char *label = result.classification[i].label;
        float skor = result.classification[i].value;

        Serial.printf("%s: %.2f%%\n", label, skor * 100.0f);

        if (strcmp(label, "Organik") == 0) {
            skorOrganik = skor;
        }
        else if (strcmp(label, "Onorganik") == 0) {
            skorOnorganik = skor;
        }
        else if (strcmp(label, "unknown") == 0) {
            skorUnknown = skor;
        }
    }

    // Unknown atau keyakinan terlalu rendah:
    // diarahkan ke posisi kiri sebagai posisi default.
    if (skorUnknown >= skorOrganik &&
        skorUnknown >= skorOnorganik) {
        return "unknown";
    }

    if (skorOrganik >= CONFIDENCE_MIN &&
        skorOrganik > skorOnorganik) {
        return "Organik";
    }

    if (skorOnorganik >= CONFIDENCE_MIN &&
        skorOnorganik > skorOrganik) {
        return "Onorganik";
    }

    return "unknown";
}

// ==================================================
// KONTROL SERVO
// ==================================================
void gerakkanServo(String jenis) {
    if (jenis == "Organik") {
        Serial.println("Sampah ORGANIK -> SERVO KANAN");
        servo.write(SUDUT_KANAN);
    }
    else {
        Serial.println(
            "Sampah ONORGANIK/UNKNOWN -> SERVO KIRI"
        );
        servo.write(SUDUT_KIRI);
    }

    delay(WAKTU_BUKA);

    // Kembali ke posisi tengah setelah sampah lewat.
    // Ubah ke SUDUT_KIRI jika posisi istirahat yang
    // kamu inginkan adalah kiri.
    servo.write(SUDUT_TENGAH);
    delay(500);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Servo SG90
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
    servo.write(SUDUT_TENGAH);

    Serial.println("\nTempat Sampah Otomatis");
    Serial.println("Menginisialisasi kamera...");

    if (!psramFound()) {
        Serial.println(
            "PERINGATAN: PSRAM tidak terdeteksi!"
        );
    }

    if (!inisialisasiKamera()) {
        Serial.println(
            "Periksa kabel dan konfigurasi kamera."
        );
        while (true) {
            delay(1000);
        }
    }

    snapshot_buf = (uint8_t *)ps_malloc(
        CAM_WIDTH * CAM_HEIGHT * 3
    );

    if (snapshot_buf == nullptr) {
        Serial.println("Gagal mengalokasikan memori gambar!");
        while (true) {
            delay(1000);
        }
    }

    kameraSiap = true;
    Serial.println("Sistem siap mendeteksi sampah.");
}

// ==================================================
// LOOP UTAMA
// ==================================================
void loop() {
    if (!kameraSiap) {
        return;
    }

    float jarak = bacaJarak();

    if (jarak > 0 && jarak <= JARAK_DETEKSI) {
        Serial.printf(
            "\nSampah terdeteksi pada %.1f cm\n",
            jarak
        );

        // Beri waktu agar objek tidak sedang bergerak cepat
        delay(200);

        String jenis = deteksiSampah();

        Serial.print("Jenis sampah: ");
        Serial.println(jenis);

        gerakkanServo(jenis);

        // Tunggu objek menjauh agar tidak berulang kali
        // diklasifikasikan saat masih berada di depan sensor.
        unsigned long mulaiTunggu = millis();

        while (millis() - mulaiTunggu < 5000) {
            float jarakBaru = bacaJarak();

            if (jarakBaru < 0 ||
                jarakBaru > JARAK_DETEKSI) {
                break;
            }

            delay(100);
        }
    }

    delay(100);
}
