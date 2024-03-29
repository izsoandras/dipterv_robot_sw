//
// ESP32 Chip model = ESP32-D0WDQ6 Rev 1
// This chip has 2 cores
// Chip ID: 5083536

#include <Arduino.h>
#include "esp_log.h"
#include "wheel_control.h"
#include "position_control.h"
#include "mykalman.h"
#include "esp_timer.h"
#include "MPU9250.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Profiling start");

    // Speed profiling - KALMAN FILTER
    float A[] = {0.814723686393179, 0.278498218867048, 0.957166948242946, 0.792207329559554, 0.678735154857774, 0.706046088019609,
                 0.905791937075619, 0.546881519204984, 0.485375648722841, 0.959492426392903, 0.757740130578333, 0.0318328463774207,
                 0.126986816293506, 0.957506835434298, 0.800280468888800, 0.655740699156587, 0.743132468124916, 0.276922984960890,
                 0.913375856139019, 0.964888535199277, 0.141886338627215, 0.0357116785741896, 0.392227019534168, 0.0461713906311539,
                 0.632359246225410, 0.157613081677548, 0.421761282626275, 0.849129305868777, 0.655477890177557, 0.0971317812358475,
                 0.0975404049994095, 0.970592781760616, 0.915735525189067, 0.933993247757551, 0.171186687811562, 0.823457828327293};
    float B[] = {0.694828622975817, 0.765516788149002,
                 0.317099480060861, 0.795199901137063,
                 0.950222048838355, 0.186872604554379,
                 0.0344460805029088, 0.489764395788231,
                 0.438744359656398, 0.445586200710900,
                 0.381558457093008, 0.646313010111265};
    float C[] = {0.709364830858073, 0.276025076998578, 0.655098003973841, 0.118997681558377, 0.959743958516081, 0.585267750979777,
                 0.754686681982361, 0.679702676853675, 0.162611735194631, 0.498364051982143, 0.340385726666133, 0.223811939491137};
    float Rv[] = {0.751267059305653, 0.547215529963803, 0.814284826068816, 0.616044676146639, 0.917193663829810, 0.0758542895630636,
                  0.255095115459269, 0.138624442828679, 0.243524968724989, 0.473288848902729, 0.285839018820374, 0.0539501186666072,
                  0.505957051665142, 0.149294005559057, 0.929263623187228, 0.351659507062997, 0.757200229110721, 0.530797553008973,
                  0.699076722656686, 0.257508254123736, 0.349983765984809, 0.830828627896291, 0.753729094278495, 0.779167230102011,
                  0.890903252535799, 0.840717255983663, 0.196595250431208, 0.585264091152724, 0.380445846975357, 0.934010684229183,
                  0.959291425205444, 0.254282178971531, 0.251083857976031, 0.549723608291140, 0.567821640725221, 0.129906208473730};
    float Rz[] = {0.568823660872193, 0.0119020695012414,
                  0.469390641058206, 0.337122644398882};
    float x0[] = {0,0,0};
    KF testKF(A, B, C, Rv, Rz, x0, 6, 2, 2);

    ESP_LOGI("prof","KF init finished");

    uint64_t start = esp_timer_get_time();

    float u[] = {0.1, 0.3};
    float y[] = {0.5, 0.2};
    uint16_t measNo = 1000;
    for (int i = 0; i < measNo; i++)
    {
        u[0] += 0.01;
        u[1] -= 0.1;
        y[0] += 0.012;
        y[1] -= 0.32;
        testKF.update(u, y);
    }

    uint64_t end = esp_timer_get_time();
    ESP_LOGI("prof","KF done");

    ESP_LOGI("kf meas", "Total time: %fms, Single run: %fms", (end - start) / 1000.0, (end - start) / (1000.0 * measNo));


    MPU9250 imu;
    Wire.begin();
    while(!imu.setup(0x68)){
        ESP_LOGI("imu","MPU setup didn't work");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI("prof","IMU init finished");
    start = esp_timer_get_time();

    float data[3];
    for(int i = 0; i < measNo; i++){
        imu.update();
    }

    end = esp_timer_get_time();
    ESP_LOGI("prof","IMU done");

    ESP_LOGI("imu meas", "Total time: %fms, Single run: %fms", (end - start) / 1000.0, (end - start) / (1000.0 * measNo));

    ESP_LOGI("prof","AccGyro start");
    start = esp_timer_get_time();
    for(int i = 0; i < measNo; i++){
        imu.update_accel_gyro();
    }

    end = esp_timer_get_time();
    ESP_LOGI("prof","AccGyro done");

    ESP_LOGI("imu meas", "Total time: %fms, Single run: %fms", (end - start) / 1000.0, (end - start) / (1000.0 * measNo));

    
    ESP_LOGI("prof","Magn start");
    start = esp_timer_get_time();
    for(int i = 0; i < measNo; i++){
        imu.update_mag();
    }

    end = esp_timer_get_time();
    ESP_LOGI("prof","Magn done");

    ESP_LOGI("imu meas", "Total time: %fms, Single run: %fms", (end - start) / 1000.0, (end - start) / (1000.0 * measNo));

}

void loop()
{
    // put your main code here, to run repeatedly:
    //ESP_LOGW("ENC","Pulse cound: %d", ident_enc.getCountReset());
    vTaskDelay(pdMS_TO_TICKS(100));
}