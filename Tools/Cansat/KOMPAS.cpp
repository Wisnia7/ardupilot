
#include <AP_HAL/AP_HAL.h> //biblioteki
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); //ta śmieszna zmienna

static AP_BoardConfig board_config; //zmienna chyba do jakiś konfiguracji

class DummyVehicle { //nie wiem co to
public:
    AP_AHRS ahrs;  // Need since https://github.com/ArduPilot/ardupilot/pull/10890
    AP_Baro baro; //Compass próbuje ustawić model magnetyczny na podstawie lokalizacji.
#if HAL_EXTERNAL_AHRS_ENABLED
    AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED
};

static DummyVehicle vehicle;
// tworzenie obiektu kompasu
static Compass compass;
static AP_SerialManager serial_manager;

uint32_t timer; //timer

// inicjalizacja
static void setup()
{
    hal.console->printf("Compass library test\n");

    board_config.init();
    vehicle.ahrs.init();
    compass.init();
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    // ustawia jakieś przesunięcia, żeby pozbyć się zakłóceń
    compass.set_and_save_offsets(0, Vector3f(0, 0, 0));
    // ustawia jakąś różnicę między północą magnetyczną a prawdziwą północą
    compass.set_declination(ToRad(0.0f));

    hal.scheduler->delay(1000); //jakieś opóźnienie
    timer = AP_HAL::micros(); //ustawienie czegoś na timerze
}

// jakaś dziwna pętla
static void loop()
{
    static const uint8_t compass_count = compass.get_count(); //bardzo śmieszna zmienn, która ustawia wyjście kompasu
    static float min[COMPASS_MAX_INSTANCES][3]; //nie wiem co to, ale wyglada interesująco
    static float max[COMPASS_MAX_INSTANCES][3];
    static float offset[COMPASS_MAX_INSTANCES][3];

    // run read() at 10Hz
    if ((AP_HAL::micros() - timer) > 100000L) { //coś z czasem?
        timer = AP_HAL::micros(); //znowu ustawienie timera
        compass.read(); //kompas czyta
        const uint32_t read_time = AP_HAL::micros() - timer; //jakaś różnica czasów

        for (uint8_t i = 0; i < compass_count; i++) { //pętla
            float heading; //jakaś zmienna

            hal.console->printf("Compass #%u: ", i); //coś pissze na konsoli

            if (!compass.healthy()) {
                hal.console->printf("not healthy\n"); //to chyba informuje czy układ jest obrócony
                continue;
            }

            Matrix3f dcm_matrix;
            // use roll = 0, pitch = 0 for this example
            dcm_matrix.from_euler(0, 0, 0); //co?
            heading = compass.calculate_heading(dcm_matrix, i);

            const Vector3f &mag = compass.get_field(i); //jakaś zmienna z kompasem

            // capture min
            min[i][0] = MIN(mag.x, min[i][0]);
            min[i][1] = MIN(mag.y, min[i][1]);
            min[i][2] = MIN(mag.z, min[i][2]);

            // capture max
            max[i][0] = MAX(mag.x, max[i][0]);
            max[i][1] = MAX(mag.y, max[i][1]);
            max[i][2] = MAX(mag.z, max[i][2]);

            // liczy przesunięcie
            offset[i][0] = -(max[i][0] + min[i][0]) / 2; 
            offset[i][1] = -(max[i][1] + min[i][1]) / 2;
            offset[i][2] = -(max[i][2] + min[i][2]) / 2;

            // wyświetla wszystko
            hal.console->printf("Heading: %.2f (%3d, %3d, %3d)",
                                (double)ToDeg(heading),
                                (int)mag.x,
                                (int)mag.y,
                                (int)mag.z); //to są chyba odchylenia we wszystkich osiach

            // jakieś przesunięcie
            hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                                (double)offset[i][0],
                                (double)offset[i][1],
                                (double)offset[i][2]);

            hal.console->printf(" t=%u", (unsigned)read_time); //coś tam pisze

            hal.console->printf("\n");
        }
    } else {

        // opóźnienie programu
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();