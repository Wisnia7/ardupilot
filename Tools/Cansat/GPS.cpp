
#include <AP_HAL/AP_HAL.h> //biblioteka jakaś
#include <AP_GPS/AP_GPS.h> //biblioteka GPS
#include <GCS_MAVLink/GCS_Dummy.h> //nie wiem co to, ale ma być
#include <AP_Notify/AP_Notify.h> //jakać biblioteka co zmienia UAVACAN na DroneCan
#include <AP_Notify/AP_BoardLED.h> //to co wyżej
#include <AP_SerialManager/AP_SerialManager.h> //nie wiem co to
#include <AP_BoardConfig/AP_BoardConfig.h> //ani to

void setup(); //główna funkcja
void loop(); //pętla

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); //jakaś dziwna stała zmienna

static AP_BoardConfig board_config; //jakaś zmienna widoczna tylko dla tej jednej funkcji

// tworzenie jakiegoś obiektu z tablicą
AP_BoardLED board_led;

// tworzy jakiś fałszywy obiekt gcs
GCS_Dummy _gcs;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = { //znowó dziwna stała zmienna
        AP_GROUPEND
};

// wkońcu coś co wykorzystuje GPS
static AP_GPS gps;
// Jakiś menadżer szeregowy, jest podobno potrzebny do komunikacji z UART
static AP_SerialManager serial_manager;

//służy inicjalizacji obiektów
void setup()
{
    hal.console->printf("GPS AUTO library test\n"); // ma coś napisać i chyba przejść linike niżej

    board_config.init(); // nie wiem jeszcze co to

    // Initialise the leds
    board_led.init(); //podobno coś z diodami

    // Inizjalizuje UART dla systemu GPS
    serial_manager.init();
    gps.init(serial_manager);
}


/*
  print a int32_t lat/long in decimal degrees
 */
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon); // nie wiem jeszcze co to, ale napewno służy do ustawiania jakiejś zmiennej
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon) //ale po co są dwie?
{
    int32_t dec_portion, frac_portion; //ustawia dwie zmmienne: do części dziesiętnej i ułamkowej
    int32_t abs_lat_or_lon = labs(lat_or_lon); //to chyba służy do wyliczenia współrzędnych absolutnych

    //  wyodrębnia część dziesiętną
    dec_portion = abs_lat_or_lon / 10000000UL;

    // a to wyodrębnia część ułamkową
    frac_portion = abs_lat_or_lon - dec_portion*10000000UL;

    // będzie pisało wynik, ale z minusem 
    if( lat_or_lon < 0 ) { // ok mamy odczytane współrzędne mniejsze od 0, więc minus ma sens
        s->printf("-"); //pisze - , ale czym jest s
    }
    s->printf("%ld.%07ld",(long)dec_portion,(long)frac_portion); // ok jakś dziwny sposób na napisanie wyniku z częścią całkowitą i ułamkową
}

// pętla, kolejna
void loop()
{
    static uint32_t last_msg_ms; ///bardzo dziwna zmienna widoczna tylko dla tej funkcji

    // aktulizuje stan GPS
    gps.update();

    // jeśli stan się zmieni to wyśle odpowiednią informację do konsoli
    // polegamy na czasie klassy GPS i ostatniej wiadomości
    // Po odebraniu nowej wiadomości czas w klasie GPS zostanie zaktualizowany
    if (last_msg_ms != gps.last_message_time_ms()) { //porównuje czas
        // resetuje czas wiadomości
        last_msg_ms = gps.last_message_time_ms();

        // uzyskuje lokalizacjie
        const Location &loc = gps.location();

        // Pisze wyniki
        hal.console->printf("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->printf(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            (double)(loc.alt * 0.01f),
                            (double)gps.ground_speed(),
                            (int)gps.ground_course(),
                            gps.num_sats(),
                            gps.time_week(),
                            (long unsigned int)gps.time_week_ms(),
                            gps.status());
    }

    //czeka 10ms
    hal.scheduler->delay(10);
}

// jakiś reset funkcji na poziomie HAL
AP_HAL_MAIN();