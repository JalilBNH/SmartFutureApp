//Bibliothèques
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pwd.h>
#include <ctime>
#include <poll.h>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <bitset>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/reader.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "wiringPi.h"
#ifndef MODRASP_MODBUS_UTIL_H
#define MODRASP_MODBUS_UTIL_H
#endif
extern "C"
{
    #include <modbus/modbus.h>
}

using namespace std;
using namespace rapidjson;
using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

//configuration
#define SIZE_OF_LINE            255//tailles lignes
#define NUMBER_OF_LINE          5000//nombres lignes
#define NUMBER_OF_COLUMN        2//nombres collonnes
#define COLONNE_TEMPS           0
#define COLONNE_RELAIS          1
#define NOMBRE_MAX_DE_RELAIS    6//nombres de relais
#define SEPARATEUR              ";"//seperateur csv scenario relais
#define MODBUS_CONFIG_PATH "/home/pi/Desktop/SmartFutureAppz/cfg/modbus_config.json"//chemin d'acces du fichier config json modbus mbh88 et rail310v

//structure mbh88 et rail310v
//***********************************//
struct modbus_config_t
{
    const char *port;//port usb convertiseur Rs485
    int baud_rate;//vitesse de transmission baud_rate
    char parity;//"E","O","N"
    int data_bit;//donnée bit
    int stop_bits;//stop bits
    int relaybox_addr;//adresse mbh88
    int powermeter1_addr;//adresse meter 1 rail310v
    int powermeter2_addr;//adresse meter 2 rail310v
    int relay_start_to_write;//relai de départ à écrire
    int relay_start_to_read;//relai de départ à lire
    int nbr_of_relays_to_write;//nombre de relais à écrire
    int nbr_of_relays_to_read;//nombre de relais à lire
    const char *scenario_file_path;//chemain d'accès fichier scénario
    const char *saved_data_path;//chemin d'accès sauvegarde des données
};

struct timer_interrupt_t
{
    int delay_measure_timer; //delais entre les mesures timer interruption
    int delay_scenario_relays; //delais entre les écritures des relais
};

//structures mbh88
//***********************************//
struct mbh88relaystate_t
{
    int relayn1; //relais 2
    int relayn2; //relais 3
    int relayn3; //relais 4
    int relayn4; //relais 5
    int relayn5; //relais 6
    int relayn6; //relais 7
    int relayn7 = 0; // relais 1 non utilisé
    int relayn8 = 0; // relais 8 non utilisé
};

struct mbh88relays_t
{
    uint16_t relay1;
    uint16_t relay2;
    uint16_t relay3;
    uint16_t relay4;
    uint16_t relay5;
    uint16_t relay6;
    uint16_t relay7;
    uint16_t relay8;
};

struct mbh88struct_t
{
    uint16_t firmware_revision;
    uint16_t model_number;
    uint16_t hardware_revision;
};
//***********************************//

//structure rail310v
//***********************************//
struct measureSlave_t
{
    int first_powermeter_register;//first register to read
    int size_of_data;//registre mesure


    //double power_scale_Kp;//config W

    //int power_scale_Kp_address;//config W

    //int meter_kW_address;//config W
    std::vector <double> meter_kW;//config W

    //double amps_scale_Ki;//config I
    //int amps_scale_Ki_address;//config I
    //int meter_Amp_address;//config I
    std::vector <double> meter_Amps;//config I

    //double volt_scale_Kv;//config V
    //int volt_scale_Kv_address;//config V
    //int meter_Volt_address;//config V
    std::vector <double> meter_Volt;//config V

    //int meter_PF_address;//config PF
    std::vector <double> meter_PF;//config PF
};
//***********************************//

//variables globales mbh88 et rail310v
//***********************************************************//
modbus_config_t modbus_config;//mbh88 config
modbus_t *ctx;// modbus contexte
timer_interrupt_t timer_settings; //timer interrup config

//variables globales mbh88
//***********************************************************//
mbh88struct_t mbh88;
mbh88relays_t mbh88relays;
mbh88relaystate_t mbh88data;
bool modbus_status_ok = false;
FILE* file_scenario;//fichier csv du mbh88
char ligne[SIZE_OF_LINE];
unsigned int nombre_de_ligne = 0;
unsigned int scenario[NUMBER_OF_LINE][NUMBER_OF_COLUMN];
unsigned char etat_relais[NOMBRE_MAX_DE_RELAIS];
char delays_min;//delais minute scenario
//***********************************************************//

//variables globales rail310v
//***********************************************************//
measureSlave_t slave_measures_1;//mesures Meter 1
measureSlave_t slave_measures_2;//mesures Meter 2
bool modbus_status1_ok = false;//meter 1
bool modbus_status2_ok = false;//meter 2
float Pva_1[10];//P apparente meter 1
float Pva_2[10];//P apparente meter 2
FILE* file_record ;//fichier CSV enregistrement des mesures Rail310v
int h, minute, s;//heure minute seconde
time_t now;//time stamp
int Init_csv;//initialisation premiere ligne (U1,U2,U3) fichier csv enregistrement mesures
int now_stamp;
//***********************************************************//

//appel fonction
int get_modbus_config_from_json_MBH88(const char *json_path, modbus_config_t &modbus_config);//mbh88
int get_modbus_config_from_json_Rail310V(const char *jsonPath, modbus_config_t &modbus_config, measureSlave_t &measures);//rail310v
int get_timer_settings_from_Json(const char *jsonPath, timer_interrupt_t &timer_settings);//rail310v mbh88
int init_modbus(modbus_config_t &modbus_config);//mbh88 et rail310v
int select_modbus_device(void);//mbh88 et rail310v
void display_config_file_json(void);
int read_holding_registers_mbh88(int reg_start, int reg_nbr, uint16_t tab_reg16[64]);//mbh88
int read_mbh88_config(uint16_t mbh88_config[3]);//mbh88
int read_relays_state_mbh88(int relay_start, int nbr_relays, uint8_t tab_reg8[64]);//mbh88
int write_1_relays_state_mbh88(int relays, int state);//mbh88
int write_relays_mbh88(int relay_start, int relays_nbr);//mbh88
int open_csv_mbh88(void);//mbh88
int read_csv_mbh88(void);//mbh88
int scenario_relays_mbh88(void);//mbh88
int fast_read_Slave_Measures_Rail310V(measureSlave_t &measure);//rail310v
void display_measures_Meter_1(float *Pva_1);//rail310v
void display_measures_Meter_2(float *Pva_2);//rail310v
void file_csv_record_Rail310v(time_t &now, int now_stamp, float *Pva_1, float *Pva_2);//rail310v
void hour_time(int &h, int &minute, int &s, time_t &now);//rail310v
void Reading_measurement_Rail310V(int signal);//rail310v
void custom_timer_seconds (long int seconds);//rail310v
void custom_reload_timer (void);//rail310v
void display_logo(void); //affichage logo SMARTFUTURE

//***********************************************************//

int main()
{
    //variables locales
    uint16_t tab_reg16[64];//registres mbh88
    //uint8_t tab_reg8[64];//registres mbh88
    //int etat_relais;//etat relais mbh88
    //***********************************************************//
    hour_time(h,minute,s,now);//lancement fonc recuperation time stamp
    now_stamp = now;

    //Rail310v
    //***********************************************************//
    // Initialisation des mesures meter 1
    int numPhase1 = 3;
    slave_measures_1.meter_PF.resize(numPhase1);
    slave_measures_1.meter_kW.resize(numPhase1);
    slave_measures_1.meter_Amps.resize(numPhase1);
    slave_measures_1.meter_Volt.resize(numPhase1);

    // Initialisation des mesures meter 2
    int numPhase2 = 3;
    slave_measures_2.meter_PF.resize(numPhase2);
    slave_measures_2.meter_kW.resize(numPhase2);
    slave_measures_2.meter_Amps.resize(numPhase2);
    slave_measures_2.meter_Volt.resize(numPhase2);
    //***********************************************************//
    display_logo();//affiche logo SMARTFUTURE
    //***********************************************************//
    get_timer_settings_from_Json(MODBUS_CONFIG_PATH, timer_settings);//config Json recuperation timer interrup et delais relais
    //***********************************************************//
    //meter 1
    //***********************************************************//
    get_modbus_config_from_json_Rail310V(MODBUS_CONFIG_PATH, modbus_config, slave_measures_1);//config Json pour le meter 1

    init_modbus(modbus_config);

    modbus_set_slave(ctx, modbus_config.powermeter1_addr);

    if  (select_modbus_device() < 0)
    {
        cout << "FIN Processus" << endl;
        exit(0);
    }
    else
    {
        cout << " RAil310V meter 1" << endl;
    }

    modbus_status1_ok = (fast_read_Slave_Measures_Rail310V(slave_measures_1) > 0);//lectures des mesures meter 1

    //---------------------------------------------------------------------------------------------------//
    //meter 2
    //---------------------------------------------------------------------------------------------------//
    get_modbus_config_from_json_Rail310V(MODBUS_CONFIG_PATH, modbus_config, slave_measures_2);

    init_modbus(modbus_config);


    modbus_set_slave(ctx, modbus_config.powermeter2_addr);

    if  (select_modbus_device() < 0)
    {
        cout << "FIN Processus" << endl;
        exit(0);
    }
    else
    {
        cout << " RAil310V meter 2" << endl;
    }

    modbus_status2_ok = (fast_read_Slave_Measures_Rail310V(slave_measures_2) > 0);//lectures des mesures meter 2

    //***********************************************************//
    //mbh88
    //***********************************************************//
    get_modbus_config_from_json_MBH88(MODBUS_CONFIG_PATH, modbus_config);

    init_modbus(modbus_config);

    modbus_set_slave(ctx, modbus_config.relaybox_addr);

    if(select_modbus_device() < 0)//si erreur
    {
        cout << "FIN Processus" << endl;
        exit(0);
    }
    else
    {
        cout << " MBH88" << endl;
    }

    read_holding_registers_mbh88(1, 3, tab_reg16);

    Init_csv = 1;//initialisation premiere ligne du fichier csv enregistrement  mesures

    display_config_file_json();//affiche la config JSON

    open_csv_mbh88();//ouverture fichier csv scenario relais

    read_csv_mbh88();//lire l'etat des relais

    custom_reload_timer();//timer interruption pour les mesures rail310v

    scenario_relays_mbh88();//scenario relais

    sleep(5);

    //***********************************************************//
    cout << "\t------------FIN----Programme-----------------------\n";
    return 0;

}

//fonctions mbh88 et rail310v
//***********************************************************//
int init_modbus(modbus_config_t &modbus_config)//mbh88 et rail310v
{
    ctx = modbus_new_rtu(modbus_config.port, modbus_config.baud_rate, modbus_config.parity, modbus_config.data_bit, modbus_config.stop_bits);
    if (ctx == NULL)
    {
        cout << endl;
        fprintf(stderr, "Impossible de creer le contexte Modbus \n");
        cout << "Erreur fonction init_modbus_Rail310V" << endl;
        cout << endl;
        cout << "FIN Processus" << endl;
        exit(0);
    }
    return 1;
}

int select_modbus_device(void)//mbh88 et rail310v
{

    if (modbus_connect(ctx) == -1)
    {
        cout << endl;
        fprintf(stderr, "Connection Impossible: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        cout << "RS485 USB CONVERT is not connected "<< endl;
        cout << endl;
        exit (0);
    }
    else
    {
        printf("\tModbus connecte :");

    }
    return 0;
}

int get_timer_settings_from_Json(const char *jsonPath, timer_interrupt_t &timer_settings)//rail310v mbh88
{
    #define JSON_GET_OR(type, name, defaultVal) (type) (root_obj.HasMember(name) ? root[name].Get<type>() : (defaultVal));

    ifstream jsonFile;
    Document root;
    bool parsingSuccessful;

    jsonFile.open(jsonPath, ios::in);
    if (!jsonFile.is_open())
    {
        cout << endl;
        cout << "Impossible d'ouvrir le fichier : modbus_config.json !" << endl;
        cout << endl;
        exit (0);
    }
    else
    {
        IStreamWrapper isw(jsonFile);
        root.ParseStream(isw);
        parsingSuccessful = !root.HasParseError();
        jsonFile.close();
        if (!parsingSuccessful)
        {
            cout << endl;
            cout << "Erreur d'analyse du fichier : modbus_config.json !" << endl;
            cout << endl;
            exit (0);
        }
    }
    const auto &root_obj = root.GetObject();
    timer_settings.delay_measure_timer = JSON_GET_OR(int, "delay_measure_timer",2);
    timer_settings.delay_scenario_relays= JSON_GET_OR(int, "delay_scenario_relays",60);


    #undef JSON_GET_OR

    return 1;
}

void hour_time(int &h, int &minute, int &s, time_t &now)//rail310v et mbh88
{
  time(&now);// Renvoie l'heure actuelle
  struct tm *local = localtime(&now);
  h = local->tm_hour;//heures
  minute = local->tm_min;//minutes
  s = local->tm_sec;//secondes
}

void display_config_file_json(void)//rail310v et mbh88
{
        cout << endl;
        cout << "\tConfig Fichier JSON  RAIL310V  Meter 1  Et  Meter 2  Et  MBH88" << endl;
        cout << endl;
        cout << "\tPort Connection USB Converter RS485  : " << modbus_config.port << endl;//affiche le port connexion
        cout << "\tVittesse De Transmission (baud_rate) : " << modbus_config.baud_rate << endl;//affiche les baud_rate
        cout << "\tData_bit                             : " << modbus_config.data_bit << endl;//affiche les données bits
        cout << "\tStop_bits                            : " << modbus_config.stop_bits << endl;//affiche les stop bits
        cout << "\tParite                               : " << modbus_config.parity << endl;//affiche la parite
        cout << "\tAdresse Meter 1           : " << modbus_config.powermeter1_addr << endl;//affiche l'adresse meter 1
        cout << "\tAdresse Meter 2           : " << modbus_config.powermeter2_addr << endl;//affiche l'adresse meter 2
        cout << "\tAdresse MBH88             : " << modbus_config.relaybox_addr << endl;//affiche l'adresse mbh88 3
        cout << "\tDelais mesures timer      : " << timer_settings.delay_measure_timer << " sec" << endl; //affiche le delais des mesures du meter
        cout << "\tDelais scenario relais    : " << timer_settings.delay_scenario_relays << " sec" << endl; // affiche le delais des écritures relais
        cout << "\tRelais de depart a ecrire : " << modbus_config.relay_start_to_write << endl;
        cout << "\tRelais de depart a lire   : " << modbus_config.relay_start_to_read << endl;
        cout << "\tNb de relais a ecrire     : " << modbus_config.nbr_of_relays_to_write << endl;
        cout << "\tNb de relais a lire       : " << modbus_config.nbr_of_relays_to_read << endl;
        cout << "\tChemin d'acces scenario   : " << modbus_config.scenario_file_path << endl;
        cout << "\tChemin d'acces donnees    : " << modbus_config.saved_data_path << endl;
        cout << endl;
        cout << endl;
        sleep(3);
}

void display_logo(void)//rail310v et mbh88
{

sleep(3);
cout <<"   ******** ****     ****     **     *******   ********** ******** **     ** ********** **     ** *******   ********"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<" **////// /**/**   **/**    ****   /**////** /////**/// /**///// /**    /**/////**/// /**    /**/**////** /**/////"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<"/**       /**//** ** /**   **//**  /**   /**     /**    /**      /**    /**    /**    /**    /**/**   /** /**"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<"/*********/** //***  /**  **  //** /*******      /**    /******* /**    /**    /**    /**    /**/*******  /*******"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<"////////**/**  //*   /** **********/**///**      /**    /**////  /**    /**    /**    /**    /**/**///**  /**////"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<"       /**/**   /    /**/**//////**/**  //**     /**    /**      /**    /**    /**    /**    /**/**  //** /**"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<" ******** /**        /**/**     /**/**   //**    /**    /**      //*******     /**    //******* /**   //**/********"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(100));
cout <<"////////  //         // //      // //     //     //     //        ///////      //      ///////  //     // ////////"<<endl;
std::this_thread::sleep_for(std::chrono::milliseconds(300));
cout << endl;
sleep(2);

}

//fonctions mbh88
//***********************************************************//
int get_modbus_config_from_json_MBH88(const char *json_path, modbus_config_t &modbus_config)//mbh88
{
    #define JSON_GET_OR(type, name, defaultVal) (type) (root_obj.HasMember(name) ? root[name].Get<type>() : (defaultVal));

    ifstream jsonFile;
    Document root;
    bool parsingSuccessful;

    jsonFile.open(json_path, ios::in);
    if (!jsonFile.is_open())
    {
        cout << endl;
        cout << "Impossible d'ouvrir le fichier : modbus_config.json !" << endl;
        cout << endl;
        exit (0);
    }
    else
    {
        IStreamWrapper isw(jsonFile);
        root.ParseStream(isw);
        parsingSuccessful = !root.HasParseError();
        jsonFile.close();
        if (!parsingSuccessful)
        {
            cout << endl;
            cout << "Erreur d'analyse du fichier : modbus_config.json !" << endl;
            cout << endl;
            exit (0);
        }
    }
    const auto &root_obj = root.GetObject();

    modbus_config.baud_rate = JSON_GET_OR(int, "baud_rate",19200);// default pour mbh88
    modbus_config.relaybox_addr = JSON_GET_OR(int, "relaybox_addr",3);// default pour mbh88
    modbus_config.data_bit = JSON_GET_OR(int, "data_bit",8);// default pour mbh88
    modbus_config.stop_bits = JSON_GET_OR(int,"stop_bits",2);// default pour mbh88
    std::string str_parity = (string) (root_obj.HasMember("parity") ? root["parity"].GetString() : ("N"));
    modbus_config.parity = str_parity.at(0); //on récupère le premier caractère
    std::string str_port = (string) (root_obj.HasMember("port") ? root["port"].GetString() : ("/dev/ttyUSB0"));
    modbus_config.port = strdup(str_port.c_str());
    modbus_config.relay_start_to_write = JSON_GET_OR(int, "relay_start_to_write",1);// default pour mbh88
    modbus_config.relay_start_to_read = JSON_GET_OR(int, "relay_start_to_read",2);
    modbus_config.nbr_of_relays_to_write = JSON_GET_OR(int, "nbr_of_relays_to_write",8);
    modbus_config.nbr_of_relays_to_read = JSON_GET_OR(int, "nbr_of_relays_to_read", 7);
    std::string str_scenario_file_path = (string) (root_obj.HasMember("scenario_file_path") ? root["scenario_file_path"].GetString() : ("/home/pi/Desktop/SmartFutureAppz/cfg/scenario_relays/scenario.csv"));
    modbus_config.scenario_file_path = strdup(str_scenario_file_path.c_str());


    #undef JSON_GET_OR
    return 1;
}

int read_holding_registers_mbh88(int reg_start, int reg_nbr, uint16_t tab_reg16[64])//mbh88
{
    int rc;

    rc = modbus_read_registers(ctx, reg_start, reg_nbr, tab_reg16);
    if (rc == -1)
    {
        cout << endl;
        cout << "Erreur fonction read_holding_registers_mbh88" << endl;
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        cout << endl;
        cout << "FIN Processus" << endl;
        exit(0);
    }
    return 1;
}

int read_relays_state_mbh88(int relay_start, int nbr_relays, uint8_t tab_reg8[64])//mbh88
{
    int rc;
    int i;

    rc=modbus_read_bits(ctx, relay_start, nbr_relays, tab_reg8);
    if (rc == -1)
    {
        cout << endl;
        cout << "Erreur relais non connectes" << endl;
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        cout << endl;
        exit (0);
    }

    for (i=0; i<rc; i++)
    {
        printf("\treg[%d]= %d (0x%X)\n",i,tab_reg8[i],tab_reg8[i]);
    }
    return 1;
}

int write_1_relays_state_mbh88(int relays, int state)//mbh88
{
    modbus_write_bit(ctx, relays, state);
    return 0;
}

int write_relays_mbh88(int relay_start, int relays_nbr)//mbh88
{
    uint8_t data_relays8[64];

    //relais remappé car le premier est KC
    data_relays8[1] = mbh88data.relayn1; //relais 2
    data_relays8[2] = mbh88data.relayn2; //relais 3
    data_relays8[3] = mbh88data.relayn3; //relais 4
    data_relays8[4] = mbh88data.relayn4; //relais 5
    data_relays8[5] = mbh88data.relayn5; //relais 6
    data_relays8[6] = mbh88data.relayn6; //relais 7
    data_relays8[0] = 0;//mbh88data.relayn7; //relais 1 non utilisé
    data_relays8[7] = 0;//mbh88data.relayn8; //relais 8 non utilisé
    modbus_write_bits(ctx, relay_start, relays_nbr, data_relays8);
    return 0;
}

int open_csv_mbh88(void)//mbh88
{

    file_scenario = fopen( modbus_config.scenario_file_path , "r") ;

    if (file_scenario==NULL)
    {
        cout << endl;
        cout << "Ouverture fichier CSV scenario impossible !" << endl;
        cout << endl;
        exit(0);
    }
    return 0;
}

int read_csv_mbh88(void)//mbh88
{
    fgets( ligne, SIZE_OF_LINE, file_scenario);

    //lecture du fichier
    while( fgets( ligne, SIZE_OF_LINE, file_scenario) != NULL )
    {
        char *ptr_col1=strtok(ligne,SEPARATEUR);
        scenario[nombre_de_ligne][COLONNE_TEMPS]=atoi(ptr_col1);

        char *ptr_col2=strtok(NULL,SEPARATEUR);
        scenario[nombre_de_ligne][COLONNE_RELAIS]=atoi(ptr_col2);

        nombre_de_ligne++;
    }

    fclose(file_scenario);
    return 0;
}

int scenario_relays_mbh88(void)//mbh88
{
    unsigned int i;
    unsigned int r;
    uint8_t tab_reg8[64];

    //printf("ligne \t time \t rel_0 \t rel_1 \t rel_2 \t rel_3 \t rel_4 \t rel_5 \n");
    for(i=0; i < nombre_de_ligne ; i++)
    {
        float time_elapsed = 0;//temps ecoulé
        int init_sleep = 5;//while
        float remaining_time = 0;//durée restante pour le sleep
        float delays_sleep;//delais sleep
        delays_sleep = timer_settings.delay_scenario_relays;//la variable delais du sleep prend la valeur de la config

        //delays_min = scenario[i+1][COLONNE_TEMPS]-scenario[i][COLONNE_TEMPS];

        //printf("%d \t %d \t ",i,scenario[i][COLONNE_TEMPS]);
        for(r=0 ; r < NOMBRE_MAX_DE_RELAIS ; r++)
        {
            etat_relais[r] = (scenario[i][COLONNE_RELAIS]>>r)&0x01;
            //printf("%d \t", etat_relais[r] );

        }
        cout << endl;
        cout << "\tEtats des Relais " << endl;
        cout << "\t*******************" << endl;
        printf("\tr0 = %d\n",etat_relais[0]);
        printf("\tr1 = %d\n",etat_relais[1]);
        printf("\tr2 = %d\n",etat_relais[2]);
        printf("\tr3 = %d\n",etat_relais[3]);
        printf("\tr4 = %d\n",etat_relais[4]);
        printf("\tr5 = %d\n",etat_relais[5]);
        cout << "\t*******************" << endl;
        cout << endl;

        mbh88data.relayn1 = etat_relais[0];
        mbh88data.relayn2 = etat_relais[1];
        mbh88data.relayn3 = etat_relais[2];
        mbh88data.relayn4 = etat_relais[3];
        mbh88data.relayn5 = etat_relais[4];
        mbh88data.relayn6 = etat_relais[5];

        modbus_set_slave(ctx, modbus_config.relaybox_addr);

        if(write_relays_mbh88(modbus_config.relay_start_to_write, modbus_config.nbr_of_relays_to_write) < 0)
        {
            return -1;
        }

        if(read_relays_state_mbh88(modbus_config.relay_start_to_read, modbus_config.nbr_of_relays_to_read, tab_reg8) < 0)
        {
            return -1;
        }

        while ( init_sleep > 1)//tant que init_sleep > 1
        {

            auto IN0 =duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();//chronometre  ms debut
            sleep(delays_sleep);
            auto END0 =duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();//chronometre ms fin
            time_elapsed = (END0-IN0)/1000;
            remaining_time = (delays_sleep-time_elapsed);

            if (remaining_time == 0)
            {
                init_sleep = 0;
            }
            else
            {
                delays_sleep = remaining_time;
            }

        }
    }
    return 1;
}

//fonctions rail310v
//***********************************************************//
int get_modbus_config_from_json_Rail310V(const char *jsonPath, modbus_config_t &modbus_config, measureSlave_t &measures)//Rail310v
{
    #define JSON_GET_OR(type, name, defaultVal) (type) (root_obj.HasMember(name) ? root[name].Get<type>() : (defaultVal));

    ifstream jsonFile;
    Document root;
    bool parsingSuccessful;

    jsonFile.open(jsonPath, ios::in);
    if (!jsonFile.is_open())
    {
        cout << endl;
        cout << "Impossible d'ouvrir le fichier : modbus_config.json !" << endl;
        cout << endl;
        exit (0);
    }
    else
    {
        IStreamWrapper isw(jsonFile);
        root.ParseStream(isw);
        parsingSuccessful = !root.HasParseError();
        jsonFile.close();
        if (!parsingSuccessful)
        {
            cout << endl;
            cout << "Erreur d'analyse du fichier : modbus_config.json !" << endl;
            cout << endl;
            exit (0);
        }
    }
    const auto &root_obj = root.GetObject();
    std::string str_port = (string) (root_obj.HasMember("port") ? root["port"].GetString() : ("/dev/ttyUSB0"));
    modbus_config.port = strdup(str_port.c_str());
    modbus_config.powermeter1_addr = JSON_GET_OR(int, "powermeter1_addr",1);
    modbus_config.powermeter2_addr = JSON_GET_OR(int, "powermeter2_addr",2);
    modbus_config.baud_rate = JSON_GET_OR(int, "baud_rate",19200);
    modbus_config.data_bit = JSON_GET_OR(int, "data_bit",8);
    std::string str_parity = (string) (root_obj.HasMember("parity") ? root["parity"].GetString() : ("N"));
    modbus_config.parity = str_parity.at(0); //on récupère le premier caractère
    modbus_config.stop_bits = JSON_GET_OR(int, "stop_bits",2);
    measures.first_powermeter_register = JSON_GET_OR(int, "first_powermeter_register",-1);
    measures.size_of_data = JSON_GET_OR(int, "size_of_data",19);
    std::string str_saved_data_path = (string) (root_obj.HasMember("saved_data_path") ? root["saved_data_path"].GetString() : ("/home/pi/Desktop/SmartFutureAppz/cfg/data_record/"));
    modbus_config.saved_data_path= strdup(str_saved_data_path.c_str());


    #undef JSON_GET_OR

    return 1;
}

int fast_read_Slave_Measures_Rail310V(measureSlave_t &measure)//Rail310V
{
    int rc;
    char size_of_data=19;
    uint16_t tab_reg[size_of_data]; //valeurs brutes lues depuis le Meter

    rc = modbus_read_registers(ctx, measure.first_powermeter_register, size_of_data, tab_reg);
    if (rc == -1) {
        cout << endl;
        fprintf(stderr, "Erreur dans fast_read_slave_mesures() = %s\n", modbus_strerror(errno));
        cout << endl;
        exit (0);
    }
    else
    {
        //à customiser selon le meter
        measure.meter_kW[0] = tab_reg[0] * pow(10,tab_reg[4]-3);
        measure.meter_kW[1] = tab_reg[1] * pow(10,tab_reg[4]-3);
        measure.meter_kW[2] = tab_reg[2] * pow(10,tab_reg[4]-3);

        measure.meter_Amps[0] = tab_reg[5] * pow(10,tab_reg[9]-3);
        measure.meter_Amps[1] = tab_reg[6] * pow(10,tab_reg[9]-3);
        measure.meter_Amps[2] = tab_reg[7] * pow(10,tab_reg[9]-3);

        measure.meter_Volt[0] = tab_reg[10] * pow(10,tab_reg[14]-3);
        measure.meter_Volt[1] = tab_reg[11] * pow(10,tab_reg[14]-3);
        measure.meter_Volt[2] = tab_reg[12] * pow(10,tab_reg[14]-3);

        if(measure.meter_Amps[0] != 0 && measure.meter_Volt[0] != 0)
            measure.meter_PF[0] = measure.meter_kW[0] / ( measure.meter_Amps[0] * measure.meter_Volt[0] );
        else
            measure.meter_PF[0] = tab_reg[15] * pow(10,0-3);

        if(measure.meter_Amps[1] != 0 && measure.meter_Volt[1] != 0)
            measure.meter_PF[1] = measure.meter_kW[1] / ( measure.meter_Amps[1] * measure.meter_Volt[1] );
        else
            measure.meter_PF[1] = tab_reg[16] * pow(10,0-3);

        if(measure.meter_Amps[2] != 0 && measure.meter_Volt[2] != 0)
            measure.meter_PF[2] = measure.meter_kW[2] / ( measure.meter_Amps[2] * measure.meter_Volt[2] );
        else
            measure.meter_PF[2] = tab_reg[17] * pow(10,0-3);
    }

    return 1;
}

void display_measures_Meter_1(float *Pva_1)//rail310v
{
    //P apparente = U * I
    Pva_1[0]=slave_measures_1.meter_Volt[0] * slave_measures_1.meter_Amps[0];//meter 1
    Pva_1[1]=slave_measures_1.meter_Volt[1] * slave_measures_1.meter_Amps[1];//meter 1
    Pva_1[2]=slave_measures_1.meter_Volt[2] * slave_measures_1.meter_Amps[2];//meter 1

    //affiche mesures meter 2
    cout << "\t*******************meter1*********************************************************************************\n";
    cout << "\tTension 1 : " << slave_measures_1.meter_Volt[0] << "V   ";
    cout << "\tIntensite 1 : " << slave_measures_1.meter_Amps[0] << "A   ";
    cout << "\tP active 1 : " << slave_measures_1.meter_kW[0] << "W   ";
    cout << "\tP apparente 1 : " << Pva_1[0] << "VA   ";
    cout << "\tPF 1 : " << slave_measures_1.meter_PF[0];
    cout << endl;

    cout << "\tTension 2 : " << slave_measures_1.meter_Volt[1] << "V   ";
    cout << "\tIntensite 2 : " << slave_measures_1.meter_Amps[1] << "A   ";
    cout << "\tP active 2 : " << slave_measures_1.meter_kW[1] << "W   ";
    cout << "\tP apparente 2 : " << Pva_1[1] << "VA   ";
    cout << "\tPF 2 : " << slave_measures_1.meter_PF[1];
    cout << endl;

    cout << "\tTension 3 : " << slave_measures_1.meter_Volt[2] << "V   ";
    cout << "\tIntensite 3 : " << slave_measures_1.meter_Amps[2] << "A   ";
    cout << "\tP active 3 : " << slave_measures_1.meter_kW[2] << "W   ";
    cout << "\tP apparente 3 : " << Pva_1[2] << "VA   ";
    cout << "\tPF 3 : " << slave_measures_1.meter_PF[2];
    cout << endl;
    cout << "\t**********************************************************************************************************\n";
    cout << endl;
    cout << endl;

}

void display_measures_Meter_2(float *Pva_2)//rail310v
{
    //P apparente = U * I
    Pva_2[0]=slave_measures_2.meter_Volt[0] * slave_measures_2.meter_Amps[0];//meter 2
    Pva_2[1]=slave_measures_2.meter_Volt[1] * slave_measures_2.meter_Amps[1];//meter 2
    Pva_2[2]=slave_measures_2.meter_Volt[2] * slave_measures_2.meter_Amps[2];//meter 2

    //affiche mesures meter 2
    cout << "\t*******************meter2*********************************************************************************\n";
    cout << "\tTension 4 : " << slave_measures_2.meter_Volt[0] << "V   ";
    cout << "\tIntensite 4 : " << slave_measures_2.meter_Amps[0] << "A   ";
    cout << "\tP active 4 : " << slave_measures_2.meter_kW[0] << "W   ";
    cout << "\tP apparente 4 : " << Pva_2[0] << "VA   ";
    cout << "\tPF 4 : " << slave_measures_2.meter_PF[0];
    cout << endl;

    cout << "\tTension 5 : " << slave_measures_2.meter_Volt[1] << "V   ";
    cout << "\tIntensite 5 : " << slave_measures_2.meter_Amps[1] << "A   ";
    cout << "\tP active 5 : " << slave_measures_2.meter_kW[1] << "W   ";
    cout << "\tP apparente 5 : " << Pva_2[1] << "VA   ";
    cout << "\tPF 5 : " << slave_measures_2.meter_PF[1];
    cout << endl;

    cout << "\tTension 6 : " << slave_measures_2.meter_Volt[2] << "V   ";
    cout << "\tIntensite 6 : " << slave_measures_2.meter_Amps[2] << "A   ";
    cout << "\tP active 6 : " << slave_measures_2.meter_kW[2] << "W   ";
    cout << "\tP apparente 6 : " << Pva_2[2] << "VA   ";
    cout << "\tPF 6 : " << slave_measures_2.meter_PF[2];
    cout << endl;
    cout << "\t**********************************************************************************************************\n";
    cout << endl;
    cout << endl;
}

void file_csv_record_Rail310v(time_t &now, int now_stamp, float *Pva_1, float *Pva_2)//rail310v
{
    char path[100];
    char timestamp_file[100] = "";





    sprintf(timestamp_file,"%d.csv",now_stamp);
    //if (sprintf(path, "/home/pi/Desktop/SmartFutureAppz/cfg/data_record/%s",timestamp_file) == 0)//creer un nom unique (time_stamp) pour chaque scenario executer
    if (sprintf(path, "%s/%s", modbus_config.saved_data_path,timestamp_file) == 0)//creer un nom unique (time_stamp) pour chaque scenario executer
    {
        cout << "Erreur sprintf dans la fonction file_csv_record_Rail310v " << endl;
        exit(0);
    }

    file_record = fopen( path, "a+");

    if (file_record==NULL)
    {
        cout << "Ouverture fichier impossible !" << endl;
        exit(0);
    }
    if (Init_csv == 1)
    {
        //ecriture mesures fichier CSV
        //fputs("timestamp;U1;U2;U3;U4;U5;U6;I1;I2;I3;I4;I5;I6;PW1;PW2;PW3;PW4;PW5;PW6;PVA1;PVA2;PVA3;PVA4;PVA5;PVA6;PF1;PF2;PF3;PF4;PF5;PF6",file_record);
        fputs("timestamp(s);U1(V);U2(V);U3(V);U4(V);U5(V);U6(V);I1(A);I2(A);I3(A);I4(A);I5(A);I6(A);PW1(W);PW2(W);PW3(W);PW4(W);PW5(W);PW6(W);PVA1(VA);PVA2(VA);PVA3(VA);PVA4(VA);PVA5(VA);PVA6(VA);PF1;PF2;PF3;PF4;PF5;PF6",file_record);
        fputs("\n",file_record);
    }

    fprintf(file_record,"%d",(unsigned int) now);
    fputs(";",file_record);

    //meter 1 tension
    fprintf(file_record,"%.2f",slave_measures_1.meter_Volt[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_Volt[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_Volt[2]);
    fputs(";",file_record);
    //meter 2 tension
    fprintf(file_record,"%.2f",slave_measures_2.meter_Volt[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_Volt[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_Volt[2]);
    fputs(";",file_record);

    //meter 1 intensite
    fprintf(file_record,"%.2f",slave_measures_1.meter_Amps[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_Amps[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_Amps[2]);
    fputs(";",file_record);
    //meter 2 intensite
    fprintf(file_record,"%.2f",slave_measures_2.meter_Amps[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_Amps[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_Amps[2]);
    fputs(";",file_record);

    //meter 1 P active
    fprintf(file_record,"%.2f",slave_measures_1.meter_kW[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_kW[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_1.meter_kW[2]);
    fputs(";",file_record);
    //meter 2 P active
    fprintf(file_record,"%.2f",slave_measures_2.meter_kW[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_kW[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",slave_measures_2.meter_kW[2]);
    fputs(";",file_record);

    //meter 1 P apparente
    fprintf(file_record,"%.2f",Pva_1[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",Pva_1[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",Pva_1[2]);
    fputs(";",file_record);
    //meter 2 P apparente
    fprintf(file_record,"%.2f",Pva_2[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",Pva_2[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.2f",Pva_2[2]);
    fputs(";",file_record);

    //meter 1 PF
    fprintf(file_record,"%.3f",slave_measures_1.meter_PF[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.3f",slave_measures_1.meter_PF[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.3f",slave_measures_1.meter_PF[2]);
    fputs(";",file_record);
    //meter 2 PF
    fprintf(file_record,"%.3f",slave_measures_2.meter_PF[0]);
    fputs(";",file_record);
    fprintf(file_record,"%.3f",slave_measures_2.meter_PF[1]);
    fputs(";",file_record);
    fprintf(file_record,"%.3f",slave_measures_2.meter_PF[2]);
    fputs("\n",file_record);

    //close file csv
    fclose(file_record);

    Init_csv = 0;
}

void Reading_measurement_Rail310V(int signo)//rail310v
{

    if (modbus_status1_ok)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            modbus_set_slave(ctx, modbus_config.powermeter1_addr);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            modbus_status1_ok = (fast_read_Slave_Measures_Rail310V(slave_measures_1) > 0);//lectures des mesures meter 1
        }
    else//si erreur
    {
        cout << endl;
        cout << "---------------- Meter 1 pas alimenter ------------" << endl;
        cout << endl;
        cout << "FIN Processus" << endl;
        exit(0);
    }

    if (modbus_status2_ok)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        modbus_set_slave(ctx,modbus_config.powermeter2_addr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        modbus_status2_ok = (fast_read_Slave_Measures_Rail310V(slave_measures_2) > 0);//lectures des mesures meter 2
    }
    else//si erreur
    {
        cout << endl;
        cout << "----------------- Meter 2 pas alimenter -----------" << endl;
        cout << endl;
        cout << "FIN Processus" << endl;
        exit(0);
    }

    cout << endl;
    hour_time(h,minute,s,now);//fonct heures minutes secondes
    printf("\tL'heure : %02d:%02d:%02d\n", h, minute, s);//affiche l'heures minutes secondes
    printf ("\ttime stamp = %d\n",(unsigned int) now);//affiche time stamp
    cout << endl;
    cout << endl;

    display_measures_Meter_1(Pva_1);//affiches mesures meter 1
    display_measures_Meter_2(Pva_2);//affiches mesures meter 2
    file_csv_record_Rail310v(now,now_stamp,Pva_1,Pva_2);//enregistres  mesures dans le fichier csv

}

void custom_timer_seconds(long int seconds)//rail310v
{
	struct itimerval timer1;

	timer1 . it_interval . tv_usec = 0;
	timer1 . it_interval . tv_sec = seconds;
	timer1 . it_value . tv_usec = 0;
	timer1 . it_value . tv_sec = seconds;
	setitimer ( ITIMER_REAL, &timer1, NULL );
}

void custom_reload_timer(void)//rail310v
{
	signal (SIGALRM, Reading_measurement_Rail310V);
	custom_timer_seconds (timer_settings.delay_measure_timer) ;
}










