#include <xc.h>
#include <string.h>
#include <stdlib.h>

#include "configuration.c"
#define _XTAL_FREQ 32000000
#define T 33333/107*2
#define STREAM_LENGTH 16

#define T 33333/107*2
#define MAX_LENGTH 700

#define FRAME_SIZE 25
#define DECODE_SIZE 128
#define FC 45
#define FS 8000
#define TABLE_WIDTH 24
#define PACKAGE_SIZE 7
#define NUMBER_OF_PACKETS 6
#define AUDIO_PACKET_SIZE 112
#define idAddress 0

//LED VARS:
static volatile int packet_number = 0;
static volatile int number_of_packets = 0;
static volatile int state_led=0;
static volatile int flash_interrupt;
static volatile int led_para;
unsigned int count; //Per les interrupcions

static const unsigned char H[3][7] = {
    {1, 0, 1, 0, 1, 0, 1},
    {0, 1, 1, 0, 0, 1, 1},
    {0, 0, 0, 1, 1, 1, 1}
};

unsigned char z[3];
unsigned int error;
unsigned char corrected[7];
unsigned char datadecoded[4];


unsigned char output[9];
unsigned char decodedchar;
unsigned char id;
unsigned char year;
unsigned char month;
unsigned char day;
unsigned char weekday;
unsigned char hora;
unsigned char minut;
unsigned char segon;

unsigned char decoded[DECODE_SIZE];

unsigned char buffer[MAX_LENGTH];
unsigned short head;
unsigned short tail;


signed short sample[FRAME_SIZE];
signed short state_audio;
signed short pow;
signed short thresh;
signed short thresh2;

signed short index;
signed short frames_per_chirp;

unsigned int k;

unsigned short data;
unsigned short valid_items;

bit para = 0;

bit put_data(unsigned char dat);

unsigned char read_data(void);
bit FIFO_is_empty(void);
void config_ADC(void);
unsigned char* audioread(void);
void process_data(void);
void config_date(unsigned char []);
//void interrupt clint(void);


signed short listen_threshold;
signed short sync_false_alarm_threshold;
signed short decision_thresh;

/*void interrupt clint(void) {
    if (PIR1bits.ADIF == 1) {

        data = ADRESHbits.ADRES;
        LATCbits.LATC7 = ~(LATCbits.LATC7);
        if (!put_data(data)) {

            para = 1;
        }

        //CLEAR INTERRUPT FLAG
        PIR1bits.ADIF = 0;
        //ADC INTERRUPT ENABLE
        PIE1bits.ADIE = 1;
        //GLOBAL INTERRUPT ENABLE
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        //START CONVERSION
        ADCON0bits.GO = 1;
    }
}*/

void config_ADC_2(void) {
    //I/O PORT CONFIGURATION
    TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSA0 = 1;
    //CHANNEL SELECT
    ADPCHbits.ADPCH = 0x00;
    //VOLTAGE REFERENCE
    ADREFbits.PREF = 0x00;
    //CLOCK
    ADCLKbits.CS = 0b111111;
    ADCON0bits.CS = 0;
    //ADC ON
    ADCON0bits.ON = 1;
    //CLEAR INTERRUPT FLAG
    PIR1bits.ADIF = 0;
    //ADC INTERRUPT ENABLE
    PIE1bits.ADIE = 1;

    //Formatting
    ADCON0bits.ADFM = 0;

    //GLOBAL INTERRUPT ENABLE
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    //ACQUISITION TIME
    /////////
    ADACQHbits.ACQ = 0b00100;
    //ACQUISITION TIME: 1 CLOCKS
    ADACQLbits.ACQ = 0x00;

    //START CONVERSION
    ADCON0bits.GO = 1;

    //SET OUTPUT
    return;
}

bit put_data(unsigned char dat) {
    if (valid_items >= MAX_LENGTH) {
        return 0;
    } else {
        valid_items++;
        buffer[tail] = dat;
        tail = (tail + 1) % MAX_LENGTH;
        return 1;
    }
}

unsigned char read_data(void) {
    while (FIFO_is_empty()&&!flash_interrupt);
    unsigned short jaja = buffer[head];
    valid_items--;
    head = (head + 1) % MAX_LENGTH;
    return jaja;
}

bit FIFO_is_empty(void) {
    if (valid_items == 0) {
        return 1;
    } else {
        return 0;
    }
}

void process_data(void) {
    pow = 0;
    for (int i = 0; i < FRAME_SIZE; i++) {
        unsigned char samplee = read_data();
        //unsigned char samplee = read_data();
        sample[i] = samplee > 128 ? (signed short) samplee - 128 : (samplee < 128 ? -(signed short) (128 - samplee) : 0);

        pow += sample[i]*((sample[i] > 0) - (sample[i] < 0)); //abs

    }
    
    switch (state_audio) {
        case 1: //LISTEN

            if (pow > listen_threshold) {

                state_audio = 2; //MAYBE SYNC
            }
            break;
            
        case 2: //SYNC

            if (pow < sync_false_alarm_threshold) {

                state_audio = 3; //MAYBE SYNC
                index = 1;
                k = 0;
            }
            break;
            
        case 3: //READ
            if (index % frames_per_chirp == frames_per_chirp / 2 && index > frames_per_chirp) {
                if (pow > listen_threshold) {
                    decoded[k] = 1;
                } else {
                    decoded[k] = 0;
                }
                k++;
                if ((k % 42) == 0){
                    state_audio = 7; //WAIT
                }
            }
            index++;
            break;
         
            
        case 7: //WAIT
            if (pow < listen_threshold) {
                state_audio = 4; //LISTEN 2
            }
            break;
            
        case 4: //LISTEN 2
            if (pow > listen_threshold) {
                state_audio = 5; //SYNC 2
            }
            break;
            
        case 5: //SYNC 2
            if (pow < sync_false_alarm_threshold) {
                state_audio = 3; 
                index = 1;
            }
            break;

        case 6: //FINISH
            break;
    }
}

unsigned char* audioread(void) {
    head = 0;
    tail = 0;
    valid_items = 0;

    para = 0;

    OSCCON1bits.NOSC = 0b110;
    OSCFRQbits.HFFRQ = 0b110;



    //initialize port A
    config_ADC_2();


    state_audio = 1;
    pow = 0;

    listen_threshold = 500;
    decision_thresh = 150;
    sync_false_alarm_threshold = 410;

    //LATCbits.LATC0 = 0;
    //LATAbits.LATA2 = 0;

    index = 0;
    k = 0;

    frames_per_chirp = FS / (FC * FRAME_SIZE);



    k = 0;
    while (!para) {

        process_data();

        switch(state_audio){
            case 3:
                LATCbits.LATC0=0;
                LATAbits.LATA2=1;
                break;
        }
       
        if (k > 112) {
            state_audio = 6;
            LATAbits.LATA2=0;
            //LATCbits.LATC0 = 0;
            //LATAbits.LATA2 = 0;

            __delay_ms(1500);
            break;
        }
        if(flash_interrupt){
            
            break;
        }
    }
    //LATCbits.LATC0 = 0;
    //LATAbits.LATA2 = 1;
    if(!flash_interrupt){

    for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 0; j < 7; j++) {
            z[i] = z[i] + H[i][j] * decoded[j];
        }
        z[i] = z[1] % 2;
    }
    error = (z[0] | z[1] << 1) | z[2] << 2;

    if (error != 0) {
        for (unsigned int j = 0; j < 7; j++) {
            corrected[j] = j == (error - 1) ? (decoded[j] == 0 ? 1 : 0) : decoded[j];
        }
    } else {
        for (unsigned int j = 0; j < 7; j++) {
            corrected[j] = decoded[j];
        }
    }

    decodedchar |= ((corrected[2] | corrected[4] << 1) | corrected[5] << 2) | corrected[6] << 3;

    output[0]=decodedchar;
    switch (decodedchar) {
        case 8: //pic configuration (id any mes dia diadelasetmana hora minuts segon)
            //LATCbits.LATC0 = 0;
            //LATAbits.LATA2 = 1;

            /*id(2 pqt) YY(2 pqt) MM(1 pqt) DD(2 pqt) weekday(2 pqt) hora(2 pqt) minuts(2 pqts) segon( 2 pqt)
           
             */
           for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 7;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 7;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 7;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            id = 0;
            id |= corrected[2] << 7 ;
            id |= corrected[4] << 6;
            id |= corrected[5] << 5;
            id |= corrected[6] << 4;

            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 14;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 14;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 14;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            id |= corrected[2] << 3;
            id |= corrected[4] << 2;
            id |= corrected[5] << 1;
            id |= corrected[6];
            output[1]=id;
            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 21;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            
            if (error != 0) {
                unsigned int l = 21;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 21;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            year = 0;
            year |= corrected[2] << 7;
            year |= corrected[4] << 6;
            year |= corrected[5] << 5;
            year |= corrected[6] << 4;


            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 28;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 28;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 28;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            year |= corrected[2] << 3;
            year |= corrected[4] << 2;
            year |= corrected[5] << 1;
            year |= corrected[6];

            output[2]=year;
            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 35;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;



            if (error != 0) {
                unsigned int l = 35;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 35;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            month = 0;
            month |= corrected[2] << 3;
            month |= corrected[4] << 2;
            month |= corrected[5] << 1;
            month |= corrected[6];

            output[3]=month;
            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 42;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;



            if (error != 0) {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            day = 0;
            day |= corrected[2] << 7;
            day |= corrected[4] << 6;
            day |= corrected[5] << 5;
            day |= corrected[6] << 4;


            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 49;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;



            if (error != 0) {
                unsigned int l = 49;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 49;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            day |= corrected[2] << 3;
            day |= corrected[4] << 2;
            day |= corrected[5] << 1;
            day |= corrected[6] ;

            output[4]=day;
            for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 56;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;



            if (error != 0) {
                unsigned int l = 56;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 56;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            weekday = 0;
            weekday |= corrected[2] << 7;
            weekday |= corrected[4] << 6;
            weekday |= corrected[5] << 5;
            weekday |= corrected[6] << 4;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 63;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 63;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 63;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            weekday |= corrected[2] << 3;
            weekday |= corrected[4] << 2;
            weekday |= corrected[5] << 1;
            weekday |= corrected[6];
            
            output[5]=weekday;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 70;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 70;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 70;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            hora = 0;
            hora |= corrected[2] << 7;
            hora |= corrected[4] << 6;
            hora |= corrected[5] << 5;
            hora |= corrected[6] << 4;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 77;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 77;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 77;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            hora |= corrected[2] << 3;
            hora |= corrected[4] << 2;
            hora |= corrected[5] << 1;
            hora |= corrected[6];

            output[6]=hora;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 84;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 84;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 84;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            minut = 0;
            minut |= corrected[2] << 7;
            minut |= corrected[4] << 6;
            minut |= corrected[5] << 5;
            minut |= corrected[6] << 4;

            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 91;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 91;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 91;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            minut |= corrected[2] << 3;
            minut |= corrected[4] << 2;
            minut |= corrected[5] << 1;
            minut |= corrected[6];

            output[7]=minut;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 98;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 98;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 98;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            segon = 0;
            segon |= corrected[2] << 7;
            segon |= corrected[4] << 6;
            segon |= corrected[5] << 5;
            segon |= corrected[6] << 4;

            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 105;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 105;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 105;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            segon |= corrected[2] << 3;
            segon |= corrected[4] << 2;
            segon |= corrected[5] << 1;
            segon |= corrected[6];
            output[8]=segon;
            break;


           
        case 3: //configuracio alarma
            // hora(2pqts) minut(2pqts) segon(2pqts) weekday(2pqts))
            //LATCbits.LATC0 = 1;
            //LATAbits.LATA2 = 0;

           for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 7;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 7;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 7;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            hora = 0;
            hora |= corrected[2] << 7;
            hora |= corrected[4] << 6;
            hora |= corrected[5] << 5;
            hora |= corrected[6] << 4;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 14;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;

            if (error != 0) {
                unsigned int l = 14;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 14;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            hora |= corrected[2] << 3;
            hora |= corrected[4] << 2;
            hora |= corrected[5] << 1;
            hora |= corrected[6];
            output[1]=hora;

             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 21;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 21;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 21;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            minut = 0;
            minut |= corrected[2] << 7;
            minut |= corrected[4] << 6;
            minut |= corrected[5] << 5;
            minut |= corrected[6] << 4;

            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 28;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 28;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 28;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            minut |= corrected[2] << 3;
            minut |= corrected[4] << 2;
            minut |= corrected[5] << 1;
            minut |= corrected[6];

            output[2]=minut;
            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 35;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 35;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 35;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            segon = 0;
            segon |= corrected[2] << 7;
            segon |= corrected[4] << 6;
            segon |= corrected[5] << 5;
            segon |= corrected[6] << 4;

            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 42;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            segon |= corrected[2] << 3;
            segon |= corrected[4] << 2;
            segon |= corrected[5] << 1;
            segon |= corrected[6];
            
            output[3]=segon;
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 49;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 49;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 49;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }


            weekday = 0;
            weekday |= corrected[2] << 7;
            weekday |= corrected[4] << 6;
            weekday |= corrected[5] << 5;
            weekday |= corrected[6] << 4;

            
             for (unsigned int i = 0; i < 3; i++) {
                unsigned int l = 42;
                z[i] = 0;
                for (unsigned int j = 0; j < 7; j++) {
                    z[i] = z[i] + H[i][j] * decoded[l];
                    l++;
                }
                z[i] = z[1] % 2;
            }

            error = (z[0] | z[1] << 1) | z[2] << 2;
            if (error != 0) {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = j == (error - 1) ? (decoded[l] == 0 ? 1 : 0) : decoded[l];
                    l++;
                }
            } else {
                unsigned int l = 42;
                for (unsigned int j = 0; j < 7; j++) {
                    corrected[j] = decoded[l];
                    l++;
                }
            }

            weekday |= corrected[2] << 3;
            weekday |= corrected[4] << 2;
            weekday |= corrected[5] << 1;
            weekday |= corrected[6];

            output[4]=weekday;
            break;
       
    }
    }
    return output;
}



////////COMENÇA CODI LED:
struct Map {
    char num0;
    char num1;
    char num2;
    char num3;
    char num4;
    char num5;
    char num6;
    char num7;
    char num8;
    char num9;
    char char1;
    char char2;
    char char3;
    char def;
};


void led_on(){
    LATAbits.LATA2 = 1;
    __delay_us(0.65*T);
}

void led_off(){
    LATAbits.LATA2 = 0;
    __delay_us(0.65*T);
}

//FLASH CODE
//General Interrupts configuration
void config_general_interrupts(){
    //Enable General Interrupts
    INTCONbits.GIE=1;
    
    //Enable Peripheral Interrupts (Timer1 Interrupt)
    INTCONbits.PEIE=1;
}
//FLASH CODE
//External Interrupts configuration
void config_external_interrupts(){
    //Set interrupts on rising edge
    INTCONbits.INTEDG=1;
    //Set ports for interrupts (RB0 for External Interrupt)
    INTPPSbits.INTPPS=0x01;
    //Enable External Interrupts
    PIE0bits.INTE=1;
    
    
    PIR0bits.INTF=0;
}

void disable_interrupts(){
    //Enable General Interrupts
    INTCONbits.GIE=0;
    
    //Enable Peripheral Interrupts (Timer1 Interrupt)
    INTCONbits.PEIE=0;
    
    //Enable External Interrupts
    PIE0bits.INTE=1;
}



void symbol_mapping(char *input_string, char *output_string){
    struct Map map1;
    map1.num0 = 0x00;
    map1.num1 = 0x01;
    map1.num2 = 0x05;
    map1.num3 = 0x15;
    map1.num4 = 0xFE;
    map1.num5 = 0xFA;
    map1.num6 = 0xEA;
    map1.num7 = 0xFF;
    map1.num8 = 0x55;
    map1.num9 = 0x08;
    map1.char1 = 0xF7;
    map1.char2 = 0xAF;
    map1.char3 = 0x50;
    map1.def = 0xCC;
    for(int i=0; i<STREAM_LENGTH; i++){
        switch(input_string[i]){
            case '0':
                output_string[i] = map1.num0;
                    break;
            case '1':
                output_string[i] = map1.num1;
                    break;
            case '2':
                output_string[i] = map1.num2;
                    break;
            case '3':
                output_string[i] = map1.num3;
                    break;
            case '4':
                output_string[i] = map1.num4;
                    break;
            case '5':
                output_string[i] = map1.num5;
                    break;
            case '6':
                output_string[i] = map1.num6;
                    break;
            case '7':
                
                output_string[i] = map1.num7;
                    break;
            case '8':
                
                output_string[i] = map1.num8;
                    break;
            case '9':
                output_string[i] = map1.num9;
                    break;
            case 'T':
                output_string[i] = map1.char1;
                    break;
            case 'M':
                output_string[i] = map1.char2;
                    break;
            case ':':
                output_string[i] = map1.char3;
                    break;
            default: 
                output_string[i] = map1.def;                
                break;
        }
    }
}

void start_sequence(){
    unsigned char seq= 0xBF;
    short window = 0x80;
    short aux = 0;
    for(int k=0;k<8;k++){
          aux = seq&window;
          int a = aux>>(7-k);
          if(a) {
              led_on();
          }
          else{
            led_off();
          }
          window = window>>1;
      }
}

void to_manchester(unsigned char data){
    unsigned char data_aux = data;
  unsigned short data_manchester;
  unsigned int i ;
 (data_manchester) = (data_manchester) << 2 ;
  for(i = 0 ; i < 8; i ++){
    if(data & 0x80) (data_manchester) |=  0x02  ; // data LSB first
    else (data_manchester) |= 0x01 ;
    if(i<7) (data_manchester) = (data_manchester) << 2 ;
    data = data << 1 ; // to next bit
  }
  unsigned short window = 0x8000;
  unsigned short aux = 0;
      for(int k=0;k<STREAM_LENGTH;k++){
          //On packet change 
          aux = data_manchester&window;
          int a = aux>>(15-k);
          if(a) {
              led_on();
          }
          else{
            led_off();
          }
          window = window>>1;
      }
  
  data = data_aux;
}
unsigned char create_checksum(unsigned char input1,unsigned char input2,unsigned char bsn){
    unsigned char checksum = 0x00;
    for(int i=0;i<8;i++){
        if(input1&0x80>>i) checksum++;
    }
    for(int i=0;i<8;i++){
        if(input2&0x80>>i) checksum++;
    }
    for(int i=0;i<8;i++){
        if(bsn&0x80>>i) checksum++;
    }
    return checksum;
}

void encode(char *input_string,int is_id){

  unsigned char bsn_mch[2];
  unsigned char data_1_mch[2];
  unsigned char data_2_mch[2];
  
  unsigned char aux_string[STREAM_LENGTH];
  unsigned char data_string[STREAM_LENGTH];
  strncpy(aux_string,input_string+(packet_number)*STREAM_LENGTH,STREAM_LENGTH);
  /*for(int i=0;i<STREAM_LENGTH;i++){
      aux_string[i] = input_string[packet_number*STREAM_LENGTH+i];
  }*/
  symbol_mapping(aux_string,data_string);
  unsigned char bsn = 0x00;
  if(is_id){
    bsn |= (packet_number%2)<<6;
  }else{
    bsn |= ((packet_number+1)%2)<<6; 
  }
  unsigned char input1;
  unsigned char input2;
  for(int i=0;i<STREAM_LENGTH;i=i+2){
        input1 = data_string[i];
        if(input_string[i+1]>0) input2 = data_string[i+1];
        else{
          input2 = 0;
        }

        start_sequence();
        to_manchester(bsn);
        to_manchester(input1);
        to_manchester(input2);
        to_manchester(create_checksum(input1,input2,bsn));
      
        start_sequence();
        to_manchester(bsn);
        to_manchester(input1);
        to_manchester(input2);
        to_manchester(create_checksum(input1,input2,bsn));

        bsn++;
  }
}

void interrupt InterruptRoutine(){
    //FLASH CODE
    if(PIR0bits.INTF==1){
        led_para = 0;
        flash_interrupt=1;
        switch(state_led){
            case 0:
                // First flash
                PIE1bits.ADIE = 0;
                state_led = 1;
                break;
            case 1:
                // Encode and send the ID
                packet_number=0;
                state_led = 2;
                PIE1bits.ADIE = 0;
                break;
            case 2:
                // Send next package when ACK received
                packet_number++;
                if(packet_number == number_of_packets+1){
                    packet_number = 0;
                    state_led = 0;
                    PIE1bits.ADIE = 0;
                    flash_interrupt = 0;
                    led_para = 1;
                }
                break;
        }
        //__delay_us(50);
        //LATCbits.LATC0 = packet_number;
        
        __delay_us(50);
        PIR0bits.INTF=0;
        __delay_us(50);
        //Return the External Interrupt Flag to 0
    }else if (PIR1bits.ADIF == 1) {
        
        data = ADRESHbits.ADRES;
        LATCbits.LATC7 = ~(LATCbits.LATC7); //Borrar?
        if (!put_data(data)) {
            para = 1;
        }
       

        //CLEAR INTERRUPT FLAG
        PIR1bits.ADIF = 0;
        //ADC INTERRUPT ENABLE
        PIE1bits.ADIE = 1;
        //GLOBAL INTERRUPT ENABLE
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        //START CONVERSION
        ADCON0bits.GO = 1;
    }
}

int get_state_led(){
    int ret;
    
    disable_interrupts();
    ret = state_led;
    config_general_interrupts();
    config_external_interrupts();
    
    return ret;
}

void initialize_ports(){
    TRISAbits.TRISA1=1;
    ANSELAbits.ANSA1=0;
    //ODCONBbits.ODCB0 = 1;
    OSCCON1bits.NDIV = 0b0000;
    //initialize port A
    ANSELAbits.ANSA2 = 0;
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 0;
    //Initialize port C0
    ANSELCbits.ANSC0 = 0;
    TRISCbits.TRISC0 = 0;
}


void blue_led_blink(){
    LATCbits.LATC0 = 1;
    __delay_ms(200);
    LATCbits.LATC0 = 0;
    __delay_ms(200);
}

void red_led_blink(){
    LATAbits.LATA2 = 1;
    __delay_ms(200);
    LATAbits.LATA2 = 0;
    __delay_ms(200);
}

unsigned char get_id(unsigned char readen){
    unsigned char header;
    unsigned char id;
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char weekday;
    header=0;
    header|=readen>>28;
    header&=0x0F;
    switch(header){
        case 8: //pic configuration data
            year=0;
            year|=readen>>20;
            
            month=0;
            month|=readen>>16;
            month&=0x0F;
            
            day=0;
            day|=readen>>8;
            
            weekday=0;
            weekday|=readen>>4;
            weekday&=0x0F;
            
            /*day:
             * monday:1
             * tuesday:2
             * wednesday:3
             * thursday:4
             * friday:5
             * saturday:6
             * sunday:7*/
            break;
        case 3://??
            break;
        case 13://??
            break;
        case 6://id
            id=0;
            id|=readen>>20;
            break;
    }
    return id;
}

void char_to_string_id(unsigned char id,unsigned char* output_string){
    int aux;
    for(int i= 0;i<8;i++){
        aux = (id&(0b10000000>>i))>>(7-i);
        if(aux==1){
            output_string[7-i] = '1';
        }else{
            output_string[7-i] = '0';
        }
    }
    
}
//AUX
void toBCD(unsigned char data_separated[], unsigned char input){
    unsigned char data_converted=0x00;
    int shift=0;
    while(input>0){
        data_converted|=(input%10)<<(shift++<<2);
        input/=10;
    }
    data_separated[1]=(data_converted>>4) & 0x0F;
    data_separated[0]=data_converted & 0x0F;
}

//INSTRUCCIÓ 1
void config_PIC(unsigned char dataIN[]){
    unsigned char pre_id;
    unsigned char id_pic, year_pic, month_pic, day_pic, weekday_pic, hour_pic, min_pic, sec_pic;
    id_pic=dataIN[0];
    year_pic=dataIN[1];
    month_pic=dataIN[2]<<4;
    day_pic=dataIN[3];
    weekday_pic=dataIN[4];
    hour_pic=dataIN[5];
    min_pic=dataIN[6];
    sec_pic=dataIN[7];
    /*Pos que es
     * 1    ID
     * 2    YEAR
     * 3    MONTH
     * 4    DAY
     * 5    WEEKDAY
     * 6    HOUR
     * 7    MINUTES
     * 8    SECONDS
     */
    //write ID
    //pre_id=eeprom_read(idAddress);
    //if(pre_id==0xFF){}
    eeprom_write(idAddress, id_pic);
    
    unsigned char actual_d[];
    
    actual_d[0] = year_pic;
    actual_d[1] = month_pic;
    actual_d[2] = day_pic;
    actual_d[3] = weekday_pic;
    actual_d[4] = hour_pic;
    actual_d[5] = min_pic;
    actual_d[6] = sec_pic;
    
    config_date(actual_d);

} 

//INSTRUCCIÓ 2
void config_Alarm(unsigned char alrm_date[]){
/*date[]:
 * char1 => HOUR
 * char2 => MINUTE
 * char4 => WEEKDAY
*/ 
    unsigned char alrm_hour[], alrm_min[];
    unsigned char alrm_weekdayAddress;
    unsigned char config_hour, config_min, config_weekday;
    config_hour=alrm_date[0];
    config_min=alrm_date[1];
    config_weekday=alrm_date[2];
   //Alarm enabled
    ALRMCONbits.ALRMEN=1;
    //Alarm mask (once a day)
    ALRMCONbits.AMASK=0b0111;
    //Alarm repeat (0 more times)
    ALRMRPTbits.ARPT=0x00;
    ALRMWD = 0b100; //config_weekday;
    toBCD(alrm_hour,config_hour);
    ALRMHRbits.HRALRMH=alrm_hour[1];
    ALRMHRbits.HRALRML=alrm_hour[0];
    toBCD(alrm_min,config_min);
    ALRMMINbits.MINALRMH=alrm_min[1];
    ALRMMINbits.MINALRML=alrm_min[0];
}

void decode_audio(unsigned char data_received[]){
    unsigned char decisor;
    unsigned char config_pic_array[8];
    unsigned char config_alarm_array[3];
    decisor=data_received[0];
    switch(decisor){
        case 0x08:
            config_pic_array[0]=data_received[1];
            config_pic_array[1]=data_received[2];
            config_pic_array[2]=data_received[3];
            config_pic_array[3]=data_received[4];
            config_pic_array[4]=data_received[5];
            config_pic_array[5]=data_received[6];
            config_pic_array[6]=data_received[7];
            config_pic_array[7]=data_received[8];
            config_pic_array[8]=data_received[9];
            config_PIC(config_pic_array);
            break;
        case 0x03:
            config_alarm_array[0]=data_received[1];
            config_alarm_array[1]=data_received[2];
            config_alarm_array[2]=data_received[3];
            config_Alarm(config_alarm_array);
            break;
        /*case 0x06:
            send_all_data();
            break;*/
        default:
            //LATCbits.LATC0=1;
            break;
    }
}

void config_date(unsigned char actual_date[]){
    unsigned char year, month, day, weekday, hour, min, sec;
    unsigned char year_s[], month_s[], day_s[], hour_s[], min_s[], sec_s[];
    year=actual_date[0];
    month=actual_date[1];
    day=actual_date[2];
    weekday=actual_date[3];
    hour=actual_date[4];
    min=actual_date[5];
    sec=actual_date[6];
    //Config RTCC
    RTCCON = 0b00110001;
    __delay_us(50);
    //Date Configuration
    //Year
    toBCD(year_s,year);
    YEARbits.YEARH=year_s[1];
    YEARbits.YEARL=year_s[0];
    toBCD(month_s,month);
    MONTHbits.MONTHH=month_s[1];
    MONTHbits.MONTHL=month_s[0];
    WEEKDAYbits.WDAY=weekday;
    toBCD(day_s,day);
    DAYbits.DAYH=day_s[1];
    DAYbits.DAYL=day_s[0];
    toBCD(hour_s,hour);
    HOURSbits.HRH=hour_s[1];
    HOURSbits.HRL=hour_s[0];
    toBCD(min_s,min);
    MINUTESbits.MINH=min_s[1];
    MINUTESbits.MINL=min_s[0];
    toBCD(sec_s,sec);
    SECONDSbits.SECH=sec_s[1];
    SECONDSbits.SECL=sec_s[0];
    RTCCON=0b10010001;
    __delay_us(50);
}

void main(void) {
    flash_interrupt = 0;    
    led_para = 0;
    unsigned char* jaja;
    unsigned char output_string[8];
    unsigned char aux_output_string[4];
    unsigned char state_general;
    config_general_interrupts();
    config_external_interrupts();
    initialize_ports();
    blue_led_blink();
    unsigned char* readen;   
    state_led=0;
    packet_number = 0;
    unsigned char input_string[] = "18092243"; //DDhhTTHH
    
    //number_of_packets = strlen(input_string)/STREAM_LENGTH+1;
    state_general=0; //LISTEN
    while(1){
        switch(state_general){
            case 0: //LISTEN
                LATCbits.LATC0=1;
                LATAbits.LATA2=0;
                 readen  = audioread();
                 
                 if (flash_interrupt){
                     state_general=3;
                 }
                 if(readen[0]==8 || readen[0]==3  ){ //comanda
                     state_general=1;
                 }
                 
                break;
            case 1: //DECODE
                decode_audio(readen);

                blue_led_blink();
                
                for (unsigned int i = 0; i < strlen(readen); i ++){
                    readen[i] = 0;
                }
                state_general = 0;
                break;
            case 3: //SEND DATA
                // intentar implementar
                LATCbits.LATC0=0;
                 while(!led_para){
                    if(state_led==0){
                        packet_number = 0;
                        LATAbits.LATA2 = 0;
                    }else if(state_led == 1){
                        unsigned char id_value = eeprom_read(idAddress);
                        if(id_value==0xFF){
                            id_value = 0;
                        }
                        char_to_string_id(id_value, output_string); 
                        encode(output_string, 1); //Send ID
                        
                    }else if(state_led == 2){
                        //send_data(input_string);
                        encode(input_string, 0); //Send package
                    }
                }
                led_para = 0;
                state_general = 0;
                break;
        }              
    }
}