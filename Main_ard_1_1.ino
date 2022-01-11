unsigned long time0;       //StartMilis
unsigned long time1;       //CurrentMilis
unsigned long tup1;        //CurrentMilis

unsigned long period = 85000;//the value is a number of milliseconds
unsigned long period0 = 0;

unsigned long periodp = 1;   //Pocetno vreme za impulsnu f-ju ventila
unsigned long periodc = 3250;//Vreme potrebno za pomeranja ventila za 2.5%

unsigned long RegTime = 3600000;    //Vremenska konstanta za biranje nove spoljasnje tempr.
unsigned long RegStartTime = 1;

const int PuzS = A2;    //Koji pin se definise kao ulaz za senzor
const int ZahtevS = A1; //Definisan ulaz za gorionik
 
volatile float  PuzSVol, PuzSCur;
volatile float  ZahtevSVol,ZahtevSCur; 

//#############STANJA SISTEMA#############################
//Bitovi / bajtovi za oznacavanje stanja
bool s = false;                       //Start rezim
bool a = false;                       //Antifrost za zastitu od kond
bool bb = false;                      //Blokada pumpe aktivna
bool r = false;                       //Regulacija aktivna

//#############TEMPERATURNE SONDE#########################

float t1,t2,t3,t4,t5,t6,t7;         
//TKotPotis, TKotPovrat, TPotis, TPovrat, TKotla, TSpolja, TKuca ili TKotlarnica
uint8_t sensor1[8] ={0x28, 0x88, 0x0C, 0x03, 0x59, 0x20, 0x01, 0x96}; //tKotlarnica
uint8_t sensor2[8] ={0x28, 0x12, 0x80, 0x95, 0xF0, 0x01, 0x3C, 0xD4}; //tPotis - INST
uint8_t sensor3[8] ={0x28, 0xAA, 0xDC, 0xD6, 0x52, 0x14, 0x01, 0x47}; //tPovrat - KOT
uint8_t sensor4[8] ={0x28, 0xAA, 0xF2, 0xDF, 0x52, 0x14, 0x01, 0x8E}; //tKotla
uint8_t sensor5[8] ={0x28, 0xFC, 0x22, 0xC0, 0x58, 0x20, 0x01, 0x92}; //tSpolja
uint8_t sensor6[8] ={0x28, 0xAA, 0x71, 0xCF, 0x52, 0x14, 0x01, 0x33}; //tPotis -  KOT
uint8_t sensor7[8] ={0x28, 0x3D, 0x8B, 0x95, 0xF0, 0x01, 0x3C, 0x36}; //tPovrat - INS

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2    // Podesen PIN na kojem se nalazi davac temperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//###############STANJE PELETA#############################

float sanduk = 0;
float danas = 0;
const float g_s = 6.17;          //g_s = koliko grama peleta po sekundi,  
const float pstart = 49.36;      //pstart = kolicina peleta na startu gorionika || odgovara 8s punjenja

bool tt = 0; 
bool pp = 0;
unsigned long ton1, ton2, puls;
    
//#######################AKTUATORI#########################

const int bp7 = 7;                                //KONTAKT
const int bp8 = 8;                                  //PUMPA
const int bp9 = 9;              //SMER VENT, LOW >> ZATVARA
const int bp10 = 10;                                  //PUZ
const int bp11 = 11;                        //POGON VENTILA
const int Mtr = 12;                         //Punjenje dugme
const int Chrg = 13;                      //Dodavanje peleta

//#######################VENTIL#############################  

int pozicija = 0;   //Trenutna pozicija ventila
int zp = 0;
int x = 0;          //Oznacava broj inkremenata za koji se pomera
int b = 0;          //Oznacava smer u kojem se ventil treba pomeriti
bool z = false;     //Bajt za oznacavnje da je sistem zauzet

//############### REGULACIJA PROMENLJIVE ####################

unsigned long WindowSize = 120000;
unsigned long windowStartTime = 1;

//############### REGULACIJA / PRERACUN #####################

//y  daje karakteristiku ventila koja ide ka kotlovskom krugu
//ug daje ugao za preracunavanje preko pozicije koja se salje ka ard

float ug,ud,y,u;
float Tmix;
float Tzelj;
int dd,gg;
float delta1,delta2;

float Q_loss;

//######################## DISPLEJ ##########################

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//######################TEST VENTIL#########################

// const int a[4] = {40,20,80,50};
// volatile int k = 0;
// int zadatPoz = 0;

//##########################################################
void setup() {
    
  windowStartTime = millis();       //PidStarTime
  Serial.begin(9600);
  
//#####               Aktivacija DS18B20 Senzora              #####
// Podesava rezoluciju seonzra na 9-bit (vreme odyiva 187.5ms po jednom) ~~~ 0.25C rezolucija

  sensors.begin();
  sensors.setResolution(11);        //9 daje inkremente na 0.5 stepeni, mozda ce 10 davati na 0.25, treba povecati na 11 0.125

  pinMode(bp7, OUTPUT);   //KONTAKT GREJ
  digitalWrite(bp7,HIGH);
  
  pinMode(bp8, OUTPUT);   //PUMPA
  digitalWrite(bp8,HIGH);
  
  pinMode(bp9, OUTPUT);   //SMER VENT, KADA LOW >> ZATVARA
  digitalWrite(bp9,HIGH);
  
  pinMode(bp10, OUTPUT);  //PUZ
  digitalWrite(bp10,HIGH);
  
  pinMode(bp11, OUTPUT);  //POGON, POMERA VENTIL
  digitalWrite(bp11,HIGH);
  
  pinMode(PuzS, INPUT);   //ADC za puz
  pinMode(ZahtevS, INPUT);//ADC za gorionik
  
  pinMode(Mtr, INPUT);      //Punjenje puza
  pinMode(Chrg, INPUT);     //Dodavanje peleta

//################ ZA DISPLEJ ##########################    
//                                  NE RADI ZA SADA
//delay(2000);
//display.clearDisplay();
//display.setTextSize(1);
//display.setTextColor(WHITE);
//display.setCursor(0,0);
    
}

void loop() {
  time1 = millis();
  
  if(zp == pozicija){ //resetuje puls, aposlutno potrebno u regulaciji
    VentTarg(pozicija);
    PomerajVent(b,x);   //dolazilo do problema, treba da bude b,x: a ne 0,0 OVO JE SADA ODLICNO!!!!
  }
  
  StanjePeleta();
  SlikaIN();
  //Greska();
  //AntiFrost();
  //Start();
  Regulacija();
  Terminacija();
  Preracunavanje();
  Sender();             //nparavio izmene za sanduk i danas
  //Display();
  Debug();

  
}

void SlikaIN(){  
  StanjePeleta();
  sensors.requestTemperatures();                    //Filter, takav da samo 15% nove vrednosti utice na merenje pa ce izbaciti pikove
  t1 = 0.85*t1 + 0.15*sensors.getTempC(sensor1);    //TKotlarnice
  t2 = 0.85*t2 + 0.15*sensors.getTempC(sensor2);    //Tpotis - INSTALACIJA
  t3 = 0.85*t3 + 0.15*sensors.getTempC(sensor3);    //TPovrat - kotao
  t4 = 0.85*t4 + 0.15*sensors.getTempC(sensor4);    //TKotla
  t5 = 0.85*t5 + 0.15*sensors.getTempC(sensor5);    //Tspolja
  t6 = 0.85*t6 + 0.15*sensors.getTempC(sensor6);    //Tpotis - Kotao (SIGURNO)
  t7 = 0.85*t7 + 0.15*sensors.getTempC(sensor7);    //Tpovrat - INSTALACIJA
  
  PuzSVol = analogRead(PuzS); 
  PuzSCur = (PuzSVol*5.0 )/1024.0; // Skaliranje struje preko ADC na arduinu za PUZ 
  
  ZahtevSVol = analogRead(ZahtevS); 
  ZahtevSCur = (ZahtevSVol*5.0 )/1024.0; // Skaliranje struje preko ADC na arduinu za ZAHTEV
}

void StanjePeleta(){                //Treba da se testira, NE VALJA NIKAKO, kada sa debugerom radi salje neke cudne vrednosti
    if ((tt == 1) && (pp == 1)){
        if (ton1 < ton2){               //ovo ce mozda normalizovati neke vrednosti
            puls = ton2 - ton1;         //Ovo je intiger, tako da treba smisliti glavom ovom retardiranom sta da radim 
            tt = 0;
            pp = 0;
            sanduk = sanduk - puls*g_s;
            danas = danas + puls*g_s;
        
            Serial.println();
            Serial.print("PUZH:  ");
            Serial.print(puls);
            Serial.println();
            Serial.print(sanduk);
            Serial.println();
        }
    } 
    else{
      if (PuzSCur > 0.02){ 
        tt = 1;
        ton1 = millis();
      }
      else if(PuzSCur <= 0.0){ 
        pp = 1;
        ton2 = millis();
      } 
    }
}
void AntiFrost(){
  //Zastita kotla od kondenzacije
  //Kada temperatura povrata kotla padne ispod 63C >>>> pali se kotao i pumpa tj. ulazi u start
  StanjePeleta();
    if (t5 < 5.0){                     //Samo ako spoljna temperatura je ispod 16C, u suprotnom nek ide na termosta //stavio sam na 8, jer ispod 15 nema smisla
        if (t3 < 40.0){
            a = true;
            digitalWrite(bp7,LOW);      //Daj kontakt za grejanje
        }
        else if ((t3 >= 45.0) || (t6 >= 70.0)){           //Ako je povrat kotla iznad ove T, onda se gasi antifrost, ili oko 80 t potisa
            a = false;
            digitalWrite(bp7,HIGH);                         //Ugasi kontakt za grejanje
                                          //Da li ce pre podici povrat ili temperaturu kotla, ne znam pa treba izbaciti ukoliko se to desi
        }
    }
    else if((t5 >= 5.0) && (a = true)){ //ako se senzor zajebe, a desava se
      a = false;
      digitalWrite(bp7,HIGH);
    }
}
void Greska(){
  StanjePeleta();
  //Ako se zaglavio pelet ili puz ne izbacuje
    //Puz radi pulsira, ali temperatura kotla opada, kontakt aktivan
    //
  //Ako smo ostali bez peleta
    //sanduk << potrebnog za rad koliko bi trebalo (opcija 2 moze doci do greske)
    //Radi puz, radi gorionik, temperatura kotla opada
  //Nije se upalio pelet
    //Gorionik pod naponom, puz zavrsio punjenje, temperatura kotla ne raste
}    
void Start(){
  //if(Greska resetovana, ili AntiFrost ili Dat nalog za palje){
  //Za provere uslova temperature dovoljno je proveravati TKotla
  StanjePeleta();    
        if((t6 >= 60.0) && (a == false) && (r == false)){                       //t6 ~ 70, onda je realna T vode u kotlu 80c!!!    
            s = false;
            sanduk = sanduk - pstart;
            danas = danas + pstart;
            //digitalWrite(bp8,LOW);   // Upali pumpu
            //delay(100);
            zp = 750;
            VentTarg(zp);           //Otvori ventil za 25%
            delay(100);
            PomerajVent(b,x);
            delay(100);
            
        }
        else{
            if((ZahtevSCur > 0.10) || (a == true)){                                                      //ovde je problem!!!!!
                s = true;                               //oznacava start rezim
                //digitalWrite(bp8,HIGH);     //Ugasi pumpu    , razmotriti da li da se gasi zbog temperatura u potisu i povratu
                //delay(100);
                zp = 1000;
                VentTarg(zp);
                delay(100);
                PomerajVent(b,x);
                delay(100);
            }                                 //Da se ventil ne pomera dok je u startu
        }                
}
    //Zatvori mesni ventil, sve dok TKotPot >= 78, onda oslobodi i upali pumpu ukoliko je Tpot >35 ?? u suprotnom neka bude zatvorena
  //Pumpa ne mora da radi kada je noc, a kotao ne radi, ili kada je dugo proslo od greske a nije resetovana
  
  //Definisati rezim punjenja, kako bi se znalo kada je gorionik poceo sa radom
  //Ukoliko se taj rezim uvede, moguce je da se ne koristi interrupt za senzor peleta, sto verovatno i nece
  
  //Uvesti taster u ormanu koji ce, ukoliko dodje do greske zapisati sve sto je do tada uradjeno, i onda zapoceti ponovni ciklus punjenja
  //Sto je zapravo taster za resetovanje greske
  //Zapravo moze i bez tog tastera da se automatizuje reset i pocetak novog ciklusa


//##################TREBA RADITI NA OVOME####################################

void Regulacija(){                                                                  //Po Temperaturi
    StanjePeleta();
    if((time1-RegStartTime > RegTime) || (RegStartTime == 1)){                      //Zadaje novu vrednost polazne temperature na svakih sat vremena
        RegStartTime += RegTime;                                                    //pa treba onda to usloviti sa nekim temperaturnim histerezisom
        Tzelj = 4.1667e-4*pow(t5,3) + 0.0195*pow(t5,2) + -0.7917*t5 + 45.3148;      //Nova kriva iz Matlab-a
    }                                                               //stalno ulazi u ovu petlju, pogresan ili uslov
    
    if((t6 > Tzelj) && (s == false)){
        r = true;
    }
    else{
        r = false;
    }
    
    if((Tzelj + 1.5 > t2) && (Tzelj - 1.5 < t2)){             //Postaviti histerezis za potisnu temperaturu, postavljeno,u mojoj glavi kao radi
        windowStartTime = millis();                           
    }                                       // problem je ovde BIO
    else{                                                     //Ukoliko nije u histerezisu, pomeraj
        if((s == false) && (bb == false) && (r == true)){      //Ukoliko nije start u toku
            if (time1 - windowStartTime > WindowSize){     //Na svaka 2 minuta pomerati ventil???    --  ovo se radi nakon zavrsavanja start rezima, jer ce osc
                windowStartTime += WindowSize;
                //Regulacija_prava
                
                ud = (Tzelj-t7)/(t6-t7)*100;       //odredi udeo mesanja, NESTO NIJE OK ALI ULAZI U PETLJU
                u = -1.0676e-7*pow(ud,5) + 2.6485e-5*pow(ud,4) + -0.0025*pow(ud,3) + 0.107*pow(ud,2) + -2.6895*ud + 94.3144;// pozicija ventila na osnov udela
                
                dd = int(u/2.5);                 //mora da bude int
                gg = dd + 1;                //gornja vrednost za poredjenje
                
                delta1 = u - dd*2.5;        // videti celobrojni umnozak za pomeranje ventila
                delta2 = gg*2.5 - u;
                                            //nema smisla  da regulise ako se nista ne desava
                if(delta1 > delta2){        //ako je veca greska na jednom uzorku, onda treba zadati pomeranje na drugi
                    zp = 1000 - gg*25;      //nova fomrula zbog obrnutog brojanja
                    VentTarg(zp);
                    delay(100);
                    PomerajVent(b,x);
                    delay(100);
                }
                else{
                    zp = 1000 - dd*25;      //nova formula zbog obrnutog grafika             
                    VentTarg(zp);
                    delay(100);
                    PomerajVent(b,x);
                    delay(100);
                }
            }
        }
    }
    //Provera i za snimanje 
    ug = 100 - pozicija*0.1;                      //izmenjena formula jer sam ja menjao na grafiku i realno
    y = -8.6235e-8*pow(ug,5) + 2.1980e-5*pow(ug,4) + -0.0017*pow(ug,3) + 0.0267*pow(ug,2) + -0.3613*ug + 100.8061;
    Tmix = y*t6*0.01 + (100-y)*t7*0.01;   //y - udeo kotla * Tkotla + (1-y) udeo iz instalacije*povrat temperatura
                                          // uporediti t2 sa Tmix
}
void Terminacija(){                 //Hajde implementrijA OVOV AISLJAKSDJLKAj KADA SE PALI PUMPTA
    StanjePeleta();
        if((t2 <= 38.0) && (s == true)){     //Kada priv put startne
            bb = true;
            digitalWrite(bp8,HIGH);
        }
        else if((a == true) && (t2 <= 38.0)){   //kada je antifrost, a t2 je ispod 38
            bb = true;
            digitalWrite(bp8,HIGH);    
        }
        else if(s == false || r == true){
            bb = false;
            digitalWrite(bp8,LOW);  //Ostavi upaljenu pumpu    
        }
        //DOVDE SAM STIGAO SINOC< SUTRA RAZMISLJA O OVOME
        
        if(t2 <= 38.0 || (r == false) || (s == true)){              //Temperatura mesanja, trebalo bi temperatura izvora
            bb = true;
            //s true i t2 sigurno manje kada priv put starta
            //ako je a true, onda proveriti da li je t2 u razini
            digitalWrite(bp8,HIGH); //Ugasi pumpu
        }
        else {
            bb = false;
            digitalWrite(bp8,LOW);  //Ostavi upaljenu pumpu
        }
}
void Preracunavanje(){
    StanjePeleta();
    
    
    if(digitalRead(Chrg)){         //Kad se stisne dugme onda povecava danasnje stanje sanduka za 15kg
        sanduk = sanduk + 15.0;
    }
    
     if(analogRead(A3) > 0.5){         //kada je 06:00, resetuje se dnevna potrosnja, pogledai samo da li treba veca vrednost od 0.5
         danas = 0;     
     }
    
    if(digitalRead(Mtr)){              //Rucno punjenje puza
        digitalWrite(bp10,LOW); 
    }
    else{
        digitalWrite(bp10,HIGH);
    }
    
    Q_loss = (t1 - t5)/1.255;          //Trenutni toplotni gubici, za debiljnu zida 30cm, to je najtanji zid u kuci
    
//  Predikcija za sutrasnji dan, u odnosu na danasnju potrosnju>>>
//  Predikcija za tekuci dan, koliko bi trebalo da se potrosi peleta >>>
//  Kada bi trebalo napuniti >>>.
//  
  //Ocitaj stanja senzora i upisi u SD CARD
  //Izracunaj izvedene vrednosti i upisi // izbaci na display???
}

void Sender(){
  StanjePeleta();
  
  Serial.print('q');
  Serial.print(t1);
  Serial.print('a');
  
  Serial.print('w');
  Serial.print(t2);
  Serial.print('a');

  Serial.print('e');
  Serial.print(t3);
  Serial.print('a');

  Serial.print('r');
  Serial.print(t4);
  Serial.print('a');

  Serial.print('t');
  Serial.print(t5);
  Serial.print('a');

  Serial.print('y');
  Serial.print(t6);
  Serial.print('a');

  Serial.print('u');
  Serial.print(t7);
  Serial.print('a');

  //Serial.print('i');
  //Serial.print(sanduk);
  //Serial.print('a');

  //Serial.print('o');
  //Serial.print(danas);
  //Serial.print('a');
  
  Serial.print('p');
  Serial.print(Q_loss);
  Serial.print('a');
}
void Display(){ 
    StanjePeleta();
    
    // if(time1 > Windowsitalj > windowsize){
    //     windwosilj += windowsize;
    // }
    
    //ekran 1
    
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
    
        display.print("TPot_K: ");
        display.setCursor(7,0);
        display.print(t6);
        
        display.print(" ");
        display.cp437(true);
        display.write(167);
        display.print("C");
        
        display.print("TPot_I: ");
        display.setCursor(7,9);
        display.print(t2);
        
        display.print(" ");
        display.cp437(true);
        display.write(167);
        display.print("C");
        
        display.print("TPvr_K: ");
        display.setCursor(7,19);
        display.print(t3);
        
        display.print(" ");
        display.cp437(true);
        display.write(167);
        display.print("C");
        
        display.print("TPvr_I: ");
        display.setCursor(7,29);
        display.print(t7);
        
        display.print(" ");
        display.cp437(true);
        display.write(167);
        display.print("C");
    
        
    
    //ekran 2
    
    //ekran 3
    
        display.display();    
}
void Debug(){
  StanjePeleta();
  
  Serial.println();
  //Pisati samo ispod ovog
  
  Serial.println("POZICIJA");
  Serial.print("Zadato: ");
  Serial.print(zp);
  Serial.print(" ... ");
  Serial.print("Realno: ");
  Serial.println(pozicija);
  
  Serial.println("REGULACIJA");
  Serial.print("Zadato: ");
  Serial.print(Tzelj);
  Serial.print(" ... ");
  Serial.print("Zad/fja(mat): ");
  Serial.print(Tmix);
  Serial.print(" ... ");
  Serial.print("Realno: ");
  Serial.println(t2);

  Serial.println("REGULACIJA U/R");
  Serial.print("Window: ");
  Serial.print(windowStartTime);
  Serial.print(" ... ");
  Serial.print("Zad_udeo(mat): ");
  Serial.println(ud);
  Serial.print("delta1: ");
  Serial.print(delta1);
  Serial.print(" ... ");
  Serial.print("delta2: ");
  Serial.println(delta2);
  Serial.print("dd: ");
  Serial.print(dd);
  Serial.print(" ... ");
  Serial.print("gg: ");
  Serial.println(gg);

  Serial.println("STANJE");
  Serial.print("s: ");
  Serial.print(s);
  Serial.print(" ... ");
  Serial.print("bb: ");
  Serial.print(bb);
  Serial.print(" ... ");
  Serial.print("r: ");
  Serial.println(r);
  
  //Pisati samo iznad ovog
  Serial.println();
      
}
//################################# FUNKCIJE POMERANJA VENTILA ---- GOTOVO 100% ###########################

void VentTarg(int zadatPoz){    //Pretvara zadatu poziciju u kretanje
        if(z == false){   //z == true, znaci da se trenutno ventil pomera u odredjenu poziciju
            if(zadatPoz == pozicija){
              x = 0;
              b = 0;
            }
        
            if(zadatPoz > pozicija){
              x = (zadatPoz-pozicija)/25;    // /5 jer trenutna perioda odgovara inkrementima od 5%
              b = 1;
              z = true;
            }
            if(zadatPoz < pozicija){
              x = (pozicija-zadatPoz)/25;
              b = -1;
              z = true;
            }
        }
    }
void PomerajVent(int b, int x){         //Obradjuje informacije za pomeranje
        if(b == 1){
            digitalWrite(bp9,LOW);     //Zatvori ventil
            delay(100);
            digitalWrite(bp11,LOW);     //Pogon ventila
            PulsPog();
        }    
        if(b == -1){
            digitalWrite(bp9,HIGH);    //Otvori ventil
            delay(100);
            digitalWrite(bp11,LOW);     //Pogon ventila
            PulsPog();
        }
        if(b == 0){
            digitalWrite(bp11,HIGH);    //Ugasi pogon ventila
            if(time1 > periodp){        //Ovaj deo nije ispitan
                periodp = millis();
            }
        }
}   
void PulsPog(){             //Gasi pomeranje nakon odr. vremena kako bi odredio poziciju
    if (time1 - periodp > periodc*x){
        periodp += periodc*x;
        pozicija += 25*x*b;
        b = 0;
        z = false;
    }
    if(pozicija >= 1000){
        pozicija = 1000;
    }
    else if(pozicija <= 0){
        pozicija = 0;
    }
}

//#######################KOMENTARI###########################  

//Definisati kada ugasiti pumpu, posto treba nocu da se gasi, i smisliti na koji nacin da prima podatke o vremenu od ESP8266
//Potrebno je relej za kontatk za paljenje staviti na izvor od 3.3V
//Delay od 100 nakon svakog pozivanja releja jer se ugasi arduino

//Trebalo bi spustiti snagu gorionika, tako da gubici na 0 stepeni budu prva brzina gorionika, mozda blago jaca
//Svakako ako to ne uspe da nadomesti gubitke u datom trenutku, upalice se kontakt zbgo antifrosta, i samim tim ce dici Temp vecom brzinom 

//Imam problem sa logovanjem koji treba resiti
