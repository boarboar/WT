
/********************
TODO - screen based UI
Test I2C & SPI in parallel
*********************/

#include <NRF24.h>
#include <SPI.h>
#include <LiquidCrystal_SR_LCD3.h>

// *************** HARWDWARE
// *************** LCD

const int PIN_LCD_STROBE         =  P2_3;  // Out: LCD IC4094 shift-register strobe
const int PIN_LCD_DATA           =  P2_5;  // Out: LCD IC4094 shift-register data
const int PIN_LCD_CLOCK          =  P2_4;  // Out: LCD IC4094 shift-register clock
const int PIN_LCD_BL              =  P2_1;

// srdata / srclock / strobe
LiquidCrystal_SR_LCD3 lcd(PIN_LCD_DATA, PIN_LCD_CLOCK, PIN_LCD_STROBE);

// *************** WIRILESS
// Singleton instance of the radio
// CE=P1_4
// SS=CS (P2_0)
NRF24 nrf24(P1_4, SS);

// *************** SW
// *************** WL/TEMP

struct wt_msg {
  uint8_t msgt;
  uint8_t sid;
  int16_t temp;
  int16_t vcc;
}; 

#define WS_CHAN  1
#define WS_MSG_TEMP  0xFE

const char *addr="wserv";

wt_msg msg = {0xFF, 0xFF, 0xFFFF, 0xFFFF};
uint8_t err=0;
volatile uint8_t rf_read=0;

unsigned long rf_millis=0;

uint8_t alarms=0;
uint16_t alarm_val=0;
uint8_t alarm_cnt[5]; 
uint16_t msgcnt=0;

#define WS_ALR_WFAIL  0x1
#define WS_ALR_BADMSG 0x2
#define WS_ALR_BAD_TEMP 0x4
#define WS_ALR_TO     0x8
#define WS_ALR_LOWVCC 0x16

#define WS_ALR_WFAIL_IDX  0
#define WS_ALR_BADMSG_IDX 1
#define WS_ALR_BAD_TEMP_IDX 2
#define WS_ALR_TO_IDX     3
#define WS_ALR_LOWVCC_IDX 4
#define WS_ALR_LAST_IDX 4

#define WS_SENS_TIMEOUT (10L*60*1000)

#define DS18_MEAS_FAIL	9999  // Temporarily!

// ************************ HIST

#define WS_HIST_SZ  32// 32*15=8hrs
//#define WS_HIST_SZ  36// 36*15=9hrs
#define HIST_NODATA 0x0FFF
#define WS_ACC_TIME  15 //mins

int16_t last_tmp=HIST_NODATA;
int16_t last_vcc=0;
unsigned long last_millis_temp=0;

struct wt_msg_hist {
  uint8_t sid;  // src
  uint8_t mins; // mins since previous put
  int16_t temp;
}; 

struct wt_msg_acc {
  uint8_t cnt;
  uint8_t mins; // mins since previous put
  int32_t temp;
};

wt_msg_hist hist[WS_HIST_SZ];
uint8_t hist_ptr=0; // NEXT record to fill

wt_msg_acc acc={0, 0, 0};
unsigned long acc_prev_time=0;

// ************************ UI

#define WS_NUILEV 10
#define WS_UI_MAIN_NOBL  0
#define WS_UI_MAIN_BL  1
#define WS_UI_HIST  2
#define WS_UI_CHART 3
#define WS_UI_CHART15 4
#define WS_UI_CHART30 5
#define WS_UI_CHART60 6
#define WS_UI_AVG   7
//#define WS_UI_AVG_2 5
//#define WS_UI_ACC   6
#define WS_UI_DEBUG 8
#define WS_UI_STAT 9

#define WS_LCD_CHAR_UP 0xD9
#define WS_LCD_CHAR_DN 0xDA
#define WS_CHART_NCOL 11

union tmp_union {  
char strbuf[16]; 
struct {int16_t t; uint8_t c;} cval[WS_CHART_NCOL];
} tmp;

//struct {int16_t t; uint8_t c;} cval[WS_CHART_NCOL];

//unsigned long mstart;
unsigned long mbloff;
unsigned long mdisp;
unsigned long mui;
uint8_t ledon=0;
uint8_t bst=0;
uint8_t uilev=0;
uint8_t bllev;  // Backlit level

const int BUTTON_1 = PUSH2;     // the number of the pushbutton pin
const int WS_BL_LEVEL=140;
const int WS_BL_TO=10000;


void setup() 
{  
  digitalWrite(RED_LED, LOW);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_LCD_BL, OUTPUT);
  delay(100); 
  
  lcd.begin(16, 2);               // initialize the lcd 
  lcd.home ();                   // go home  
  initBars();
  initHist();
  Bl_On();

  err = radioSetup();
  
  if(!err) {
    lcd.print("INIT OK");  
    digitalWrite(RED_LED, HIGH); delay(1000); digitalWrite(RED_LED, LOW);
  }
  else {
    lcd.print("ERR="); lcd.setCursor(5, 1); lcd.print(err);   
    for(int i=0; i<err; i++) {
      digitalWrite(RED_LED, HIGH); delay(500); digitalWrite(RED_LED, LOW);
    }
  }
  delay(2000);  
  lcd.clear();
  lcd.print("WAIT...");
  acc_prev_time=mdisp=mui=millis();
  attachInterrupt(BUTTON_1, btn1Pressed, FALLING);
  attachInterrupt(P2_2, radioIRQ, FALLING); 
}


void loop()
{ 
 if(rf_read) {
  rf_read=0;  
  err=radioRead();  
  digitalWrite(RED_LED, HIGH);
  ledon=1;
  updateScreenData(); 
 }
 
 unsigned long ms=millis(); 
 if(ms-mui>100 || ms<mui) { // UI cycle
   mui=ms;
   if(ledon && ms-rf_millis>100) { //blink
     digitalWrite(RED_LED, LOW); 
     ledon=false;  
   }
   if(bllev>0 && mdisp>mbloff) {  // dimmer
     Bl_Dim();
     if(uilev) {
       uilev=0;
       redrawScreen();
     }
   }
   if(bst==1) { // btn pressed
     if(digitalRead(BUTTON_1)==LOW) { // still pressed, debounce ok
       Bl_On();
       bst=2;
       uilev=(uilev+1)%WS_NUILEV;
       if(uilev==WS_UI_MAIN_NOBL) uilev=WS_UI_MAIN_BL;
       redrawScreen();      
     }
   }

   if(ms-mdisp > 1000 || mdisp>ms) { // 1 sec screen update
     mdisp=ms;     
     if(!(alarms&WS_ALR_TO) && millis()-rf_millis>WS_SENS_TIMEOUT) {
       alarms |= WS_ALR_TO;
       alarm_val=(millis()-rf_millis)/60000;
       alarm_cnt[WS_ALR_TO_IDX]++;
       updateScreenData();
     }
     else updateScreenTime();  
   }  
 } // UI cycle 
}

/****************** UI ****************/

void redrawScreen() {
   updateScreenData();
   lcd.setCursor(15, 0); lcd.print(uilev); // show UI level
}

void updateScreenData() {  
  lcd.clear();    
  switch(uilev) {
    case WS_UI_MAIN_NOBL:
    case WS_UI_MAIN_BL:  
      //if(!err) { // 2013.12.27
        if(msg.msgt==0xFF) {
          lcd.setCursor(0, 1); lcd.print("NO DATA");
        }
        else {
          uint16_t ent, frc;
          int16_t diff = getHistDiff(1);
          lcd.setCursor(9, 0); lcd.print("S=");
          lcd.setCursor(11, 0); lcd.print(msg.sid, HEX);    
          lcd.setCursor(13, 0); lcd.print(alarms ? '!' : ' ');    
          lcd.setCursor(0, 1); lcd.print("t="); printTemp(last_tmp);
          lcd.setCursor(8, 1); lcd.write(diff>0?WS_LCD_CHAR_UP:(diff<0?WS_LCD_CHAR_DN:0x20)); 
          ent=last_vcc/100; frc=last_vcc%100; sprintf(tmp.strbuf, "v=%d.%02.2d", ent, frc);          
          lcd.setCursor(10, 1); lcd.print(tmp.strbuf);
        }
      /* } // 2013.12.27        
      else {
       lcd.setCursor(0, 1);
       lcd.print("ERR="); lcd.setCursor(5, 1); lcd.print(err);     
      }*/
    break;
    case WS_UI_HIST: 
      printHist(1);
    break; 
    case WS_UI_CHART: 
      chartHist(1);
    break;     
    case WS_UI_CHART15: 
      chartHist1(1, 15);
    break;         
    case WS_UI_CHART30: 
      chartHist1(1, 30);
    break;
    case WS_UI_CHART60: 
      chartHist1(1, 60);
    break;                 
    case WS_UI_AVG: 
      printAvg(1);
    break; 
    case WS_UI_DEBUG: {
      lcd.setCursor(0, 0); lcd.print("DBG, AL="); lcd.print(alarms, BIN);
      if(alarms) { lcd.print(" AV="); lcd.print(alarm_val); }
      lcd.setCursor(0, 1); for(int i=0; i<=WS_ALR_LAST_IDX; i++) {lcd.print(alarm_cnt[i]); lcd.print(" ");}
    }
    break;
    case WS_UI_STAT: {
      lcd.setCursor(0, 0); lcd.print("UT="); dispTime(millis(), true); 
      lcd.setCursor(0, 1); lcd.print("C="); lcd.print(msgcnt); 
      lcd.print(" H="); lcd.print(getHistSz()); lcd.print(" HB="); lcd.print(sizeof(hist)); 
    }
    break;             
    default:
      lcd.setCursor(0, 1); lcd.print("TBD");
    break;
  }
  updateScreenTime();
  
}

void updateScreenTime() {  
  switch(uilev) {
    case WS_UI_MAIN_NOBL:
    case WS_UI_MAIN_BL:  
      lcd.setCursor(0, 0); 
      //dispTime(millis()-rf_millis, false);
      dispTime(millis()-last_millis_temp, false);
    break;
    default:
    break;
  }
}

void printTemp(int16_t disptemp) {
  // ----------
  if(disptemp<-999 || disptemp>999) { // change to smth better...
    lcd.print("BADTMP");
    return;
  }
  char s='+';
  if(disptemp<0) {
    s='-';
    disptemp=-disptemp;
  } else if(disptemp==0) s=' ';
  sprintf(tmp.strbuf, "%c%d.%d", s, disptemp/10, disptemp%10);
  lcd.print(tmp.strbuf);
} 

void dispTime(unsigned long dt, bool flong) {
  unsigned long ts=dt/1000;
  uint8_t tss=ts%60, tsm=(ts/60)%60, tsh=(ts/3600)%24, days=ts/3600/24;  
  if(days>0) { 
    if(!flong) lcd.print("> "); 
    lcd.print(days); lcd.print("d,"); 
  }
  if(days==0 || flong) {
    sprintf(tmp.strbuf, "%02d:%02d:%02d", tsh, tsm, tss);    
    lcd.print(tmp.strbuf);
  }
}

/****************** HIST ****************/

void initHist() {
 for(uint8_t i=0;i<WS_HIST_SZ;i++) hist[i].temp=HIST_NODATA;
}

uint8_t addHistAcc(struct wt_msg *pmsg) {
  uint8_t mins;
  last_vcc=pmsg->vcc;
  if(DS18_MEAS_FAIL==pmsg->temp) {
    alarms |= WS_ALR_BAD_TEMP; alarm_val=err; alarm_cnt[WS_ALR_BAD_TEMP_IDX]++;
    return 1;
  }
  msgcnt++;
  last_tmp=pmsg->temp; 
  acc.cnt++;
  acc.temp+=pmsg->temp;
  mins=(millis()-acc_prev_time)/60000L; // TEMPORARILY (NEED TO HANDLE WRAPAROUND)!!!!
  last_millis_temp=millis();
  if(mins>=WS_ACC_TIME) { 
    addHist(1, mins, acc.temp/acc.cnt);
    acc_prev_time=millis();
    acc.temp=0;
    acc.cnt=0;
  }
  return 0;
}

void addHist(uint8_t sid, uint8_t mins, int16_t temp) {
  //hist[hist_ptr].sid=sid;
  hist[hist_ptr].temp=temp;
  hist[hist_ptr].mins=mins;
  hist_ptr = (hist_ptr+1)%WS_HIST_SZ; // next (and the oldest reading)
}

int16_t getHistDiff(uint8_t sid) {
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) return 0;
  return last_tmp-hist[lst].temp;
}

uint8_t getHistSz() {
  uint8_t cnt=0;
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) {
    return 0;
  }
  uint8_t prev=lst;
  do {
    cnt++;    
    prev=prev==0?WS_HIST_SZ-1 : prev-1;   
  } while(prev!=lst && hist[prev].temp!=HIST_NODATA);
  return cnt;
}

void printHist(uint8_t sid) {
  //uint8_t cnt=0;
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) {
    lcd.print("NO DATA");
    return;
  }
  uint8_t prev=lst;
  uint8_t mbefore=(millis()-acc_prev_time)/60000L; // TEMPORARILY (NEED TO HANDLE WRAPAROUND)!!!!;
  uint8_t hpos=0;
  do {
    lcd.setCursor(hpos, 0); printTemp(hist[prev].temp);
    lcd.setCursor(hpos, 1); lcd.print(mbefore);
    mbefore+=hist[prev].mins;
    hpos+=6;
    prev=prev==0?WS_HIST_SZ-1 : prev-1;   
  } while(prev!=lst && hist[prev].temp!=HIST_NODATA && hpos<14);
}

int16_t getHistAvg(uint8_t sid, uint16_t from, uint16_t period) { // avearge in from-period .... from
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) return HIST_NODATA;
  uint16_t mbefore=0;
  uint8_t start=lst;
  if(from>0) { 
    do {
      mbefore+=hist[start].mins;
      start=start==0?WS_HIST_SZ-1 : start-1; 
    } while(start!=lst && hist[start].temp!=HIST_NODATA && mbefore<from);  
    if(start==lst || hist[start].temp==HIST_NODATA) return HIST_NODATA;
  }
  
  mbefore=0;
  from += period;
  uint8_t cnt=0;
  int32_t acc=0;
  do {
    mbefore+=hist[start].mins;
    acc+=hist[start].temp;
    cnt++;
    start=start==0?WS_HIST_SZ-1 : start-1; 
  } while(start!=lst && hist[start].temp!=HIST_NODATA && mbefore<from);

  return acc/cnt;
}

void printAvg(uint8_t sid) {
  int16_t avg;
  lcd.setCursor(0, 0); lcd.print("30m:");
  for(int i=0; i<3; i++) {
    avg=getHistAvg(sid, i*30, (i+1)*30-1); 
    if(avg == HIST_NODATA) break;
    printTemp(avg);
  }
  lcd.setCursor(0, 1); lcd.print("1 h:");
  for(int i=0; i<3; i++) {
    avg=getHistAvg(sid, i*60, (i+1)*60-1);
    if(avg == HIST_NODATA) break;
    printTemp(avg);
  }  
}

void chartHist(uint8_t sid) {
  int16_t mint, maxt;
  uint8_t cnt=WS_CHART_NCOL;

  lcd.setCursor(15, 1);
  lcd.print("C");
  
  lcd.setCursor(0, 0); 
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) {
    lcd.print("NO DATA");
    return;
  }
  maxt=hist[lst].temp;
  mint=hist[lst].temp;
   
  uint8_t prev=lst;  
  do {
    if(hist[prev].temp>maxt) maxt=hist[prev].temp;
    if(hist[prev].temp<mint) mint=hist[prev].temp;
    prev=prev==0?WS_HIST_SZ-1 : prev-1; 
    cnt--;
  } while(prev!=lst && hist[prev].temp!=HIST_NODATA && cnt>0);
  
  lcd.setCursor(WS_CHART_NCOL, 0); printTemp(maxt);
  lcd.setCursor(WS_CHART_NCOL, 1); printTemp(mint);
  
  const uint8_t gridY = 7; 
  int16_t tdiff=maxt-mint;
  
  if(tdiff<gridY*4) {
    int16_t adg=(gridY*5-tdiff)/2;
    maxt+=adg;
    mint-=adg;
    tdiff=maxt-mint;
  }
  
  uint8_t yg;
  cnt=WS_CHART_NCOL;  
  prev=lst; 
  do {
    if(tdiff==0) yg=4;
    else yg=(uint8_t)((hist[prev].temp-mint)*gridY/tdiff);    
    prev=prev==0?WS_HIST_SZ-1 : prev-1; 
    cnt--;
    lcd.setCursor(cnt, 1-yg/4);
    lcd.write((uint8_t)(3-yg%4));
  } while(prev!=lst && hist[prev].temp!=HIST_NODATA && cnt>0);   

}

void chartHist1(uint8_t sid, uint8_t s) {
  int16_t mint, maxt; 
 
  lcd.setCursor(15, 1);
  if(s==15 || s==30) lcd.write(s==15 ? 0xF0 : 0xF2);
  else lcd.print(s==60 ? '1' : '-'); 
  
  lcd.setCursor(0, 0); 
  uint8_t lst=hist_ptr==0?WS_HIST_SZ-1 : hist_ptr-1;   
  if(hist[lst].temp==HIST_NODATA) {
    lcd.print("NO DATA");
    return;
  }
  
  uint8_t mbefore=(millis()-acc_prev_time)/60000L; // TEMPORARILY (NEED TO HANDLE WRAPAROUND)!!!!;

  maxt=hist[lst].temp;
  mint=hist[lst].temp;
  bzero(tmp.cval, sizeof(tmp.cval));
 
  uint8_t prev=lst;  
  uint8_t icol;
  int16_t alldur=WS_CHART_NCOL*s;
  do {
    icol=mbefore/s;
    if(icol>=0 && icol<WS_CHART_NCOL) {
      tmp.cval[icol].t+=hist[prev].temp; tmp.cval[icol].c++;
      if(hist[prev].temp>maxt) maxt=hist[prev].temp;
      if(hist[prev].temp<mint) mint=hist[prev].temp;
    }
    mbefore+=hist[prev].mins;
    prev=prev==0?WS_HIST_SZ-1 : prev-1; 
  } while(prev!=lst && hist[prev].temp!=HIST_NODATA && mbefore<=alldur);  
  
  const uint8_t gridY = 7; 
  int16_t tdiff=maxt-mint;
  int16_t low=mint;
  
  if(tdiff<gridY*4) {
    int16_t adg=(gridY*5-tdiff)/2;
    low=mint-adg;
    tdiff=maxt-mint+2*adg;
  }
  
  uint8_t yg;
  for(icol=0; icol<WS_CHART_NCOL; icol++) {
    if(!tmp.cval[icol].c && icol-1>=0 && icol+1<WS_CHART_NCOL && tmp.cval[icol-1].c && tmp.cval[icol+1].c) { 
      //interpolate
      tmp.cval[icol].c=tmp.cval[icol-1].c+tmp.cval[icol+1].c;
      tmp.cval[icol].t=tmp.cval[icol-1].t+tmp.cval[icol+1].t;
    }
    
    if(tmp.cval[icol].c) {
      if(tdiff==0) yg=4;
      else yg=(uint8_t)((tmp.cval[icol].t/tmp.cval[icol].c-low)*gridY/tdiff);    
      lcd.setCursor(WS_CHART_NCOL-icol-1, 1-yg/4);
      lcd.write((uint8_t)(3-yg%4));
    }
  }
  
  lcd.setCursor(WS_CHART_NCOL, 0); printTemp(maxt);  
  lcd.setCursor(WS_CHART_NCOL, 1); printTemp(mint);
   
}

/****************** UI LOWLEVEL ****************/

void initBars() {
 uint8_t i;
 for(i=0; i<4; i++) createBar(i, 0); 
 //for(i=0; i<4; i++) createBar(i+4, 1); 
}

void createBar(uint8_t idx, uint8_t dash) {
  byte c[8];
  bzero(c, 8);
  c[idx*2]=c[idx*2+1]=0b11111; // bar
  if(dash) {
    for(uint8_t i=0; i<8; i++) { // V-dash
      if(i%2==0) c[i] |= 0b00001; 
      else c[i] &= 0b11110; 
    }
  }
  lcd.createChar(idx, c);
}

void Bl_On() {
 bllev=WS_BL_LEVEL;
 analogWrite(PIN_LCD_BL, bllev); 
 mbloff=millis()+WS_BL_TO;
}

void Bl_Dim() {
 if(bllev>=2) {
   bllev-=2;
   analogWrite(PIN_LCD_BL, bllev); 
 }
}

void btn1Pressed() {
 attachInterrupt(BUTTON_1, btn1Released, RISING);
 bst=1;
}

void btn1Released() {
 attachInterrupt(BUTTON_1, btn1Pressed, FALLING);
 bst=0;
}

/****************** RADIO ****************/

uint8_t radioSetup() {
  if (!nrf24.init())
    err=1;
  else if (!nrf24.setChannel(WS_CHAN))
    err=2;
  else if (!nrf24.setThisAddress((uint8_t*)addr, strlen(addr)))
    err=3;
  else if (!nrf24.setPayloadSize(sizeof(wt_msg)))
    err=4;
  //else if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPower0dBm))
  else if (!nrf24.setRF(NRF24::NRF24DataRate250kbps, NRF24::NRF24TransmitPower0dBm)) 
    err=5;      
  nrf24.spiWriteRegister(NRF24_REG_00_CONFIG, nrf24.spiReadRegister(NRF24_REG_00_CONFIG)|NRF24_MASK_TX_DS|NRF24_MASK_MAX_RT); // only DR interrupt
  
  if(err) {alarms |= WS_ALR_WFAIL; alarm_val=err; alarm_cnt[WS_ALR_WFAIL_IDX]++;}
  
  return err;
}

uint8_t radioRead() {
 rf_millis=millis();
 err = 0;
 uint8_t len=sizeof(msg); 
 if(!nrf24.available()) { err=6; alarms |= WS_ALR_WFAIL; alarm_cnt[WS_ALR_WFAIL_IDX]++;}
 else if(!nrf24.recv((uint8_t*)&msg, &len)) { err=7; alarms |= WS_ALR_WFAIL; alarm_cnt[WS_ALR_WFAIL_IDX]++;}
 else if(WS_MSG_TEMP != msg.msgt) { err=8; alarms |= WS_ALR_BADMSG; alarm_cnt[WS_ALR_BADMSG_IDX]++;}
 else {    
   if(0==addHistAcc(&msg)) alarms = 0; // at the moment
 }
 return err;
}

void radioIRQ() {
  /*
  // 0x42
  rf_status=nrf24.statusRead();  
  */
  rf_read=1;
}
