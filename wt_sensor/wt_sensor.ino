
/************
TODO - 

INPUT_PULLUP & LOW all test & unused pins

************/

 /*
NRF24TransmitPowerm18dBm 	-18 dBm
NRF24TransmitPowerm12dBm 	-12 dBm
NRF24TransmitPowerm6dBm 	-6 dBm
NRF24TransmitPower0dBm 	0 dBm 

NRF24DataRate1Mbps 	1 Mbps
NRF24DataRate2Mbps 	2 Mbps
NRF24DataRate250kbps 	250 kbps 
*/
#include "Energia.h"

#include "DS18B20.h"
#include "NRF24.h"
#include <SPI.h>

struct wt_msg {
  uint8_t msgt;
  uint8_t sid;
  int16_t temp;
  int16_t vcc;
}; 

#define WS_DELAY 1000

// pins
#define WS_OWPIN    P1_3 
#define WS_ID1      P2_4
#define WS_ID2      P2_5
#define WS_TEST     P2_2
#define WS_CE       P1_4 
#define WS_SS       P2_0
#define WS_MISO     P1_7 // 2552
#define WS_MOSI     P1_6
#define WS_SCK     P1_5

/* 
unused
P1_1, 1_2, 2_1, 2_3
test & init
2_4, 2_5, 2_2
*/

#define WS_CHAN  1
#define WS_MSG_TEMP  0xFE
#define WS_IS_NOACK false

#define WS_DELAY_TEST  5
//#define WS_DELAY_NORM  300 // 5 min
#define WS_DELAY_NORM  900 // 15 min
#define VCC_CNT_N      4 // every hour; 


// Singleton instance of the radio
// CE=P1_4
// SS=CS (P2_0)
NRF24 nrf24(P1_4, SS);

// Singleton instance of the sensor
DS18B20 term(WS_OWPIN);

const char *addr="wsen1";
const char *dst_addr="wserv";

static wt_msg msg = {WS_MSG_TEMP, 1, 0, 0};

static uint8_t err=0;
static uint8_t testmode=0;
static uint16_t sdelay=0;
static int16_t vcc=0;
static uint8_t vcc_cnt=0;

#define RLS_PIN(P) {digitalWrite(P, LOW); pinMode(P, INPUT_PULLUP); }

/* ID matrix

P2_4\P2_5  0  1
0          4  2
1          3  1
=1+(!P2_5)+2*(!P2_4)

*/

void setup() 
{  
  delay(100);
  digitalWrite(RED_LED, LOW);
  pinMode(RED_LED, OUTPUT);
  
  
  pinMode(WS_ID1, INPUT_PULLUP);
  pinMode(WS_ID2, INPUT_PULLUP);
  pinMode(WS_TEST, INPUT_PULLUP);
    
  //analogReference(INTERNAL2V5);
  //analogRead(11);
  
  for(uint8_t i=0; i<5; i++) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
    }
    
  testmode=digitalRead(WS_TEST)==LOW;
  sdelay=testmode ? WS_DELAY_TEST : WS_DELAY_NORM;
  msg.sid=1+(digitalRead(WS_ID2)==HIGH?0:1)+(digitalRead(WS_ID1)==HIGH?0:1)*2;
 
  delay(500);
  for(uint8_t i=0; i<msg.sid; i++) {
      digitalWrite(RED_LED, HIGH);
      delay(500);
      digitalWrite(RED_LED, LOW);
      delay(500);
    }
    
  uint8_t power=NRF24::NRF24TransmitPower0dBm;
  if(msg.sid>1) power=NRF24::NRF24TransmitPowerm6dBm; // for remote locations
  
  if (!nrf24.init())
    err=3;
  else if (!nrf24.setChannel(WS_CHAN))
    err=4;
  else if (!nrf24.setThisAddress((uint8_t*)addr, strlen(addr)))
    err=5;
  else if (!nrf24.setPayloadSize(sizeof(wt_msg)))
    err=6;
  //else if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPower0dBm))
  //else if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPowerm6dBm))
  else if (!nrf24.setRF(NRF24::NRF24DataRate250kbps, power))
    err=7;    
  //nrf24.setRetry(2);
  nrf24.setRetry(10, 3);  
      
  if(!err) {
    if(WS_IS_NOACK) nrf24.spiWriteRegister(NRF24_REG_1D_FEATURE, NRF24_EN_DYN_ACK);  // NO ACK
  }
  else {
    for(uint8_t i=0; i<err; i++) {
      digitalWrite(RED_LED, HIGH);
      delay(200);
      digitalWrite(RED_LED, LOW);
      delay(200);
    }
  }
  
  //RLS_PIN(P1_0); 
  RLS_PIN(P1_1); RLS_PIN(P1_2); 
  RLS_PIN(P2_1); RLS_PIN(P2_2); RLS_PIN(P2_3); RLS_PIN(P2_4); RLS_PIN(P2_5);
  
  RLS_PIN(P1_3); 

/*  
  lpm_init(testmode); 
  lpm_delay(1); //
  */
  
  sleepSeconds(1);
}

void loop()
{
  uint8_t nret=3;
  do {
      term.GetData16_1();
      //lpm_delay(1);
      //delay(94); // 9-bit conversion
      sleep(94);
      msg.temp=term.GetData16_2();
  } while(DS18_MEAS_FAIL==msg.temp && --nret>0);
  
  RLS_PIN(P1_3);
  
  if(DS18_MEAS_FAIL!=msg.temp) msg.temp=(msg.temp*5)/8;
  
  if(!vcc_cnt) vcc=getVcc();  
  if(++vcc_cnt==VCC_CNT_N) vcc_cnt=0;
  
  //msg.vcc=getVcc();
  msg.vcc=vcc;
  
  nret=3;
  do {
    if (!nrf24.setTransmitAddress((uint8_t*)dst_addr, strlen(dst_addr))) 
      err=3;
    else if (!nrf24.send((uint8_t*)&msg, sizeof(msg), WS_IS_NOACK))
      err=4;  
    else if(!nrf24.waitPacketSent())
      err=5;
    else  
      err=0;
    if(err) sleepSeconds(1);  
  } while(err && --nret>0);
    
  nrf24.powerDown(); //!!!!! obviously need to make it to go to DEEP SLEEP!
  
  if(err) {
    digitalWrite(RED_LED, LOW);
    for(uint8_t i=0; i<err; i++) {
      digitalWrite(RED_LED, HIGH);
      //delay(200);
      sleep(200);
      digitalWrite(RED_LED, LOW);
      //delay(200);
      sleep(200);
    }
  }
  else {
    if(testmode) {
      digitalWrite(RED_LED, HIGH);
      //lpm_delay(1);
      sleepSeconds(1);
      digitalWrite(RED_LED, LOW);
    }
  }  
  
 //lpm_delay(sdelay);
 sleepSeconds(sdelay);
}


// returns VCC in .01 volts
int16_t getVcc() {
  // start with the 1.5V internal reference
  analogReference(INTERNAL1V5);
  int data = analogRead(11);
  // if overflow, VCC is > 3V, switch to the 2.5V reference
  if (data==0x3ff) {
    analogReference(INTERNAL2V5); // NOTE!!!! THIS DRAINS POWER!!!  any REF other than DEFAULT !!!
    data = (int16_t)map(analogRead(11), 0, 1023, 0, 500);
  } else {
    data = (int16_t)map(data, 0, 1023, 0, 300);
  }
  analogReference(DEFAULT);
  return data;  
}


/*
// LPM SECTION

volatile uint32_t lpm_sec=0; 
static uint8_t tm_step=1;

void lpm_delay(uint16_t s) {
  uint32_t tsec=lpm_sec+s;
  do { LPM3; } while(lpm_sec<tsec);
}

void lpm_init(uint8_t fast) {
 // TA0 (!!!) - to be compartible with 4552
  if(fast) {// 1 sec
   BCSCTL1 &= ~DIVA_3;     // ACLK without divider - nominally 12kHz 
   TA0CCR0=12000;     // 12000 ticks is approx 1 second
   tm_step=1;
  }
  else {// 10 sec
   BCSCTL1 |= DIVA_3;      
   TA0CCR0=15000;     
   tm_step=10;
  }
  BCSCTL3 = (LFXT1S_2);   // Source ACLK from VLO
  TA0CCTL0 = CCIE;             //  CCR0 interupt activated
  TA0CTL = TASSEL_1 | ID_0 | MC_1;	// Clock for TIMER = ACLK, By 1 division, up mode
}

__attribute__((interrupt(TIMER0_A0_VECTOR))) void RTC_isr(void)
{
 lpm_sec+=tm_step;
 _BIC_SR_IRQ(LPM3_bits); //exit at full speed so our main loop runs again.
}
*/

