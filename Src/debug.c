#include "main.h"
#include "debug.h"
#include "lmic.h"

/*  ************************************** */
/*    DO NOT CHANGE BELOW THIS LINE        */
/*  ************************************** */

void debug_init () {
    // configure LED pin as output
//    debug_led(0);

    // configure USART1 (115200/8N1, tx-only)

    // print banner
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led (int val) {
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,val);
}

void debug_char (char c) {
  char buffer[] = "";
  buffer[0]= c;
}

void debug_hex (u1_t b) {
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_buf (const u1_t* buf, int len) {
    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_int (s4_t v) {
    char buf[10], *p = buf;
    int n = debug_fmt(buf, sizeof(buf), v, 10, 0, 0);
    while(n--)
        debug_char(*p++);
}

void debug_str (const char* str) {
    while(*str) {
        debug_char(*str++);
    }
}

void debug_float2str (double val, int n) {
	char buf[100], *p = buf;
	//int n = 7;	// number of digits
	gcvt(val, n, buf);
    while(n--)
        debug_char(*p++);
}

void debug_valfloat (const char* label, double val, int n) {
    debug_str(label);
    debug_float2str(val,n);
    debug_char('\r');
    debug_char('\n');
}

void debug_val (const char* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_valdec (const char* label, s4_t val) {
    debug_str(label);
    debug_int(val);
    debug_char('\r');
    debug_char('\n');
}

int debug_fmt (char* buf, int max, s4_t val, int base, int width, char pad) {
    char num[33], *p = num, *b = buf;
    u4_t m, v;
    // special handling of negative decimals
    v = (base == 10 && val < 0) ? -val : val;
    // generate digits backwards
    do {
        *p++ = ((m=v%base) <= 9) ? m+'0' : m+'A'-10;
    } while( v /= base );
    // prefix negative decimals with '-'
    if(base == 10 && val < 0) {
        *p++ = '-';
    }
    // add leading zeroes or spaces
    while( b-buf < max-1 && b-buf < width-(p-num) ) {
        *b++ = pad;
    }
    // copy digits and sign forwards
    do *b++ = *--p;
    while( b-buf < max && p > num );
    // return number of characters written
    return b - buf;
}

void debug_event (int ev) {
    static const char* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
        [EV_SCAN_FOUND]     = "SCAN_FOUND",
        [EV_TXSTART]        = "EV_TXSTART",
    };
    debug_str((ev < sizeof(evnames)/sizeof(evnames[0])) ? evnames[ev] : "EV_UNKNOWN" );
    debug_char('\r');
    debug_char('\n');
}
