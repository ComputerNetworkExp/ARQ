#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

//DIY datatype
typedef unsigned char uint8;
typedef char int8;
typedef unsigned short uint16;
typedef short int16;  
typedef int int32;
typedef unsigned int uint32;
typedef uint8 byte;
typedef unsigned char bool;

//DIY Constance
static const bool TRUE = 1;
static const bool FALSE = 0;
static const uint32 DATA_TIMER = 2000; //超时时间2s
static const uint32 ACK_TIMER = 200;
#define MAX_SEQ 7
#define WINDOW_SIZE ((MAX_SEQ + 1) >> 1)

//Frame Structure
typedef struct{
    uint8 kind;//Type of the Frame
    uint8 ack;//Piggybacking
    uint8 seq;//Sequence Number
    byte data[PKT_LEN];//Data of the Frame 
    uint32 padding;
}FRAME;
typedef FRAME* FRAME_ITER;

//Global Variables
static uint8 cnt_buffered;
static byte buffer[PKT_LEN];
static bool phl_ready = FALSE;
//Sliding Window Protocol 
static FRAME recv_window[WINDOW_SIZE],post_window[WINDOW_SIZE];
static bool recv_arrived[WINDOW_SIZE];
static uint8 frame_expected = 0;//Lower Edge of Receiver
static uint8 recv_tail = WINDOW_SIZE;
static uint8 ack_expected = 0;
static uint8 next_frame_id = 0;

static bool within_range(uint8 l,uint8 r,uint8 val);
//static void recv_window_push(byte *buf,int32 len);
static FRAME recv_window_slide();
//static bool is_recv_window_empty();
//Confirm whether the frame is required
static bool is_recv_waiting(uint8 seq);


static void post_window_push(byte *buf,int32 len);
//static FRAME post_window_slide();
//static bool is_post_window_empty();
static bool is_post_window_exist(uint8 seq);

//Send data frame
static void send_data_frame(uint8 seq);
//Add CRC code
static void put_frame(byte *frame, int len);
//Send ACK frame
static void send_ack_frame(uint8 seq);
int main(int argc, char **argv){
    
    protocol_init(argc,argv);
    lprintf("Designed by RowletQwQ, build: "__DATE__" "__TIME__"\n");
    
    disable_network_layer();
    int32 event, arg;
    int32 len = 0;
    FRAME f;

    memset(recv_arrived,0,sizeof(recv_arrived));
    for (;;){
        event = wait_for_event(&arg);
        switch(event){
            case NETWORK_LAYER_READY:
                get_packet(buffer);
                post_window_push(buffer,PKT_LEN);
                ++cnt_buffered;
                send_data_frame(next_frame_id);
                next_frame_id = (next_frame_id + 1) % MAX_SEQ;
                break;
            
            case PHYSICAL_LAYER_READY:
                phl_ready = TRUE;
                break;

            case FRAME_RECEIVED:
                len = recv_frame((unsigned char *)&f, sizeof f);
                if (len < 5 || crc32((unsigned char *)&f, len) != 0) {
                    dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                    break;
                }
                if (f.kind == FRAME_ACK) 
                    dbg_frame("Recv ACK  %d\n", f.ack);
                if (f.kind == FRAME_DATA) {
                    dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                    if(is_recv_waiting(f.seq) && !recv_arrived[f.seq%WINDOW_SIZE]){
                        recv_arrived[f.seq%WINDOW_SIZE] = TRUE;
                        recv_window[f.seq%WINDOW_SIZE] = f;
                        while(recv_arrived[frame_expected%WINDOW_SIZE]){
                            recv_arrived[frame_expected%WINDOW_SIZE] = FALSE;
                            FRAME buf = recv_window_slide();
                            put_packet(buf.data,len - 7);
                            start_ack_timer(ACK_TIMER);
                        }
                    }
                    
                } 
                while(is_post_window_exist(f.ack)){
                    --cnt_buffered;
                    stop_timer(ack_expected%WINDOW_SIZE);
                    ack_expected = (ack_expected + 1) % (MAX_SEQ + 1);
                }
                break;

            case DATA_TIMEOUT:
                dbg_event("---- DATA %d timeout\n", arg); 
                send_data_frame(arg);
                break;

            case ACK_TIMEOUT:
                send_ack_frame(frame_expected);
                break;
        }

        if(cnt_buffered < WINDOW_SIZE && phl_ready){
            enable_network_layer();
        }else{
            disable_network_layer();
        }
    }
    
    return 0;
}
static bool within_range(uint8 l,uint8 r,uint8 val){
    if(l<r){
        return l <= val && val < r;
    }
    return l >= val || val < r;
}
static bool is_recv_waiting(uint8 seq){
    if(seq > MAX_SEQ){
        dbg_warning("Bad Sequence Number, Except No More Than %u, But Get %u\n",MAX_SEQ,seq);
        return FALSE;
    }
    if(frame_expected == recv_tail){
        //Empty
        return FALSE;
    }
    return within_range(frame_expected,recv_tail,seq);
}
static bool is_post_window_exist(uint8 seq){
    if(seq > MAX_SEQ){
        dbg_warning("Bad Sequence Number, Except No More Than %u, But Get %u\n",MAX_SEQ,seq);
        return FALSE;
    }
    if(ack_expected == next_frame_id){
        return FALSE;
    }
    return within_range(ack_expected,next_frame_id,seq);
}
static FRAME recv_window_slide(){
    FRAME ret = recv_window[frame_expected%WINDOW_SIZE];
    frame_expected = (frame_expected + 1) % MAX_SEQ;
    recv_tail = (recv_tail + 1) %MAX_SEQ;
    return ret;
}
static void send_data_frame(uint8 seq){
    FRAME_ITER iter = &post_window[seq%WINDOW_SIZE];
    iter->kind = FRAME_DATA;
    iter->seq = seq;
    iter->ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    put_frame((byte*)iter,3 + PKT_LEN);
    
    dbg_frame("Send DATA %d %d, ID %d\n", iter->seq, iter->ack, *(short *)iter->data);
    start_timer(seq,DATA_TIMER);
    stop_ack_timer();
}
static void put_frame(byte *frame, int len){
    *(byte *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void post_window_push(byte *buf,int32 len){
    FRAME_ITER iter = &post_window[next_frame_id%WINDOW_SIZE];
    memcpy(iter->data,buf,len);
}
static void send_ack_frame(uint8 seq){
    FRAME s;
    s.kind = FRAME_ACK;
    s.ack = seq;
    
    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame((unsigned char *)&s, 2);
}