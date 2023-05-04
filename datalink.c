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
static const uint32 ACK_TIMER = 1;
#define MAX_SEQ 7
#define SEQ_MOD (MAX_SEQ + 1)
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
static uint8 oldest_frame_id = 0;
static uint8 next_frame_id = 0;

static uint8 ack_sequence[WINDOW_SIZE];
static uint8 ack_sequence_front = 0;
static uint8 ack_sequence_tail = 0; 


static bool within_range(uint8 l,uint8 r,uint8 val);
static FRAME recv_window_slide();
static bool is_recv_waiting(uint8 seq);

static void post_window_push(byte *buf,int32 len);
static bool is_post_window_exist(uint8 seq);

static uint8 pop_oldest_ack_seq();
static void push_ack_seq(uint8 seq);
static bool is_ack_seq_empty();

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
                next_frame_id = (next_frame_id + 1) % (MAX_SEQ + 1);
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
                    dbg_frame("Recv DATA %d, Piggybacking ACK %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                    if(is_recv_waiting(f.seq) && !recv_arrived[f.seq%WINDOW_SIZE]){
                        recv_arrived[f.seq%WINDOW_SIZE] = TRUE;
                        recv_window[f.seq%WINDOW_SIZE] = f;
                        send_ack_frame(f.seq);
                        //push_ack_seq(f.seq);
                        //start_ack_timer(ACK_TIMER);//Start Timer for ACK, Piggybacking or Sending single ACK Frame
                        while(recv_arrived[frame_expected%WINDOW_SIZE]){
                            recv_arrived[frame_expected%WINDOW_SIZE] = FALSE;
                            FRAME buf = recv_window_slide();
                            put_packet(buf.data,len - 7);
                            
                        }
                    }
                    
                } 
                if(is_post_window_exist(f.ack)){
                    //收到ACK,确认是否需要滑动发送窗口
                    --cnt_buffered;
                    dbg_frame("Stop Timer %d\n",oldest_frame_id%WINDOW_SIZE);
                    stop_timer(oldest_frame_id%WINDOW_SIZE);
                    //push_ack_seq(f.ack);
                    while(!is_ack_seq_empty()&&oldest_frame_id == ack_sequence[ack_sequence_front % WINDOW_SIZE]){
                        pop_oldest_ack_seq();
                        oldest_frame_id = (oldest_frame_id + 1) % (MAX_SEQ + 1);
                    }
                    
                }
                break;

            case DATA_TIMEOUT:
                dbg_event("---- DATA %d timeout\n", arg); 
                send_data_frame(arg);
                break;

            case ACK_TIMEOUT:
                //TODO if there are any other ACK,Clear then all
                while(!is_ack_seq_empty()){
                    uint8 ack_num = pop_oldest_ack_seq();
                    send_ack_frame(ack_num);
                }
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
        dbg_warning("is_post_window_exist:Bad Sequence Number, Except No More Than %u, But Get %u\n",MAX_SEQ,seq);
        return FALSE;
    }
    if(oldest_frame_id == next_frame_id){
        return FALSE;
    }
    return within_range(oldest_frame_id,next_frame_id,seq);
}
static FRAME recv_window_slide(){
    FRAME ret = recv_window[frame_expected%WINDOW_SIZE];
    frame_expected = (frame_expected + 1) % (MAX_SEQ + 1);
    recv_tail = (recv_tail + 1) %(MAX_SEQ + 1);
    return ret;
}
static void send_data_frame(uint8 seq){
    FRAME_ITER iter = &post_window[seq%WINDOW_SIZE];
    iter->kind = FRAME_DATA;
    iter->seq = seq;
    if(is_ack_seq_empty()){
        iter->ack = MAX_SEQ + 1;
    }else{
        iter->ack = pop_oldest_ack_seq();
    }
    put_frame((byte*)iter,3 + PKT_LEN);
    
    dbg_frame("Send DATA %d %d, ID %d\n", iter->seq, iter->ack, *(short *)iter->data);
    start_timer(seq,DATA_TIMER);
    //stop_ack_timer();

    //TODO 如果还有ACK帧,重开ACK timer
    /*if(!is_ack_seq_empty()){
        start_ack_timer(ACK_TIMER);
    }*/
}
static void put_frame(byte *frame, int len){
    *(uint32 *)(frame + len) = crc32(frame, len);
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

static uint8 pop_oldest_ack_seq(){
    uint8 ret = ack_sequence[ack_sequence_front % WINDOW_SIZE];
    ack_sequence_front = (ack_sequence_front + 1) % SEQ_MOD;
    return ret;
}
static void push_ack_seq(uint8 seq){
    ack_sequence[ack_sequence_tail % WINDOW_SIZE] = seq;
    ack_sequence_tail = (ack_sequence_tail + 1) % SEQ_MOD;
}
static bool is_ack_seq_empty(){
    return ack_sequence_front==ack_sequence_tail;
}