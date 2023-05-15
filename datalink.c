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

//Frame Structure
typedef struct{
    uint8 kind;//Type of the Frame
    uint8 ack;//Piggybacking
    uint8 seq;//Sequence Number
    byte data[PKT_LEN];//Data of the Frame 
    uint32 padding;
}FRAME;
typedef FRAME* FRAME_ITER;

//DIY Constance
static const bool TRUE = 1;
static const bool FALSE = 0;
static const int32 DATA_TIMER = 1500; //超时时间1500ms
static const int32 ACK_TIMER = 300; //超时时间300ms
static const int32 TRAN_TIME = 1000*sizeof(FRAME)/8000;
static const uint32 ACK_TIMER_ID = 0x80;
//static const uint8 NAK_INTERVAL = 4;
//static const int32 CHANNEL_DELAY = 270;//270ms
#define MAX_SEQ 63
#define SEQ_MOD (MAX_SEQ + 1)
#define WINDOW_SIZE ((MAX_SEQ + 1) >> 1)



//Global Variables
static uint8 cnt_buffered;
static byte buffer[PKT_LEN];
static bool phl_ready = FALSE;

//Sliding Window Protocol 
static FRAME recv_window[WINDOW_SIZE],post_window[WINDOW_SIZE];
static bool recv_arrived[WINDOW_SIZE],post_arrived[WINDOW_SIZE];
static uint8 nak_counter[WINDOW_SIZE];
static uint8 frame_expect_new = 0;
static uint8 recv_front = 0;//Lower Edge of Receiver
static uint8 recv_tail = WINDOW_SIZE;
static uint8 oldest_frame_id = 0;
static uint8 next_frame_id = 0;

static uint8 ack_sequence[WINDOW_SIZE];
static uint8 ack_sequence_front = 0;
static uint8 ack_sequence_tail = 0; 


static bool within_range(uint8 l,uint8 r,uint8 val);
static uint8 recv_window_slide();
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
//Send NAK frame
static void send_nak_frame(uint8 seq);
//Choice which NAK to send
static void choice_nak_to_send();
int main(int argc, char **argv){
    
    protocol_init(argc,argv);
    lprintf("Designed by RowletQwQ, build: "__DATE__" "__TIME__"\n");
    
    disable_network_layer();
    int32 event, arg;
    int32 len = 0;
    FRAME f;
    memset(recv_arrived,0,sizeof(recv_arrived));
    memset(post_arrived,0,sizeof(post_arrived));
    memset(nak_counter,0,sizeof(nak_counter));
    for (;;){
        event = wait_for_event(&arg);
        switch(event){
            case NETWORK_LAYER_READY:
                //Get the packet, then push it into the post window, waiting for ACK
                get_packet(buffer);
                post_window_push(buffer,PKT_LEN);
                ++cnt_buffered;
                send_data_frame(next_frame_id);
                next_frame_id = (next_frame_id + 1) % (MAX_SEQ + 1);
                dbg_frame("Post Buffered Count %d,Next_Frame_Id %d\n",cnt_buffered,next_frame_id);
                break;
            
            case PHYSICAL_LAYER_READY:
                phl_ready = TRUE;
                break;

            case FRAME_RECEIVED:
                len = recv_frame((unsigned char *)&f, sizeof f);
                if (len < 5 || crc32((unsigned char *)&f, len) != 0) {
                    dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                    
                    //When accept an error Frame,Send Least Resend Frame
                    choice_nak_to_send();
                    break;
                }
                if(f.kind == FRAME_NAK){
                    dbg_frame("Recv NAK  %d\n", f.ack);
                    if(is_post_window_exist(f.ack) && get_timer(f.ack) < DATA_TIMER - ACK_TIMER - TRAN_TIME){
                        dbg_frame("Resend DATA %d, ID %d\n", f.ack, *(short *)post_window[f.ack%WINDOW_SIZE].data);
                        send_data_frame(f.ack);
                    }else{
                        dbg_frame("NAK %d is out of date\n",f.ack);
                    }
                    break;
                }
                if (f.kind == FRAME_ACK){
                    dbg_frame("Recv ACK  %d\n", f.ack);
                } 
                if (f.kind == FRAME_DATA) {
                    dbg_frame("Recv DATA %d, Piggybacking ACK %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                    push_ack_seq(f.seq);
                    //send_ack_frame(f.seq);
                    // check whether the ACK Timer exists
                    if(get_timer(ACK_TIMER_ID) == 0){
                        start_ack_timer(ACK_TIMER);//Start Timer for ACK, Piggybacking or Sending single ACK Frame
                    }
                    // check 
                    if(is_recv_waiting(f.seq) && !recv_arrived[f.seq%WINDOW_SIZE]){
                        //Update frame_expect_new to the newest possible Frame
                        if(frame_expect_new == f.seq){
                            frame_expect_new = (frame_expect_new + 1) % (MAX_SEQ + 1);
                            if(!is_recv_waiting(frame_expect_new)){
                                frame_expect_new = f.seq;
                            }
                        }
                        //frame_expect_new = f.seq;
                        dbg_frame("Confirm DATA %d, ID %d, Frame Excepted %d, Tail %d\n",f.seq,*(short *)f.data,recv_front,recv_tail);
                        recv_arrived[f.seq%WINDOW_SIZE] = TRUE;
                        recv_window[f.seq%WINDOW_SIZE] = f;
                        nak_counter[f.seq%WINDOW_SIZE] = 0;
                        
                        while(recv_arrived[recv_front%WINDOW_SIZE] == TRUE){
                            //Sliding the recv window, and update frame_expect_new
                            recv_arrived[recv_front%WINDOW_SIZE] = FALSE;
                            dbg_frame("Recv Window:Frame Excepted %d, Tail %d\n",recv_front,recv_tail);
                            FRAME_ITER buf = &recv_window[recv_window_slide()];
                            frame_expect_new = recv_front;
                            dbg_frame("Sending DATA %d to Network Layer,ID %d\n",buf->seq,*(short *)buf->data);
                            put_packet(buf->data,len - 7);
                            
                        }
                        
                    }
                } 
                if(is_post_window_exist(f.ack)){
                    //收到ACK,确认是否需要滑动发送窗口
                    dbg_frame("Correct ACK, Oldest Frame ID %d, Next Frame ID %d, Now ID %d\n",oldest_frame_id,next_frame_id,f.ack);
                    dbg_frame("Stop Timer %d\n",f.ack%WINDOW_SIZE);
                    
                    stop_timer(f.ack);
                    post_arrived[f.ack%WINDOW_SIZE] = TRUE;
                    //push_ack_seq(f.ack);
                    while(post_arrived[oldest_frame_id%WINDOW_SIZE]&&oldest_frame_id != next_frame_id){
                        //post_arrived[oldest_frame_id%WINDOW_SIZE] = FALSE;
                        --cnt_buffered;//此处减小规模
                        oldest_frame_id = (oldest_frame_id + 1) % (MAX_SEQ + 1);
                    }

                    dbg_frame("Post Buffered Count %d,Oldest_Frame_Id %d\n",cnt_buffered,oldest_frame_id);
                    
                }else{
                    dbg_frame("Bad ACK, Oldest Frame ID %d, Next Frame ID %d, Now ID %d\n",oldest_frame_id,next_frame_id,f.ack);
                }
                break;

            case DATA_TIMEOUT:
                dbg_event("---- DATA %d timeout\n", arg);
                dbg_frame("Resend DATA %d\n", arg);
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
static void choice_nak_to_send(){
    uint8 least_resend_frame = 0xff;
    uint8 least_resend_frame_cnt = 0xff;
    //From recv_front forward to frame_expect_new,get the frame do not received
    for(uint8 i = recv_front; i != frame_expect_new ; i = (i + 1) % (MAX_SEQ + 1)){
        if(!recv_arrived[i%WINDOW_SIZE] && nak_counter[i%WINDOW_SIZE] < least_resend_frame_cnt){
            least_resend_frame_cnt = nak_counter[i%WINDOW_SIZE];
            least_resend_frame = i;
        }
    }
    if(!recv_arrived[frame_expect_new%WINDOW_SIZE] && nak_counter[frame_expect_new%WINDOW_SIZE] < least_resend_frame_cnt ){
        least_resend_frame = frame_expect_new;
        least_resend_frame_cnt = nak_counter[frame_expect_new];
    }
    nak_counter[least_resend_frame%WINDOW_SIZE]++;
    dbg_frame("Least Resend Frame %d, ID %d, Count %d\n",least_resend_frame,*(short *)recv_window[least_resend_frame%WINDOW_SIZE].data,
    nak_counter[least_resend_frame%WINDOW_SIZE]);
    send_nak_frame(least_resend_frame);
}

static bool within_range(uint8 l,uint8 r,uint8 val){
    if(l<r){

        return (l <= val) && (val < r);
    }
    return (l <= val) || (val < r);
}
static bool is_recv_waiting(uint8 seq){
    if(seq > MAX_SEQ){
        //dbg_event("***is_recv_waiting:Bad Sequence Number, Except No More Than %u, But Get %u\n",MAX_SEQ,seq);
        return FALSE;
    }
    if(recv_front == recv_tail){
        //Empty
        return FALSE;
    }
    return within_range(recv_front,recv_tail,seq);
}
static bool is_post_window_exist(uint8 seq){
    if(seq > MAX_SEQ){
        //dbg_event("***is_post_window_exist:Bad Sequence Number, Except No More Than %u, But Get %u\n",MAX_SEQ,seq);
        return FALSE;
    }
    if(oldest_frame_id == next_frame_id){
        return FALSE;
    }
    return within_range(oldest_frame_id,next_frame_id,seq);
}
static uint8 recv_window_slide(){
    uint8 ret = recv_front%WINDOW_SIZE;
    //FRAME ret = recv_window[recv_front%WINDOW_SIZE];
    recv_front = (recv_front + 1) % (MAX_SEQ + 1);
    recv_tail = (recv_tail + 1) %(MAX_SEQ + 1);
    return ret;
}
static void send_data_frame(uint8 seq){
    FRAME_ITER iter = &post_window[seq%WINDOW_SIZE];
    
    put_frame((byte*)iter,3 + PKT_LEN);

    dbg_frame("Send DATA %d, Seq Num %d, Piggybacking %d, ID %d\n", iter->seq, seq, iter->ack, *(short *)iter->data);
    start_timer(seq,DATA_TIMER);
    dbg_frame("Start Timer %d\n",seq);
    //stop_ack_timer();
    post_arrived[seq%WINDOW_SIZE] = FALSE;
    
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
    iter->kind = FRAME_DATA;
    iter->seq = next_frame_id;
    if(is_ack_seq_empty()){
        iter->ack = MAX_SEQ + 1;//NO ACK Provided
    }else{
        iter->ack = pop_oldest_ack_seq();
    }
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

static void send_nak_frame(uint8 seq){
    FRAME s;
    s.kind = FRAME_NAK;
    s.ack = seq;
    
    dbg_frame("Send NAK  %d\n", s.ack);

    put_frame((unsigned char *)&s, 2);
}