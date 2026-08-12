#ifndef RECEIVER_C
#define RECEIVER_C

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#define UDP_PORT 8804
#define MAX_CHUNKS 256
#define CHUNK_SIZE 2048
#define MAX_PAYLOAD_SIZE (MAX_CHUNKS * CHUNK_SIZE)

// Frame buffer structure
typedef struct {
    uint8_t *data;
    int data_len;
    uint8_t chunks[MAX_CHUNKS][CHUNK_SIZE];
    int chunk_lens[MAX_CHUNKS];
    uint8_t chunk_present[MAX_CHUNKS]; // Nuevo: tracker de chunks
    int expected_chunks;
    int received_chunks;
} FrameBuffer;

static int sock = -1;
static FrameBuffer fb;

// Debug stats
static int dbg_pkts_recv = 0;
static int dbg_frames_ok = 0;
static int dbg_frames_dropped = 0;
static int dbg_bad_magic = 0;
static int dbg_missing_chunks = 0;
static time_t last_print = 0;

void print_debug_stats() {
    time_t now = time(NULL);
    if (now - last_print >= 1) {
        printf("[C-LOG] Packets: %d | BadMagic: %d | FramesOK: %d | MissingChunks: %d | FramesDropped: %d\n", 
               dbg_pkts_recv, dbg_bad_magic, dbg_frames_ok, dbg_missing_chunks, dbg_frames_dropped);
        fflush(stdout);
        
        dbg_pkts_recv = 0;
        dbg_frames_ok = 0;
        dbg_frames_dropped = 0;
        dbg_bad_magic = 0;
        dbg_missing_chunks = 0;
        last_print = now;
    }
}

int init_receiver(int fd, int rcvbuf_size) {
    sock = fd;
    
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));
    
    // Set non-blocking timeout via SO_RCVTIMEO
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000; // 500ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Init frame buffer
    fb.data = malloc(MAX_PAYLOAD_SIZE);
    fb.data_len = 0;
    fb.expected_chunks = 0;
    fb.received_chunks = 0;
    memset(fb.chunk_present, 0, sizeof(fb.chunk_present));
    
    return sock;
}

// Blocks until a complete frame is received.
// Returns the size of the assembled JPEG scan, or -1 on timeout/error.
int receive_frame(uint8_t *out_buffer, int max_size) {
    uint8_t buf[2048];
    
    if (last_print == 0) last_print = time(NULL);
    
    while (1) {
        print_debug_stats();
        
        int len = recv(sock, buf, sizeof(buf), 0);
        if (len < 0) {
            return -1; // timeout or error
        }
        
        dbg_pkts_recv++;
        
        if (len < 56) continue;
        
        // Check magic
        if (buf[0] != 0x55 || buf[1] != 0xaa || buf[2] != 0x55 || buf[3] != 0xaa) {
            dbg_bad_magic++;
            continue;
        }
        
        uint16_t idx = buf[32] | (buf[33] << 8);
        uint16_t total = buf[36] | (buf[37] << 8);
        
        if (idx == 0) {
            // Check if we had a complete frame pending
            int assembled_len = 0;
            if (fb.expected_chunks > 0) {
                if (fb.received_chunks == fb.expected_chunks) {
                    for (int i = 0; i < fb.expected_chunks; i++) {
                        if (assembled_len + fb.chunk_lens[i] <= max_size) {
                            memcpy(out_buffer + assembled_len, fb.chunks[i], fb.chunk_lens[i]);
                            assembled_len += fb.chunk_lens[i];
                        }
                    }
                    dbg_frames_ok++;
                } else {
                    dbg_missing_chunks++;
                    dbg_frames_dropped++;
                }
            }
            
            // Start new frame
            fb.expected_chunks = total;
            fb.received_chunks = 0;
            memset(fb.chunk_present, 0, sizeof(fb.chunk_present));
            
            if (assembled_len > 0) {
                // Save the new chunk 0 for the NEXT call!
                fb.chunk_lens[idx] = len - 56;
                memcpy(fb.chunks[idx], buf + 56, len - 56);
                fb.chunk_present[idx] = 1;
                fb.received_chunks++;
                return assembled_len; // Return the PREVIOUS frame
            }
        }
        
        if (idx < MAX_CHUNKS && !fb.chunk_present[idx]) {
            fb.chunk_lens[idx] = len - 56;
            memcpy(fb.chunks[idx], buf + 56, len - 56);
            fb.chunk_present[idx] = 1;
            fb.received_chunks++;
        }
        
        // Frame complete?
        if (fb.expected_chunks > 0 && fb.received_chunks == fb.expected_chunks) {
            int assembled_len = 0;
            for (int i = 0; i < fb.expected_chunks; i++) {
                if (assembled_len + fb.chunk_lens[i] <= max_size) {
                    memcpy(out_buffer + assembled_len, fb.chunks[i], fb.chunk_lens[i]);
                    assembled_len += fb.chunk_lens[i];
                }
            }
            
            // Reset state
            fb.expected_chunks = 0;
            fb.received_chunks = 0;
            memset(fb.chunk_present, 0, sizeof(fb.chunk_present));
            
            dbg_frames_ok++;
            return assembled_len;
        }
    }
}

#endif
