#ifndef SEND_DATA_H
#define SEND_DATA_H

#include <time.h>


#define NUM_FLOATS 300000  // 1000000

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    time_t unix_time;
    size_t num_points;
    double angle_velocity_x;
    double angle_velocity_y;
    double angle_velocity_z;
    double *float_array_ptr;
} data_packet;


int send_data(const char* ip_addr, const char* port_num, data_packet* packet_ptr);

#ifdef __cplusplus
}
#endif

#endif // SEND_DATA_H