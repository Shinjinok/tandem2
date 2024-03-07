/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of FlightGear2
*/

#include "SIM_FlightGear2.h"

#if HAL_SIM_FLIHGTGEAR2_ENABLED

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;


static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "BRD_OPTIONS", 0},
    { "INS_GYR_CAL", 0 },
    { "AHRS_EKF_TYPE", 10},// XPlane sensor data is not good enough for EKF. Use fake EKF by default
    { "GPS_TYPE", 1 },
    { "RPM_TYPE", 10 },//RPM_TYPE_SITL
    { "CAN_D1_UC_OPTION", 288 },//5:send gnss, 8:enable stats
    { "DISARM_DELAY", 0},
    { "ATC_HOVR_ROL_TRM",    0 },
    { "CAN_SLCAN_CPORT",   0 },
    { "CAN_P1_DRIVER",   1 },
    { "ATC_ANG_PIT_P",    1.0 },
    { "ATC_ANG_RLL_P",    1.0 },
    { "ATC_ANG_YAW_P",    1.0 },
    { "ATC_RAT_PIT_D",     0.5 },
    { "ATC_RAT_PIT_I",     0.01 },
    { "ATC_RAT_PIT_P",     1.0 },
    { "ATC_RAT_RLL_D",     0.5 },
    { "ATC_RAT_RLL_I",     0.01 },
    { "ATC_RAT_RLL_P",     1.0 },
    { "ATC_RAT_YAW_D",     0.5 },
    { "ATC_RAT_YAW_I",     0.01 },
    { "ATC_RAT_YAW_P",     1.0 },
    { "PSC_VELXY_D",     0.25 },
    { "PSC_VELXY_P",     0.5 },
    { "PSC_VELXY_I",     0.01 },
    { "PSC_VELZ_D",     0.5 },
    { "PSC_VELZ_P",     1.0 },
    { "PSC_VELZ_I",     0.01 },
    { "ATC_RATE_FF_ENAB",     0 },
    { "PSC_POSXY_P",     2.0 },

};

FlightGear2::FlightGear2(const char *frame_str) :
    Aircraft(frame_str),
    //last_timestamp(0),
    sock(true)
{
    fprintf(stdout, "Starting SITL FlightGear2\n");
    float val;
    for (uint8_t i=0; i<ARRAY_SIZE(sim_defaults); i++) {

    AP_Param::set_and_save_by_name(sim_defaults[i].name, sim_defaults[i].value);
    AP_Param::get(sim_defaults[i].name, val);
    printf("%s set value to %f (%f)\n",sim_defaults[i].name,  sim_defaults[i].value, val);
    if(abs(sim_defaults[i].value - val) > 1e-10){
        
    }
    
        if (sim_defaults[i].save) {
            
            enum ap_var_type ptype;
            AP_Param *p = AP_Param::find(sim_defaults[i].name, &ptype);
            if (!p->configured()) {
                p->save();
            }
        }
    
    }
}

/*
  Create and set in/out socket
*/
void FlightGear2::set_interface_ports(const char* address, const int port_in_, const int port_out_)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // FlightGear2 keeps sending us packets. Not strictly necessary but
    // useful for debugging
    
    int port_in =  port_in_ + instance * 10;
    
    int  port_out = port_out_ + instance * 10;

    printf("port_in %d port_out %d\n ",port_in,port_out);
    if (!sock.bind(address, port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        usleep(6e7);
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", address, port_in);
    sock.reuseaddress();
    sock.set_blocking(false);

    _FlightGear2_address = address;
    _FlightGear2_port = port_out;
    printf("Setting FlightGear2 interface to %s:%d \n", _FlightGear2_address, _FlightGear2_port);
}

/*
  decode and send servos
*/
void FlightGear2::send_servos(const struct sitl_input &input)
{
   udp_in_packet pkt;

    pkt.serveo[0] = (float) (input.servos[0]-1500) / 250.0f ;
    pkt.serveo[1] = (float) (input.servos[1]-1500) / -250.0f ;
    pkt.serveo[2] = (float) (2000-input.servos[2]) / 1000.0f;
    pkt.serveo[3] = (float) (input.servos[3]-1500) / 500.0f ;
    pkt.serveo[4] = (float) (2000-input.servos[2]) / 1000.0f;
    

    /*pkt.serveo[0] = (ch[0] -0.5)*2.0;
    pkt.serveo[1] = (-ch[1] +0.5)*2.0;
    pkt.serveo[2] = 1.0- ch[2];
    pkt.serveo[3] = (ch[3]-0.5)*2.0;
    pkt.serveo[4] = 1.0- ch[2];*/


    uint32_t data[5];
    data[0] = __bswap_32(pkt.data[0]);
    data[1] = __bswap_32(pkt.data[1]);
    data[2] = __bswap_32(pkt.data[2]);
    data[3] = __bswap_32(pkt.data[3]);
    data[4] = __bswap_32(pkt.data[4]);
    sock.sendto(&data, sizeof(data), _FlightGear2_address, _FlightGear2_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void FlightGear2::recv_fdm(const struct sitl_input &input)
{
    U_packet pkt;
    D_packet dp;
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
   
   // uint64_t now = AP_HAL::micros64();
    

    
    while (sock.recv(&dp, RCV_SIZE, 100) != RCV_SIZE) {
      //  printf("!= RCV_SIZE\n");
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch FlightGear2 reset
        if (get_wall_time_us() > last_wall_time_us + FlightGear2_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }
    
    

    for (long unsigned int i=0; i < NUM_64_DATA; i++){
        pkt.dp.data64[i] = __bswap_64(dp.data64[i]);
    }

    for (long unsigned int i=0; i < NUM_32_DATA; i++){
        pkt.dp.data32[i] = __bswap_32(dp.data32[i]);
    }

   
    accel_body = Vector3f(pkt.g_packet.pilot_accel_swu_xyz[0]* FEET_TO_METERS,
                          pkt.g_packet.pilot_accel_swu_xyz[1]* FEET_TO_METERS,
                          pkt.g_packet.pilot_accel_swu_xyz[2]* FEET_TO_METERS);

    gyro = Vector3f(pkt.g_packet.pqr_rad[0],
                    pkt.g_packet.pqr_rad[1],
                    pkt.g_packet.pqr_rad[2]);

     // compute dcm from imu orientation
    dcm.from_euler(pkt.g_packet.orientation_rpy_deg[0]*DEG_TO_RAD_DOUBLE,
                   pkt.g_packet.orientation_rpy_deg[1]*DEG_TO_RAD_DOUBLE,
                   pkt.g_packet.orientation_rpy_deg[2]*DEG_TO_RAD_DOUBLE); 
                                  

    velocity_ef = Vector3f(pkt.g_packet.speed_ned_fps[0]*FEET_TO_METERS,
                        pkt.g_packet.speed_ned_fps[1]*FEET_TO_METERS,
                        pkt.g_packet.speed_ned_fps[2]*FEET_TO_METERS);

    location.lat = pkt.g_packet.lat_lon[0] * 1.0e7;
    location.lng = pkt.g_packet.lat_lon[1] * 1.0e7;
    location.alt = pkt.g_packet.alt * 100.0f;
    //position.xy().zero();
    //position.z = -pkt.g_packet.alt * 100.0f;
    position = origin.get_distance_NED_double(location);


    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_ef;

    // airspeed
    airspeed = velocity_air_bf.length();

    // airspeed as seen by a fwd pitot tube (limited to 120m/s)
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);

    motor_mask = 1;
    rpm[0] = pkt.g_packet.rpm;

    // Convert from a meters from origin physics to a lat long alt
    update_position();

    double deltat;
    if (pkt.g_packet.timestamp < last_timestamp) {
        // Physics time has gone backwards, don't reset AP
        printf("Detected physics reset\n");
        deltat = 0;
    //    last_received_bitmask = 0;
    } else {
        deltat = pkt.g_packet.timestamp - last_timestamp;
    }
    time_now_us += deltat * 1.0e6;

    if (is_positive(deltat) && deltat < 0.1) {
        // time in us to hz
        if (use_time_sync) {
            adjust_frame_time(1.0 / deltat);
        }
        // match actual frame rate with desired speedup
        time_advance();
    }
    last_timestamp = pkt.g_packet.timestamp;
    frame_counter++;


}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void FlightGear2::drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do {
        received = sock.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);

}

/*
  update the FlightGear2 simulation by one time step
 */
void FlightGear2::update(const struct sitl_input &input)
{
    send_servos(input);
    
    recv_fdm(input);
    
    // update magnetic field
    update_mag_field_bf();

// allow for changes in physics step
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));

    //printf("FPS %.2f\n", rate_hz); // this is instantaneous rather than any clever average sjo edited
    //drain_sockets();
    
}




#endif  // HAL_SIM_FlightGear2_ENABLED
