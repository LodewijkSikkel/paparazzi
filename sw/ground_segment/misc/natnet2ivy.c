/*
 * Copyright (C) 2014 Freek van Tienen
 *               2015 Lodewijk Sikke <l.n.c.sikkel@tudelft.nl>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** 
 * \file natnet2ivy.c
 * \brief NatNet (GPS) to ivy bridge
 *
 * This receives aircraft position information through the Optitrack system
 * NatNet UDP stream and forwards it to the ivy bus. An aircraft with the gps
 * subsystem "datalink" is then able to parse the GPS position and use it to
 * navigate inside the Optitrack system.
 *
 * The local reference frame is defined as a local right-handed coordinate
 * frame. The x-axis coincides with the Optitrack x-axis, whereas the y-axis
 * is in the same plane parallel to the ground. The z-axis complements the
 * reference frame and is pointing down.
 *
 *   ^ x
 *   |
 *   |
 * z * ----> y
 *
 * All computations are in floats as it makes no sense of increasing the 
 * precision while the Optitrack tracking system outputs are also in floats.
 */

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <time.h>

#include "std.h"
#include "arch/linux/udp_socket.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra_float.h" 
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"

/** Debugging options */
uint8_t verbose = 0;
#define printf_natnet   if(verbose == 2) printf
#define printf_debug    if(verbose == 1) printf

/** NatNet defaults */
char *natnet_addr               = "255.255.255.255";
char *natnet_multicast_addr     = "239.255.42.99";
uint16_t natnet_cmd_port        = 1510;
uint16_t natnet_data_port       = 1511;
uint8_t natnet_major            = 2;
uint8_t natnet_minor            = 7;

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255:2010";
#endif

/** Sample frequency and derevitive defaults */
uint32_t freq_transmit          = 30;     ///< Transmitting frequency in Hz
uint16_t min_velocity_samples   = 4;      ///< The amount of position samples needed for a valid velocity
bool small_packets              = FALSE;

/** Connection timeout when not receiving **/
#define CONNECTION_TIMEOUT          .5

/** NatNet parsing defines */
#define MAX_PACKETSIZE    100000
#define MAX_NAMELENGTH    256
#define MAX_rigid_bodies   128

#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999

/** Tracked rigid bodies */
struct RigidBody {
  int id;                           ///< Rigid body ID from the tracking system
  struct FloatVect3 opti_coord;     ///< Rigid body x, y and z position in meters in Optitrack coordinates in meters
  struct FloatQuat opti_orient;     ///< Rigid body orientation in radians represented as a quaternion in the Optitrack coordinate frame
  struct FloatVect3 local_coord;    ///< Rigid body x, y and z position in meters in the local coordinate frame
  struct FloatQuat local_orient;    ///< Rigid body orientation in radians represented as a quaternion in the local coordinate frame
  struct FloatVect3 tracking_coord; ///< Rigid body x, y and z position in meters in the desired tracking orientation 
  struct FloatQuat tracking_orient; ///< Rigid body orientation in radians represented as a quaternion in the desired tracking orientation
  int nMarkers;                     ///< Number of markers inside the rigid body (both visible and not visible)
  float error;                      ///< Error of the position in cm
  int nSamples;                     ///< Number of samples since last transmit
  bool posSampled;                  ///< If the position is sampled last sampling

  struct FloatVect3 local_vel;      ///< Sum of the (last_vel_* - current_vel_*) during nVelocitySamples
  struct FloatVect3 tracking_vel;   ///< Velocity in meters per second in the desired tracking orientation
  struct EcefCoor_f ecef_vel;       ///< Last valid ECEF velocity in meters
  int nVelocitySamples;             ///< Number of velocity samples gathered
  int totalVelocitySamples;         ///< Total amount of velocity samples possible
  int nVelocityTransmit;            ///< Amount of transmits since last valid velocity transmit
};
struct RigidBody rigid_bodies[MAX_rigid_bodies];    ///< All rigid bodies which are tracked

/** Mapping between rigid body and aircraft */
struct Aircraft {
  uint8_t ac_id;
  float lastSample;
  bool connected;
};
struct Aircraft aircrafts[MAX_rigid_bodies];                  ///< Mapping from rigid body ID to aircraft ID

/** Natnet socket connections */
struct UdpSocket natnet_data, natnet_cmd;

/** Rotation from Optitrack reference frame to local reference frame */
struct FloatQuat local_ref;

/** Tracking location LTP and orientation with respect to the local reference frame */
struct LtpDef_f tracking_ltp;      ///< The tracking system LTP definition
struct FloatQuat tracking_ref_quat;    ///< The orientation of the tracking system represented as a quaternion w.r.t. local reference frame
struct FloatEulers tracking_ref_eulers; ///< The orientation of the tracking system represented as Euler angles w.r.t local reference frame

/** Save the latency from natnet */
float natnet_latency;

/** Parse the packet from NatNet */
void natnet_parse(unsigned char *in)
{
  int i, j, k;

  // Create a pointer to go trough the packet
  char *ptr = (char *)in;
  printf_natnet("Begin Packet\n-------\n");

  // Message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2); ptr += 2;
  printf_natnet("Message ID : %d\n", MessageID);

  // Packet size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2); ptr += 2;
  printf_natnet("Byte count : %d\n", nBytes);

  if (MessageID == NAT_FRAMEOFDATA) {   // FRAME OF MOCAP DATA packet
    // Frame number
    int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
    printf_natnet("Frame # : %d\n", frameNumber);

    // ========== MARKERSETS ==========
    // Number of data sets (markersets, rigid_bodies, etc)
    int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
    printf_natnet("Marker Set Count : %d\n", nMarkerSets);

    for (i = 0; i < nMarkerSets; i++) {
      // Markerset name
      char szName[256];
      strcpy(szName, ptr);
      int nDataBytes = (int) strlen(szName) + 1;
      ptr += nDataBytes;
      printf_natnet("Model Name: %s\n", szName);

      // marker data
      int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
      printf_natnet("Marker Count : %d\n", nMarkers);

      for (j = 0; j < nMarkers; j++) {
        float x = 0; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0; memcpy(&z, ptr, 4); ptr += 4;
        printf_natnet("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
      }
    }

    // Unidentified markers
    int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
    printf_natnet("Unidentified Marker Count : %d\n", nOtherMarkers);
    for (j = 0; j < nOtherMarkers; j++) {
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      printf_natnet("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, x, y, z);
    }

    // ========== RIGID BODIES ==========
    // Rigid bodies
    int nrigid_bodies = 0;
    memcpy(&nrigid_bodies, ptr, 4); ptr += 4;
    printf_natnet("Rigid Body Count : %d\n", nrigid_bodies);

    // Check if there ie enough space for the rigid bodies
    if (nrigid_bodies > MAX_rigid_bodies) {
      fprintf(stderr,
              "Could not sample all the rigid bodies because the amount of rigid bodies is bigger then %d (MAX_rigid_bodies).\r\n",
              MAX_rigid_bodies);
      exit(EXIT_FAILURE);
    }

    for (j = 0; j < nrigid_bodies; j++) {
      // Store the rigid body position and orientation in the RigidBody struct
      struct RigidBody prev_rigid_body;
      memcpy(&prev_rigid_body, &rigid_bodies[j], sizeof(struct RigidBody));

      memcpy(&rigid_bodies[j].id, ptr, 4); ptr += 4;
      memcpy(&rigid_bodies[j].opti_coord.x, ptr, 4); ptr += 4;  
      memcpy(&rigid_bodies[j].opti_coord.y, ptr, 4); ptr += 4;   
      memcpy(&rigid_bodies[j].opti_coord.z, ptr, 4); ptr += 4;   
      memcpy(&rigid_bodies[j].opti_orient.qx, ptr, 4); ptr += 4;
      memcpy(&rigid_bodies[j].opti_orient.qy, ptr, 4); ptr += 4;  
      memcpy(&rigid_bodies[j].opti_orient.qz, ptr, 4); ptr += 4;
      memcpy(&rigid_bodies[j].opti_orient.qi, ptr, 4); ptr += 4;
      printf_natnet("ID (%d) : %d\n", j, rigid_bodies[j].id);
      printf_natnet("pos: [%3.2f,%3.2f,%3.2f]\n", rigid_bodies[j].opti_coord.x, rigid_bodies[j].opti_coord.y, rigid_bodies[j].opti_coord.z);
      printf_natnet("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", rigid_bodies[j].opti_orient.qx, rigid_bodies[j].opti_orient.qy, rigid_bodies[j].opti_orient.qz,
                    rigid_bodies[j].opti_orient.qi); 

      // Rotate the position from the Optitrack reference frame to the local reference frame
      float_quat_vmult(&rigid_bodies[j].local_coord, &local_ref, &rigid_bodies[j].opti_coord);

      // Define the rotation in the local reference frame
      struct FloatQuat temp_quat;
      float_quat_comp(&temp_quat, &rigid_bodies[j].opti_orient, &local_ref);
      float_quat_inv_comp_norm_shortest(&rigid_bodies[j].local_orient, &local_ref, &temp_quat);

      // Differentiate the position to get the speed
      rigid_bodies[j].totalVelocitySamples++;
      if (prev_rigid_body.local_coord.x != rigid_bodies[j].local_coord.x || // do a member-wise comparison because of potential padding bytes
          prev_rigid_body.local_coord.y != rigid_bodies[j].local_coord.y || 
          prev_rigid_body.local_coord.z != rigid_bodies[j].local_coord.z || 
          prev_rigid_body.local_orient.qi != rigid_bodies[j].local_orient.qi || 
          prev_rigid_body.local_orient.qx != rigid_bodies[j].local_orient.qx || 
          prev_rigid_body.local_orient.qy != rigid_bodies[j].local_orient.qy || 
          prev_rigid_body.local_orient.qz != rigid_bodies[j].local_orient.qz) {

        if (prev_rigid_body.posSampled) {
          rigid_bodies[j].local_vel.x += (rigid_bodies[j].local_coord.x - prev_rigid_body.local_coord.x);
          rigid_bodies[j].local_vel.y += (rigid_bodies[j].local_coord.y - prev_rigid_body.local_coord.y);
          rigid_bodies[j].local_vel.z += (rigid_bodies[j].local_coord.z - prev_rigid_body.local_coord.z);
          rigid_bodies[j].nVelocitySamples++;
        }

        rigid_bodies[j].nSamples++;
        rigid_bodies[j].posSampled = TRUE; // to ensure the same position is not send twice
      } else {
        rigid_bodies[j].posSampled = FALSE;
      }

      // When marker id changed, reset the velocity
      if (prev_rigid_body.id != rigid_bodies[j].id) {
        rigid_bodies[j].local_vel.x = 0;
        rigid_bodies[j].local_vel.x = 0;
        rigid_bodies[j].local_vel.x = 0;
        rigid_bodies[j].nSamples = 0;
        rigid_bodies[j].nVelocitySamples = 0;
        rigid_bodies[j].totalVelocitySamples = 0;
        rigid_bodies[j].posSampled = FALSE;
      }

      // Associated marker positions
      memcpy(&rigid_bodies[j].nMarkers, ptr, 4); ptr += 4;
      printf_natnet("Marker Count: %d\n", rigid_bodies[j].nMarkers);
      int nBytes = rigid_bodies[j].nMarkers * 3 * sizeof(float);
      float *markerData = (float *)malloc(nBytes);
      memcpy(markerData, ptr, nBytes);
      ptr += nBytes;

      if (natnet_major >= 2) {
        // Associated marker IDs
        nBytes = rigid_bodies[j].nMarkers * sizeof(int);
        int *markerIDs = (int *)malloc(nBytes);
        memcpy(markerIDs, ptr, nBytes);
        ptr += nBytes;

        // Associated marker sizes
        nBytes = rigid_bodies[j].nMarkers * sizeof(float);
        float *markerSizes = (float *)malloc(nBytes);
        memcpy(markerSizes, ptr, nBytes);
        ptr += nBytes;

        for (k = 0; k < rigid_bodies[j].nMarkers; k++) {
          printf_natnet("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k],
                        markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
        }

        if (markerIDs) {
          free(markerIDs);
        }
        if (markerSizes) {
          free(markerSizes);
        }

      } else {
        for (k = 0; k < rigid_bodies[j].nMarkers; k++) {
          printf_natnet("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k * 3], markerData[k * 3 + 1],
                        markerData[k * 3 + 2]);
        }
      }
      if (markerData) {
        free(markerData);
      }

      if (natnet_major >= 2) {
        // Mean marker error
        memcpy(&rigid_bodies[j].error, ptr, 4); ptr += 4;
        printf_natnet("Mean marker error: %3.8f\n", rigid_bodies[j].error);
      }

      // 2.6 and later
      if (((natnet_major == 2) && (natnet_minor >= 6)) || (natnet_major > 2) || (natnet_major == 0)) {
        // params
        short params = 0; memcpy(&params, ptr, 2); ptr += 2;
//           bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
      }
    } // next rigid body

    // ========== SKELETONS ==========
    // Skeletons (version 2.1 and later)
    if (((natnet_major == 2) && (natnet_minor > 0)) || (natnet_major > 2)) {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4); ptr += 4;
      printf_natnet("Skeleton Count : %d\n", nSkeletons);
      for (j = 0; j < nSkeletons; j++) {
        // Skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4); ptr += 4;
        // # of rigid bodies (bones) in skeleton
        int nrigid_bodies = 0;
        memcpy(&nrigid_bodies, ptr, 4); ptr += 4;
        printf_natnet("Rigid Body Count : %d\n", nrigid_bodies);
        for (j = 0; j < nrigid_bodies; j++) {
          // Rigid body pos/ori
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
          float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
          float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
          float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
          printf_natnet("ID : %d\n", ID);
          printf_natnet("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
          printf_natnet("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

          // Sssociated marker positions
          int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
          printf_natnet("Marker Count: %d\n", nRigidMarkers);
          int nBytes = nRigidMarkers * 3 * sizeof(float);
          float *markerData = (float *)malloc(nBytes);
          memcpy(markerData, ptr, nBytes);
          ptr += nBytes;

          // Associated marker IDs
          nBytes = nRigidMarkers * sizeof(int);
          int *markerIDs = (int *)malloc(nBytes);
          memcpy(markerIDs, ptr, nBytes);
          ptr += nBytes;

          // Associated marker sizes
          nBytes = nRigidMarkers * sizeof(float);
          float *markerSizes = (float *)malloc(nBytes);
          memcpy(markerSizes, ptr, nBytes);
          ptr += nBytes;

          for (k = 0; k < nRigidMarkers; k++) {
            printf_natnet("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k],
                          markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
          }

          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          printf_natnet("Mean marker error: %3.2f\n", fError);

          // Release resources
          if (markerIDs) {
            free(markerIDs);
          }
          if (markerSizes) {
            free(markerSizes);
          }
          if (markerData) {
            free(markerData);
          }
        } // next rigid body
      } // next skeleton
    }

    // ========== LABELED MARKERS ==========
    // Labeled markers (version 2.3 and later)
    if (((natnet_major == 2) && (natnet_minor >= 3)) || (natnet_major > 2)) {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
      printf_natnet("Labeled Marker Count : %d\n", nLabeledMarkers);
      for (j = 0; j < nLabeledMarkers; j++) {
        int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
        float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
        float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

        printf_natnet("ID  : %d\n", ID);
        printf_natnet("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        printf_natnet("size: [%3.2f]\n", size);
      }
    }

    // Latency
    natnet_latency = 0.0f; memcpy(&natnet_latency, ptr, 4); ptr += 4;
    printf_natnet("latency : %3.3f\n", natnet_latency);

    // Timecode
    unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);  ptr += 4;
    unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
    printf_natnet("timecode : %d %d", timecode, timecodeSub);

    // End of data tag
    int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
    printf_natnet("End Packet\n-------------\n");
  } else {
    printf("Error: Unrecognized packet type from Optitrack NatNet.\n");
  }
}

/** The transmitter periodic function */
gboolean timeout_transmit_callback(gpointer data)
{
  int i;

  // Loop trough all the available rigid_bodies (TODO: optimize)
  for (i = 0; i < MAX_rigid_bodies; i++) {
    // Check if ID's are correct
    if (rigid_bodies[i].id >= MAX_rigid_bodies) {
      fprintf(stderr,
              "Could not parse rigid body %d from NatNet, because ID is higher then or equal to %d (MAX_rigid_bodies-1).\r\n",
              rigid_bodies[i].id, MAX_rigid_bodies - 1);
      exit(EXIT_FAILURE);
    }

    // Check if we want to transmit (follow) this rigid
    if (aircrafts[rigid_bodies[i].id].ac_id == 0) {
      continue;
    }

    // When we don track anymore and timeout or start tracking
    if (rigid_bodies[i].nSamples < 1
        && aircrafts[rigid_bodies[i].id].connected
        && (natnet_latency - aircrafts[rigid_bodies[i].id].lastSample) > CONNECTION_TIMEOUT) {
      aircrafts[rigid_bodies[i].id].connected = FALSE;
      fprintf(stderr, "#error Lost tracking rigid id %d, aircraft id %d.\n",
              rigid_bodies[i].id, aircrafts[rigid_bodies[i].id].ac_id);
    } else if (rigid_bodies[i].nSamples > 0 && !aircrafts[rigid_bodies[i].id].connected) {
      fprintf(stderr, "#pragma message: Now tracking rigid id %d, aircraft id %d.\n",
              rigid_bodies[i].id, aircrafts[rigid_bodies[i].id].ac_id);
    }

    // Check if we still track the rigid
    if (rigid_bodies[i].nSamples < 1) {
      continue;
    }

    // Update the last tracked
    aircrafts[rigid_bodies[i].id].connected = TRUE;
    aircrafts[rigid_bodies[i].id].lastSample = natnet_latency;

    /* 
     * THESE FUNCTIONS WERE USED FOR TESTING AND SHOULD BE REMOVED ACCORDINGLY
     */

    // printf("pos: [%3.2f,%3.2f,%3.2f]\n", rigid_bodies[i].opti_coord.x, rigid_bodies[i].opti_coord.y, rigid_bodies[i].opti_coord.z);

    // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", rigid_bodies[i].qx, rigid_bodies[i].qy, rigid_bodies[i].qz,
                    // rigid_bodies[i].qw);

    /* 
     * Define the rotation matrix that rotates the Optitrack frame to the local 
     * FRD (forward-right-down) reference frame through a conventional 3-2-1 
     * rotation
     */
    // struct FloatRMat r_mat; 
    // struct FloatEulers rotation_eulers;
    // rotation_eulers.phi = 90. / 180. * M_PI; // 90 degree rotation around the x-axis
    // rotation_eulers.theta = 0;
    // rotation_eulers.psi = 0;
    // float_rmat_of_eulers_321(&r_mat, &rotation_eulers);

    // Define the position in local coordinates
    // struct FloatVect3 local_coord;
    // float_rmat_vmult(&local_coord, &r_mat, &rigid_bodies[i].opti_coord);

    // printf("pos: [%3.2f,%3.2f,%3.2f] -> [%3.2f,%3.2f,%3.2f]\n", 
      // rigid_bodies[i].opti_coord.x, rigid_bodies[i].opti_coord.y, rigid_bodies[i].opti_coord.z, 
      // local_coord.x, local_coord.y, local_coord.z);

    // Define the the orientation in the local coordinate frame
    // struct FloatQuat rotation_quat;
    

    // Rotate the rigid body rotations to the local coordinate frame
    // struct FloatEulers temp_eulers;
    // float_eulers_of_quat(&temp_eulers, &rigid_bodies[i].opti_orient);
    // printf("temp: [%3.2f,%3.2f,%3.2f]\n", 
      // DegOfRad(temp_eulers.phi), DegOfRad(temp_eulers.theta), DegOfRad(temp_eulers.psi));

    // struct FloatQuat temp_quat;
    // float_quat_comp(&temp_quat, &rigid_bodies[i].opti_orient, &rotation_quat);
    // float_quat_inv_comp(&orient, &rotation_quat, &temp_quat);

    // Compute the Euler angles accordingly
    // float_eulers_of_quat(&orient_eulers, &orient);

    // printf("orient: [%3.2f,%3.2f,%3.2f,%3.2f] (%3.2f,%3.2f,%3.2f)\n", 
      // orient.qi, orient.qx, orient.qy, orient.qz,
      // DegOfRad(orient_eulers.phi), DegOfRad(orient_eulers.theta), DegOfRad(orient_eulers.psi));
    /*
     * END
     */

    // Rotote the local position to the desired orientation of the tracking system
    float_quat_vmult(&rigid_bodies[i].tracking_coord, &tracking_ref_quat, &rigid_bodies[i].local_coord);

    // Convert the position to ecef and lla based on the Optitrack LTP
    struct EcefCoor_f ecef_pos;
    ecef_of_ned_point_f(&ecef_pos , &tracking_ltp , (struct NedCoor_f *)&rigid_bodies[i].tracking_coord);
    struct LlaCoor_f lla_pos;
    lla_of_ecef_f(&lla_pos, &ecef_pos);

    // Check if we have enough samples to estimate the velocity
    rigid_bodies[i].nVelocityTransmit++;
    if (rigid_bodies[i].nVelocitySamples >= min_velocity_samples) {
      // Calculate the derevative of the sum to get the correct velocity     (1 / freq_transmit) * (samples / total_samples)
      double sample_time = //((double)rigid_bodies[i].nVelocitySamples / (double)rigid_bodies[i].totalVelocitySamples) /
        ((double)rigid_bodies[i].nVelocityTransmit / (double)freq_transmit);
      rigid_bodies[i].local_vel.x = rigid_bodies[i].local_vel.x / sample_time;
      rigid_bodies[i].local_vel.y = rigid_bodies[i].local_vel.y / sample_time;
      rigid_bodies[i].local_vel.z = rigid_bodies[i].local_vel.z / sample_time;

      // Add the Optitrack angle to the x and y velocities
      float_quat_vmult(&rigid_bodies[i].tracking_vel, &tracking_ref_quat, &rigid_bodies[i].local_vel);

      // Convert the speed to ecef based on the Optitrack LTP
      ecef_of_ned_vect_f(&rigid_bodies[i].ecef_vel, &tracking_ltp , (struct NedCoor_f *)&rigid_bodies[i].tracking_vel);
    }

    // Rotate the orientation from the local to the tracking reference frame
    struct FloatQuat temp_quat;
    float_quat_comp(&temp_quat, &rigid_bodies[i].local_orient, &tracking_ref_quat);
    float_quat_inv_comp_norm_shortest(&rigid_bodies[i].tracking_orient, &tracking_ref_quat, &temp_quat);

    struct FloatEulers local_eulers;
    float_eulers_of_quat(&local_eulers, &rigid_bodies[i].local_orient);
    printf("Local Eulers: (%.2f, %.2f, %.2f)\n", local_eulers.phi, local_eulers.theta, local_eulers.psi);

    // Calculate the heading by adding the Natnet offset angle and normalizing it
    float heading = 0;//-orient_eulers.psi + 90.0 / 57.6 -
                     //(float)tracking_offset_angle; //the optitrack axes are 90 degrees rotated wrt ENU
    NormRadAngle(heading);

    printf_debug("[%d -> %d]Samples: %d\t%d\t\tTiming: %3.3f latency\n", rigid_bodies[i].id,
                 aircrafts[rigid_bodies[i].id].ac_id
                 , rigid_bodies[i].nSamples, rigid_bodies[i].nVelocitySamples, natnet_latency);
    printf_debug("    Heading: %f\t\tPosition: %f\t%f\t%f\t\tVelocity: %f\t%f\t%f\n", DegOfRad(heading),
                 rigid_bodies[i].tracking_coord.x, rigid_bodies[i].tracking_coord.y, rigid_bodies[i].tracking_coord.z,
                 rigid_bodies[i].ecef_vel.x, rigid_bodies[i].ecef_vel.y, rigid_bodies[i].ecef_vel.z);

    // Transmit the REMOTE_GPS packet on the ivy bus (either small or big)
    if (small_packets) {
      /* The position in the local reference frame is a float and the 11 LSBs of the (signed) x and y axis are 
       * compressed into a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
       * used.
       */

      uint32_t pos_xyz = 0;
      // check if position within limits
      if (fabs(rigid_bodies[i].tracking_coord.x * 100.) < pow(2, 10)) {
        pos_xyz = (((uint32_t)(rigid_bodies[i].tracking_coord.x * 100.0)) & 0x7FF) << 21; // bits 31-21 x position in cm
      } else {
        fprintf(stderr, "Warning!! X position out of maximum range of small message (±%.2fm): %.2f\n", pow(2, 10) / 100, rigid_bodies[i].tracking_coord.x);
        pos_xyz = (((uint32_t)(pow(2, 10) * rigid_bodies[i].tracking_coord.x / fabs(rigid_bodies[i].tracking_coord.x))) & 0x7FF) << 21; // bits 31-21 x position in cm
      }

      if (fabs(rigid_bodies[i].tracking_coord.y * 100.) < pow(2, 10)) {
        pos_xyz |= (((uint32_t)(rigid_bodies[i].tracking_coord.y * 100.0)) & 0x7FF) << 10; // bits 20-10 y position in cm
      } else {
        fprintf(stderr, "Warning!! Y position out of maximum range of small message (±%.2fm): %.2f\n", pow(2, 10) / 100, rigid_bodies[i].tracking_coord.y);
        pos_xyz |= (((uint32_t)(pow(2, 10) * rigid_bodies[i].tracking_coord.y / fabs(rigid_bodies[i].tracking_coord.y))) & 0x7FF) << 10; // bits 20-10 y position in cm
      }

      if (rigid_bodies[i].tracking_coord.z * 100. < pow(2, 9) && rigid_bodies[i].tracking_coord.z > 0.) {
        pos_xyz |= (((uint32_t)(fabs(rigid_bodies[i].tracking_coord.z) * 100.0)) & 0x3FF); // bits 9-0 z position in cm
      } else if (rigid_bodies[i].tracking_coord.z > 0.) {
        fprintf(stderr, "Warning!! Z position out of maximum range of small message (%.2fm): %.2f\n", pow(2, 9) / 100, rigid_bodies[i].tracking_coord.z);
        pos_xyz |= (((uint32_t)(pow(2, 9))) & 0x3FF);                              // bits 9-0 z position in cm
      }
      // printf("NED Pos: %u (%.2f, %.2f, %.2f)\n", pos_xyz, pos.x, pos.y, pos.z);

      /* The speed is a float and the 11 LSBs of the x and y axis and 10 LSBs of z (all signed) are compressed into
       * a single integer.
       */
      uint32_t speed_xyz = 0;
      // check if speed within limits
      if (fabs(rigid_bodies[i].tracking_vel.x * 100) < pow(2, 10)) {
        speed_xyz = (((uint32_t)(rigid_bodies[i].tracking_vel.x * 100.0)) & 0x7FF) << 21; // bits 31-21 speed x in cm/s
      } else {
        fprintf(stderr, "Warning!! X Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 10) / 100, rigid_bodies[i].tracking_vel.x);
        speed_xyz = (((uint32_t)(pow(2, 10) * rigid_bodies[i].tracking_vel.x / fabs(rigid_bodies[i].tracking_vel.x))) & 0x7FF) << 21; // bits 31-21 speed x in cm/s
      }

      if (fabs(rigid_bodies[i].tracking_vel.y * 100) < pow(2, 10)) {
        speed_xyz |= (((uint32_t)(rigid_bodies[i].tracking_vel.y * 100.0)) & 0x7FF) << 10; // bits 20-10 speed y in cm/s
      } else {
        fprintf(stderr, "Warning!! Y Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 10) / 100, rigid_bodies[i].tracking_vel.y);
        speed_xyz |= (((uint32_t)(pow(2, 10) * rigid_bodies[i].tracking_vel.y / fabs(rigid_bodies[i].tracking_vel.y))) & 0x7FF) << 10; // bits 20-10 speed y in cm/s
      }

      if (fabs(rigid_bodies[i].tracking_vel.z * 100) < pow(2, 9)) {
        speed_xyz |= (((uint32_t)(rigid_bodies[i].tracking_vel.z * 100.0)) & 0x3FF); // bits 9-0 speed z in cm/s
      } else {
        fprintf(stderr, "Warning!! Z Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 9) / 100, rigid_bodies[i].tracking_vel.z);
        speed_xyz |= (((uint32_t)(pow(2, 9) * rigid_bodies[i].tracking_vel.z / fabs(rigid_bodies[i].tracking_vel.z))) & 0x3FF);       // bits 9-0 speed z in cm/s
      }
      // printf("NED Vel: %u (%.2f, %.2f, %.2f)\n", rigid_bodies[i].tracking_vel.x, rigid_bodies[i].tracking_vel.y, rigid_bodies[i].tracking_vel.z);

      // Transform the tracking orientation quaternion to Eulers
      struct FloatEulers tracking_orient_eulers;
      float_eulers_of_quat(&tracking_orient_eulers, &rigid_bodies[i].tracking_orient);

      /* The Euler angles of the body with respect to the tracking reference frames are floats and the 11 LSBs of the
       * roll and pitch angles and 10 LSBs of the heading are compressed into a single integer.
       */
      uint32_t eulers;
      // check if eulers within limits
      if (fabs(tracking_orient_eulers.phi * 100) < pow(2, 10)) {
        eulers = (((uint32_t)(tracking_orient_eulers.phi * 100.0)) & 0x7FF) << 21; // bits 31-21 roll angle in rad
      } else {
        fprintf(stderr, "Warning!! X Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 10) / 100, tracking_orient_eulers.phi);
        eulers = (((uint32_t)(pow(2, 10) * tracking_orient_eulers.phi / fabs(tracking_orient_eulers.phi))) & 0x7FF) << 21;  // bits 31-21 roll angle in rad
      }

      if (fabs(tracking_orient_eulers.theta * 100) < pow(2, 10)) {
        eulers = (((uint32_t)(tracking_orient_eulers.theta * 100.0)) & 0x7FF) << 10; // bits 31-21 roll angle in rad
      } else {
        fprintf(stderr, "Warning!! X Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 10) / 100, tracking_orient_eulers.theta);
        eulers = (((uint32_t)(pow(2, 10) * tracking_orient_eulers.theta / fabs(tracking_orient_eulers.theta))) & 0x7FF) << 10;  // bits 31-21 roll angle in rad
      }

      if (fabs(tracking_orient_eulers.psi * 100) < pow(2, 9)) {
        eulers = (((uint32_t)(tracking_orient_eulers.psi * 100.0)) & 0x3FF); // bits 31-21 roll angle in rad
      } else {
        fprintf(stderr, "Warning!! X Speed out of maximum range of small message (±%.2fm/s): %.2f\n", pow(2, 9) / 100, tracking_orient_eulers.psi);
        eulers = (((uint32_t)(pow(2, 9) * tracking_orient_eulers.psi / fabs(tracking_orient_eulers.psi))) & 0x3FF);  // bits 31-21 roll angle in rad
      }
      printf("Eulers: %u (%.2f, %.2f, %.2f)\n", eulers, tracking_orient_eulers.phi, tracking_orient_eulers.theta, tracking_orient_eulers.psi);

      // Send a 13 byte message
      IvySendMsg("0 REMOTE_GPS_SMALL %d %d %d %d", aircrafts[rigid_bodies[i].id].ac_id, // uint8 rigid body ID (1 byte)
                 pos_xyz,                           // uint32 NED X, Y and Z in CM (4 bytes)
                 speed_xyz,                         // uint32 NED velocity X, Y, Z in cm/s (4 bytes)
                 eulers);                           // uint32 Euler angles rad*1e2 (4 bytes)
    } else {
      IvySendMsg("0 REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d", aircrafts[rigid_bodies[i].id].ac_id,
                 rigid_bodies[i].nMarkers,                //uint8 Number of markers (sv_num)
                 (int)(ecef_pos.x * 100.0),              //int32 ECEF X in CM
                 (int)(ecef_pos.y * 100.0),              //int32 ECEF Y in CM
                 (int)(ecef_pos.z * 100.0),              //int32 ECEF Z in CM
                 (int)(lla_pos.lat * 10000000.0),        //int32 LLA latitude in rad*1e7
                 (int)(lla_pos.lon * 10000000.0),        //int32 LLA longitude in rad*1e7
                 (int)(rigid_bodies[i].opti_coord.z * 1000.0),       //int32 LLA altitude in mm above elipsoid
                 (int)(rigid_bodies[i].opti_coord.z * 1000.0),       //int32 HMSL height above mean sea level in mm
                 (int)(rigid_bodies[i].ecef_vel.x * 100.0), //int32 ECEF velocity X in m/s
                 (int)(rigid_bodies[i].ecef_vel.y * 100.0), //int32 ECEF velocity Y in m/s
                 (int)(rigid_bodies[i].ecef_vel.z * 100.0), //int32 ECEF velocity Z in m/s
                 0,
                 (int)(heading * 10000000.0));           //int32 Course in rad*1e7
    }

    // Reset the velocity differentiator if we calculated the velocity
    if (rigid_bodies[i].nVelocitySamples >= min_velocity_samples) {
      rigid_bodies[i].local_vel.x = 0;
      rigid_bodies[i].local_vel.y = 0;
      rigid_bodies[i].local_vel.z = 0;
      rigid_bodies[i].nVelocitySamples = 0;
      rigid_bodies[i].totalVelocitySamples = 0;
      rigid_bodies[i].nVelocityTransmit = 0;
    }

    rigid_bodies[i].nSamples = 0;
  }

  return TRUE;
}

/** The NatNet sampler periodic function */
static gboolean sample_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
  static unsigned char buffer_data[MAX_PACKETSIZE];
  static int bytes_data = 0;

  // Keep on reading until we have the whole packet
  bytes_data += udp_socket_recv(&natnet_data, buffer_data, MAX_PACKETSIZE);

  // Parse NatNet data
  if (bytes_data >= 2 && bytes_data >= buffer_data[1]) {
    natnet_parse(buffer_data);
    bytes_data = 0;
  }

  return TRUE;
}


/** Print the program help */
void print_help(char *filename)
{
  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h, --help                  Display this help\n"
    "   -v, --verbose <level>       Verbosity level 0-2 (0)\n\n"

    "   -ac <rigid_id> <ac_id>      Use rigid ID for GPS of ac_id (multiple possible)\n\n"

    "   -multicast_addr <ip>        NatNet server multicast address (239.255.42.99)\n"
    "   -server <ip>                NatNet server IP address (255.255.255.255)\n"
    "   -version <id>               NatNet server version (2.5)\n"
    "   -data_port <port>           NatNet server data socket UDP port (1510)\n"
    "   -cmd_port <port>            NatNet server command socket UDP port (1511)\n\n"

    "   -ecef <x> <y> <z>           ECEF coordinates of the tracking system\n"
    "   -lla <lat> <lon> <alt>      Latitude, longitude and altitude of the tracking system\n"
    "   -quat <qi> <qx> <qy> <gz>   Orientation of the tracking system w.r.t. the local reference frame\n"
    "   -eulers <phi> <theta> <psi> Orientation of the tracking system w.r.t. the local reference frame\n\n"

    "   -tf <freq>                  Transmit frequency to the ivy bus in hertz (60)\n"
    "   -vel_samples <samples>      Minimum amount of samples for the velocity differentiator (4)\n"
    "   -small                      Send small packets instead of bigger (FALSE)\n\n"

    "   -ivy_bus <address:port>     Ivy bus address and port (127.255.255.255:2010)\n";
  fprintf(stderr, usage, filename);
}

/** Check the amount of arguments */
void check_argcount(int argc, char **argv, int i, int expected)
{
  if (i + expected >= argc) {
    fprintf(stderr, "Option %s expected %d arguments\r\n\r\n", argv[i], expected);
    print_help(argv[0]);
    exit(0);
  }
}

/** Parse the options from the commandline */
static void parse_options(int argc, char **argv)
{
  int i, count_ac = 0;
  for (i = 1; i < argc; ++i) {

    // Print help
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help(argv[0]);
      exit(0);
    }
    // Set the verbosity level
    if (strcmp(argv[i], "--verbosity") == 0 || strcmp(argv[i], "-v") == 0) {
      check_argcount(argc, argv, i, 1);

      verbose = atoi(argv[++i]);
    }

    // Set an rigid body to ivy ac_id
    else if (strcmp(argv[i], "-ac") == 0) {
      check_argcount(argc, argv, i, 2);

      int rigid_id = atoi(argv[++i]);
      uint8_t ac_id = atoi(argv[++i]);

      if (rigid_id >= MAX_rigid_bodies) {
        fprintf(stderr, "Rigid body ID must be less then %d (MAX_rigid_bodies)\n\n", MAX_rigid_bodies);
        print_help(argv[0]);
        exit(EXIT_FAILURE);
      }
      aircrafts[rigid_id].ac_id = ac_id;
      count_ac++;
    }

    // Set the NatNet multicast address
    else if (strcmp(argv[i], "-multicast_addr") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_multicast_addr = argv[++i];
    }
    // Set the NatNet server ip address
    else if (strcmp(argv[i], "-server") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_addr = argv[++i];
    }
    // Set the NatNet server version
    else if (strcmp(argv[i], "-version") == 0) {
      check_argcount(argc, argv, i, 1);

      float version = atof(argv[++i]);
      natnet_major = (uint8_t)version;
      natnet_minor = (uint8_t)(version * 10.0) % 10;
    }
    // Set the NatNet server data port
    else if (strcmp(argv[i], "-data_port") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_data_port = atoi(argv[++i]);
    }
    // Set the NatNet server command port
    else if (strcmp(argv[i], "-cmd_port") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_cmd_port = atoi(argv[++i]);
    }

    // Set the Tracking system position in ECEF
    else if (strcmp(argv[i], "-ecef") == 0) {
      check_argcount(argc, argv, i, 3);

      struct EcefCoor_f tracking_ecef;
      tracking_ecef.x  = atof(argv[++i]);
      tracking_ecef.y  = atof(argv[++i]);
      tracking_ecef.z  = atof(argv[++i]);
      ltp_def_from_ecef_f(&tracking_ltp, &tracking_ecef);
    }
    // Set the tracking system position in LLA
    else if (strcmp(argv[i], "-lla") == 0) {
      check_argcount(argc, argv, i, 3);

      struct EcefCoor_d tracking_ecef_d;
      struct LlaCoor_d tracking_lla_d;
      tracking_lla_d.lat  = atof(argv[++i]);
      tracking_lla_d.lon  = atof(argv[++i]);
      tracking_lla_d.alt  = atof(argv[++i]);
      ecef_of_lla_d(&tracking_ecef_d, &tracking_lla_d);

      // Casting from float to double will cause a loss of precision
      struct EcefCoor_f tracking_ecef_f;
      tracking_ecef_f.x = (float)tracking_ecef_d.x;
      tracking_ecef_f.y = (float)tracking_ecef_d.y;
      tracking_ecef_f.z = (float)tracking_ecef_d.z;
      ltp_def_from_ecef_f(&tracking_ltp, &tracking_ecef_f);
    }
    // Set the orientation of the tracking system as a quaternion
    else if (strcmp(argv[i], "-quat") == 0) {
      check_argcount(argc, argv, i, 4);

      tracking_ref_quat.qi = atof(argv[++i]);
      tracking_ref_quat.qx = atof(argv[++i]);
      tracking_ref_quat.qy = atof(argv[++i]);
      tracking_ref_quat.qz = atof(argv[++i]);
    }
    // Set the orientation of the tracking system in Euler angles
    else if (strcmp(argv[i], "-eulers") == 0) {
      check_argcount(argc, argv, i, 4);

      tracking_ref_eulers.phi = RadOfDeg(atof(argv[++i]));
      tracking_ref_eulers.theta = RadOfDeg(atof(argv[++i]));
      tracking_ref_eulers.psi = RadOfDeg(atof(argv[++i]));

      // Compute the orientation of the tracking system as a quaternion
      float_quat_of_eulers(&tracking_ref_quat, &tracking_ref_eulers);
    }

    // Set the transmit frequency
    else if (strcmp(argv[i], "-tf") == 0) {
      check_argcount(argc, argv, i, 1);

      freq_transmit = atoi(argv[++i]);
    }
    // Set the minimum amount of velocity samples for the differentiator
    else if (strcmp(argv[i], "-vel_samples") == 0) {
      check_argcount(argc, argv, i, 1);

      min_velocity_samples = atoi(argv[++i]);
    }
    // Set to use small packets
    else if (strcmp(argv[i], "-small") == 0) {
      small_packets = TRUE;
    }

    // Set the ivy bus
    else if (strcmp(argv[i], "-ivy_bus") == 0) {
      check_argcount(argc, argv, i, 1);

      ivy_bus = argv[++i];
    }

    // Unknown option
    else {
      fprintf(stderr, "Unknown option %s\r\n\r\n", argv[i]);
      print_help(argv[0]);
      exit(0);
    }
  }

  // Check if at least one aircraft is set
  if (count_ac < 1) {
    fprintf(stderr, "You must specify at least one aircraft (-ac <rigid_id> <ac_id>)\n\n");
    print_help(argv[0]);
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char **argv)
{
  // Set the default tracking system ECEF coordinates
  struct EcefCoor_f tracking_ecef;
  tracking_ecef.x = 3924332;
  tracking_ecef.y = 300362;
  tracking_ecef.z = 5002197;
  ltp_def_from_ecef_f(&tracking_ltp, &tracking_ecef);

  // Set the default rotation to rotate the Optitrack axis system to the local reference frame
  struct FloatEulers local_eulers;
  local_eulers.phi = 90. / 180. * M_PI;
  local_eulers.theta = 0.;
  local_eulers.psi = 0.;
  float_quat_of_eulers(&local_ref, &local_eulers);

  // Set the default rotation from the local reference frame to the NED reference frame
  tracking_ref_eulers.phi = 0.;
  tracking_ref_eulers.theta = 0.;
  tracking_ref_eulers.psi = 33. / 180. * M_PI;
  float_quat_of_eulers(&tracking_ref_quat, &tracking_ref_eulers);

  // Parse the options from cmdline
  parse_options(argc, argv);
  printf_debug("Tracking system Latitude: %f Longitude: %f Offset to North: %f degrees\n", DegOfRad(tracking_ltp.lla.lat),
               DegOfRad(tracking_ltp.lla.lon), DegOfRad(tracking_ref_eulers.psi));

  // Create the network connections
  printf_debug("Starting NatNet listening (multicast address: %s, data port: %d, version: %d.%d)\n",
               natnet_multicast_addr, natnet_data_port, natnet_major, natnet_minor);
  udp_socket_create(&natnet_data, "", -1, natnet_data_port, 0); // Only receiving
  udp_socket_subscribe_multicast(&natnet_data, natnet_multicast_addr);
  udp_socket_set_recvbuf(&natnet_data, 0x100000); // 1MB

  printf_debug("Starting NatNet command socket (server address: %s, command port: %d)\n", natnet_addr, natnet_cmd_port);
  udp_socket_create(&natnet_cmd, natnet_addr, natnet_cmd_port, 0, 1);
  udp_socket_set_recvbuf(&natnet_cmd, 0x100000); // 1MB

  // Create the Ivy Client
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit("natnet2ivy", "natnet2ivy READY", 0, 0, 0, 0);
  IvyStart(ivy_bus);

  // Create the main timers
  printf_debug("Starting transmitting and sampling timeouts (transmitting frequency: %dHz, minimum velocity samples: %d)\n",
               freq_transmit, min_velocity_samples);
  g_timeout_add(1000 / freq_transmit, timeout_transmit_callback, NULL);

  GIOChannel *sk = g_io_channel_unix_new(natnet_data.sockfd);
  g_io_add_watch(sk, G_IO_IN | G_IO_NVAL | G_IO_HUP,
                 sample_data, NULL);

  // Run the main loop
  g_main_loop_run(ml);

  return 0;
}
