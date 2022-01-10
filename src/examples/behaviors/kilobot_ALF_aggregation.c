#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define COLLISION_BITS 8
#define SECTORS_IN_COLLISION 2
#define TICKS_TO_SEC 32
#define NUM_ROBOTS 20
#define REFRESH_RATE 10
#define NEIGHBOUR_THRESHOLD 3

typedef enum
{ // Enum for different motion types
  TURN_LEFT = 1,
  TURN_RIGHT = 2,
  STOP = 3,
  FORWARD = 4,
} motion_t;

typedef enum
{ // Enum for boolean flags
  false = 0,
  true = 1,
} bool;

typedef enum
{ // Enum for boolean flags
  BROWNIAN = 0,
  PERSISTENT = 1,
} adaptive_walk;

typedef enum
{ // Enum for boolean flags
  ARK_MSG_TYPE = 0,
  KILO_MSG_TYPE = 1,
} msg_type;

typedef enum
{ // store the previous freespace for wall avoidance
  LEFT = 1,
  RIGHT = 2,
} Free_space;

motion_t current_motion_type = STOP; // Current motion type

/***********WALK PARAMETERS***********/
adaptive_walk current_walk = BROWNIAN; // start with a meaningless value
const float std_motion_steps = 5 * 16; // variance of the gaussian used to compute forward motion
float levy_exponent = 1.4;             // 2 is brownian like motion (alpha)
float crw_exponent = 0.9;              // higher more straight (rho)
uint32_t turning_ticks = 0;            // keep count of ticks of turning
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0;       // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

/***********WALL AVOIDANCE***********/
// the kb is "equipped" with a proximity sensor
const uint8_t sector_base = (pow(2, COLLISION_BITS / 2) - 1);
uint8_t left_side = 0;
uint8_t right_side = 0;
uint8_t proximity_sensor = 0;
Free_space free_space = LEFT;
bool wall_avoidance_start = false;

/* ---------------------------------------------- */
// Variables for Smart Arena messages
int sa_type = 0;
int sa_payload = 0;

/* Message send to the other kilobots */
message_t messageA;

/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;

/* Flag for decision to send a word */
bool sending_msg = false;

/* ---------------------------------------------- */
/***********EXPERIMENT VARIABLES***********/
uint32_t census_ticks;
int neighbor_count = 0;
int num_robots = NUM_ROBOTS;
int perceived_neighbors[NUM_ROBOTS];
int perceived_count = 0;

/*-------------------------------------------------------------------*/
/* Print Kilobot state                                               */
/*-------------------------------------------------------------------*/
void print_state()
{
  printf("Current walk: ");
  switch (current_walk)
  {
  case PERSISTENT:
    printf("PERSISTENT\n");
    break;
  case BROWNIAN:
    printf("BROWNIAN\n");
    break;

  default:
    printf("Error, no one of the possible state happens\n");
    break;
  }
}

/*-------------------------------------------------------------------*/
/* Turn on the right led color                                       */
/*-------------------------------------------------------------------*/
void check_state()
{
  switch (current_walk)
  {
  case BROWNIAN:
    set_color(RGB(0, 0, 3));
    break;
  case PERSISTENT:
    set_color(RGB(3, 0, 0));
    break;

  default:
    set_color(RGB(3, 3, 3));
    break;
  }
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
  if (current_motion_type != new_motion_type)
  {
    switch (new_motion_type)
    {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left, kilo_straight_right);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left, 0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0, kilo_turn_right);
      break;
    case STOP:
    default:
      set_motors(0, 0);
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
  // index of first element in the 3 sub-blocks of data
  uint8_t shift = kb_index * 3;

  sa_type = data[shift + 1] >> 2 & 0x0F;
  sa_payload = ((data[shift + 1] & 0b11) << 8) | (data[shift + 2]);

  switch (sa_type)
  {
  case 0:
    if (sa_payload != 0)
    {
      // get rotation toward the center (if far from center)
      // avoid colliding with the wall
      proximity_sensor = sa_payload;
      wall_avoidance_start = true;
    }
    break;
  }
}

/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx()
{
  if (sending_msg)
  {
    /* this one is filled in the loop */
    return &messageA;
  }
  return 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void message_tx_success()
{
  sending_msg = false;
}

/*-------------------------------------------------------------------*/
/* Function to broadcast a message                                        */
/*-------------------------------------------------------------------*/
void broadcast()
{
  if (sending_msg == false && (kilo_ticks > last_broadcast_ticks + max_broadcast_ticks || kilo_ticks < max_broadcast_ticks))
  {
    last_broadcast_ticks = kilo_ticks;
    sending_msg = true;
  }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d)
{

  /* Unpack the message - extract ID, type and payload */
  if (msg->type == ARK_MSG_TYPE)
  {
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);

    if (id1 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 0);
    }
    else if (id2 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 1);
    }
    else if (id3 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 2);
    }
  }

  else if (msg->type == KILO_MSG_TYPE)
  {
    // set_color(RGB(0, 3, 0));

    // If distance is too much, the message will be discarded
    uint8_t cur_distance = estimate_distance(d);

    /* ----------------------------------*/
    /* KB interactive message            */
    /* ----------------------------------*/
    // if new_information == true means that the kb has yet the info about the target, so the following msg not needed
    if (cur_distance < 100 && msg->data[0] != kilo_uid && msg->crc == message_crc(msg))
    {
      if (perceived_neighbors[msg->data[0]] != 1)
      {
        if (kilo_uid == 0)
        {
          printf("Percived kilobot %d\n", msg->data[0]);
        }
        perceived_count += 1;
        perceived_neighbors[msg->data[0]] = 1;
      }
    }
  }

  else
  {
    printf("Received msg with msd type: %d\n", msg->type);
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the LMCRW random walk                       */
/*-------------------------------------------------------------------*/

void random_walk()
{
  switch (current_walk)
  {
  case PERSISTENT:
    crw_exponent = 0.9;
    levy_exponent = 2.0;
    // printf("kID=%d, PERSISTENT\n", kilo_uid);
    break;
  case BROWNIAN:
    crw_exponent = 0.0;
    levy_exponent = 2.0;
    // printf("kID=%d, BROWNIAN\n", kilo_uid);
    break;
  default:
    break;
  }
  switch (current_motion_type)
  {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if (kilo_ticks > last_motion_ticks + turning_ticks)
    {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

  case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks)
    {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;

      if (rand_soft() % 2)
      {
        set_motion(TURN_LEFT);
      }
      else
      {
        set_motion(TURN_RIGHT);
      }
      double angle = 0;
      if (crw_exponent == 0)
      {
        angle = (uniform_distribution(0, (M_PI)));
      }
      else
      {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
    break;

  case STOP:
  default:
    set_motion(STOP);
  }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
  /* Initialise LED and motors */
  set_color(RGB(0, 0, 0));
  if (kilo_uid == 0)
    set_color(RGB(0, 0, 3));
  set_motors(0, 0);

  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  seed = rand_hard();
  srand(seed);

  set_motion(FORWARD);

  /* Initialise KBots message */
  messageA.type = KILO_MSG_TYPE; // 0 for ARK, 1 for KBots
  messageA.data[0] = kilo_uid;
  messageA.crc = message_crc(&messageA);

  /* Initialise counter to update kilobot neighbours count */
  census_ticks = kilo_ticks;

  /* initialize elements of array perceived_neighbors to 0 */
  for (int i = 0; i < num_robots; i++)
  {
    perceived_neighbors[i] = 0;
    if (i == kilo_uid)
      perceived_neighbors[i] = 1;
  }
}

/*-------------------------------------------------------------------*/
/* count 1s after decimal to binary conversion                       */
/*-------------------------------------------------------------------*/
uint8_t countOnes(uint8_t n)
{
  uint8_t count = 0;
  // array to store binary number
  // uint8_t binaryNum[8];

  // counter for binary array
  int i = 0;
  while (n > 0)
  {

    // storing remainder in binary array
    // binaryNum[i] = n % 2;
    if ((n % 2) == 1)
      count++;
    n = n / 2;
    i++;
  }

  return count;
}

/*-------------------------------------------------------------------*/
/* Function implementing wall avoidance procedure                    */
/*-------------------------------------------------------------------*/
void wall_avoidance_procedure(uint8_t sensor_readings)
{
  right_side = sensor_readings & sector_base;
  left_side = (sensor_readings >> (COLLISION_BITS / 2)) & sector_base;

  uint8_t count_ones = countOnes(sensor_readings);
  if (count_ones > SECTORS_IN_COLLISION)
  {
    if (right_side < left_side)
    {
      // set_color(RGB(0,0,3));
      set_motion(TURN_RIGHT);
      free_space = RIGHT;
    }
    else if (right_side > left_side)
    {
      // set_color(RGB(3,0,0));
      set_motion(TURN_LEFT);
      free_space = LEFT;
    }

    else
    {
      // rotate towards the last free space kept in memory
      set_motion(free_space);
    }
    if (kilo_ticks > last_motion_ticks + turning_ticks)
    {
      turning_ticks = (uint32_t)((M_PI / COLLISION_BITS) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
  }
  // else
  // {
  //   set_color(RGB(0,3,0));
  // }
}

/*-------------------------------------------------------------------*/
/* Reset variables each refresh_rate seconds                         */
/*-------------------------------------------------------------------*/
void update_variables()
{
  if (kilo_uid == 0)
  {
    // /* output each array element's value */
    // for (int j = 0; j < num_robots; j++)
    // {
    //   printf("Element[%d] = %d\n", j, perceived_neighbors[j]);
    // }
    printf("\n\n");
  }

  if (perceived_count >= NEIGHBOUR_THRESHOLD)
    current_walk = BROWNIAN;
  else
    current_walk = PERSISTENT;

  perceived_count = 0;

  for (int i = 0; i < num_robots; i++)
  {
    perceived_neighbors[i] = 0;
    if (i == kilo_uid)
      perceived_neighbors[i] = 1;
  }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
  // turn on the right led color
  check_state();
  // if (kilo_uid == 0)
  //   print_state();

  if (wall_avoidance_start)
  {
    wall_avoidance_procedure(proximity_sensor);
    proximity_sensor = 0;
    wall_avoidance_start = false;
  }
  else
  {
    random_walk();
    broadcast();
    if (kilo_ticks > (census_ticks + REFRESH_RATE * TICKS_TO_SEC)) // raise timer each 10 seconds
    {
      census_ticks = kilo_ticks;

      update_variables();
      /***
       * DO SOMETHING
       * ***/
    }
    // printf("kID=%d, crw=%f\tlevy=%f\n", kilo_uid, crw_exponent, levy_exponent);
  }
}

int main()
{
  kilo_init();
  // register message reception callback
  kilo_message_rx = message_rx;
  // register message transmission callback
  kilo_message_tx = message_tx;
  // register tranmsission success callback
  kilo_message_tx_success = message_tx_success;

  kilo_start(setup, loop);
  return 0;
}