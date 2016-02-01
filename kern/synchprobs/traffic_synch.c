#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
#define NUM_OF_DIRECTIONS 4

static struct lock *intersection_lock;

static int num_vehicles[NUM_OF_DIRECTIONS][NUM_OF_DIRECTIONS];
static struct cv *intersection_cvs[NUM_OF_DIRECTIONS][NUM_OF_DIRECTIONS];

static bool
is_right_turn(Direction origin, Direction dest) {
  /*
   * right turn:      left turn:
   * north --> west   west  --> north
   * south --> east   east  --> south
   * west  --> south  south --> west
   * east  --> north  north --> east
   */
  return ((origin == north && dest == west) ||
	  (origin == south && dest == east) ||
	  (origin == west && dest == south) ||
	  (origin == east && dest == north));
}

static bool
legal_to_go(Direction origin, Direction dest) {
  for (unsigned int i=0; i<NUM_OF_DIRECTIONS; ++i) {
    for (unsigned int j=0; j<NUM_OF_DIRECTIONS; ++j) {
      if (num_vehicles[i][j] > 0) {
	// implict convert enum to unsigned int
	if ((origin == i && dest == j) ||
	    (origin == j && dest == i) ||
	    (dest != j && is_right_turn(origin, dest))) {
	  // ok to enter together
	  continue;
	}
	else {
	  return false;
	}
      }
    }
  }

  return true;
}


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  intersection_lock = lock_create("intersection_lock");
  if (intersection_lock == NULL) {
    panic("could not create intersection semaphore");
  }
  for (unsigned int i=0; i<NUM_OF_DIRECTIONS; ++i) {
    for (unsigned int j=0; j<NUM_OF_DIRECTIONS; ++j) {

      intersection_cvs[i][j] = cv_create("intersection cv");
      if (intersection_cvs[i][j] == NULL) {
	panic("could not create condition variable");
      }

      num_vehicles[i][j] = 0;
    }
  }

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersection_lock != NULL);
  lock_destroy(intersection_lock);

  for (unsigned int i=0; i<NUM_OF_DIRECTIONS; ++i) {
    for (unsigned int j=0; j<NUM_OF_DIRECTIONS; ++j) {

      cv_destroy(intersection_cvs[i][j]);

    }
  }
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  lock_acquire(intersection_lock);
  
  while (!legal_to_go(origin, destination)) {
    cv_wait(intersection_cvs[origin][destination], intersection_lock);
  }

  ++num_vehicles[origin][destination];

  lock_release(intersection_lock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  lock_acquire(intersection_lock);

  --num_vehicles[origin][destination];

  if (num_vehicles[origin][destination] == 0) {
    // last vehicle from origin to destination, notify other
    // potential waiting vehicles.
    for (unsigned int i=0; i<NUM_OF_DIRECTIONS; ++i) {
      for (unsigned int j=0; j<NUM_OF_DIRECTIONS; ++j) {
	cv_broadcast(intersection_cvs[i][j], intersection_lock);
      }
    }

  }

  lock_release(intersection_lock);
}
