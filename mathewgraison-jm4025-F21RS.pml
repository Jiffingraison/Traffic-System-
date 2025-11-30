#define MAX_REQ 1 /* Set MAX_REQ = 3 for T2 & T3 & 1 for T4 */

/* Message types for manoeuvres */
mtype = { STRAIGHT, LEFT, RIGHT };

/*  External channels: AV -> Router, Controller -> AV */
chan req1 = [1] of { mtype };
chan res1 = [1] of { mtype };

chan req2 = [1] of { mtype };
chan res2 = [1] of { mtype };

chan req3 = [1] of { mtype };
chan res3 = [1] of { mtype };

/* Internal router -> controller channels */
chan alpha_in = [3] of { byte, mtype };
chan beta_in  = [3] of { byte, mtype };

/* Permission event channel for T2 monitor */
chan perm_events = [10] of { byte, mtype };

/* Global variables used by T2 + LTL Safety*/
byte  last_id  = 0;
mtype last_dir = STRAIGHT;

/* Flags for T4: true when AV is waiting for reply */
bool wait1 = false;
bool wait2 = false;
bool wait3 = false;


/*  Autonomous Vehicles (AV1, AV2, AV3) */

/* Type 1: Private Car (STRAIGHT only) */
proctype AV1()
{
    mtype dir = STRAIGHT;
    byte  c   = 0;

    do
    :: (c < MAX_REQ) ->
           wait1 = true;
           req1 ! dir;
           res1 ? dir;
           perm_events ! 1, dir;
           wait1 = false;
           c++
    :: else -> break
    od
}

/* Type 2: City Bus (LEFT only) */
proctype AV2()
{
    mtype dir = LEFT;
    byte  c   = 0;

    do
    :: (c < MAX_REQ) ->
           wait2 = true;
           req2 ! dir;
           res2 ? dir;
           perm_events ! 2, dir;
           wait2 = false;
           c++
    :: else -> break
    od
}

/* Type 3: Robotaxi (alternates RIGHT/LEFT) */
proctype AV3()
{
    mtype dir = RIGHT;
    byte  c   = 0;

    do
    :: (c < MAX_REQ) ->
           wait3 = true;
           req3 ! dir;
           res3 ? dir;
           perm_events ! 3, dir;
           wait3 = false;

           /* Alternate for next request */
           if
           :: dir == RIGHT -> dir = LEFT
           :: dir == LEFT  -> dir = RIGHT
           fi;

           c++
    :: else -> break
    od
}


/* Router: central dispatcher */

proctype Router()
{
    mtype dir;

    do
    :: req1 ? dir ->
           alpha_in ! 1, dir       /* Type 1 always → Alpha */

    :: req2 ? dir ->
           beta_in ! 2, dir        /* Type 2 always → Beta */

    :: req3 ? dir ->
           if
           :: dir == RIGHT -> alpha_in ! 3, dir
           :: dir == LEFT  -> beta_in  ! 3, dir
           fi
    od
}


/* Controllers */

/* Alpha Controller: must NEVER process LEFT */
proctype Alpha()
{
    byte  id;
    mtype dir;

    do
    :: alpha_in ? id, dir ->
           if
           :: dir == LEFT ->
                  assert(false)     /* invalid for Alpha */
           :: else ->
                  if
                  :: id == 1 -> res1 ! dir
                  :: id == 3 -> res3 ! dir
                  fi
           fi
    od
}

/* Beta Controller: must NEVER process RIGHT */
proctype Beta()
{
    byte  id;
    mtype dir;

    do
    :: beta_in ? id, dir ->
           if
           :: dir == RIGHT ->
                  assert(false)     /* invalid for Beta */
           :: else ->
                  if
                  :: id == 2 -> res2 ! dir
                  :: id == 3 -> res3 ! dir
                  fi
           fi
    od
}


/* Safety Monitor (T2): assertion-based checks */

proctype SafetyMonitor()
{
    do
    :: perm_events ? last_id, last_dir ->
           if
           :: last_id == 1 -> assert(last_dir == STRAIGHT)
           :: last_id == 2 -> assert(last_dir == LEFT)
           :: last_id == 3 -> assert(last_dir == LEFT || last_dir == RIGHT)
           fi
    od
}

/* T3- comment out T3 when running T4*/
/*
ltl Safety {
    [] !((last_id == 1 && last_dir != STRAIGHT) ||
         (last_id == 2 && last_dir != LEFT)     ||
         (last_id == 3 && !(last_dir == LEFT || last_dir == RIGHT)))
}
*/

/* T4- comment out T4 when running T3 */
ltl Response {
    [] ( (wait1 -> <> !wait1) &&
         (wait2 -> <> !wait2) &&
         (wait3 -> <> !wait3) )
}


/* env prevent invalid state*/

proctype Env()
{
    do :: timeout -> skip od
}


/* init launch the system processess */

init { 
    run AV1();
    run AV2();
    run AV3();
    run Router();
    run Alpha();
    run Beta();
    run SafetyMonitor();
    run Env();
}
