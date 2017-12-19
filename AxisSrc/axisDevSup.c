#include    <math.h>
#include    "axisRecord.h"
#include    "axis.h"
#include    "axisDevSup.h"



/*******************************************************************************
Device support allows us to string several motor commands into a single
"transaction", using the calls prototyped below:

        int start_trans(dbCommon *mr)
        int build_trans(int command, double *parms, dbCommon *mr)
        int end_trans(struct dbCommon *mr, int go)

For clarity and to avoid typo's, the macros defined below provide simplified
calls.

                --- NOTE WELL ---
        The following macros assume that the variable "pmr" points to a motor
        record, and that the variable "pdset" points to that motor record's device
        support entry table:
                axisRecord *pmr;
                struct motor_dset *pdset = (struct motor_dset *)(pmr->dset);

        No checks are made in this code to ensure that these conditions are met.
*******************************************************************************/
/* To begin a transaction... */
#define INIT_MSG()                              (*pdset->start_trans)(pmr)

/* To send a single command... */
#define WRITE_MSG(cmd,parms)    (*pdset->build_trans)((cmd), (parms), pmr)

/* To end a transaction and send accumulated commands to the motor... */
#define SEND_MSG()                              (*pdset->end_trans)(pmr)


/*****************************************************************************
  Calls to device support
  Wrappers that call device support.
*****************************************************************************/
void devSupStop(axisRecord *pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    INIT_MSG();
    WRITE_MSG(STOP_AXIS, NULL);
    SEND_MSG();
}

/* No WRITE_MSG(STOP_AXIS, NULL); after this point */
#define STOP_AXIS #ErrorSTOP_AXIS
/******************************************************************************/

void devSupLoadPos(axisRecord *pmr, double newpos)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double tmp = newpos;
    INIT_MSG();
    WRITE_MSG(LOAD_POS, &tmp);
    SEND_MSG();
}
/* No WRITE_MSG(LOAD_POS, newpos); after this point */
#define LOAD_POS #ErrorLOAD_POS

/******************************************************************************/
RTN_STATUS devSupGetInfo(axisRecord *pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    RTN_STATUS status;
    INIT_MSG();
    status = WRITE_MSG(GET_INFO, NULL);
    if (status != ERROR)
    {
        SEND_MSG();
    }
    return status;
}
/* No WRITE_MSG(GET_INFO, NULL); after this point */
#define GET_INFO #ErrorGET_INFO

/*****************************************************************************/
RTN_STATUS devSupUpdateLimitFromDial(axisRecord *pmr, motor_cmnd command,
                                        double dialValue)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double tmp_raw = dialValue / pmr->mres;

    RTN_STATUS status;

    INIT_MSG();
    status = WRITE_MSG(command, &tmp_raw);
    if (status == OK)
    {
        SEND_MSG();
    }
    return status;
}

/*****************************************************************************/
void devSupMoveAbsRaw(axisRecord *pmr, double vel, double vbase,
                      double acc, double pos)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    INIT_MSG();
    if (vel <= vbase)
        vel = vbase + 1;
    WRITE_MSG(SET_VELOCITY, &vel);
    WRITE_MSG(SET_VEL_BASE, &vbase);
    if (acc > 0.0)  /* Don't SET_ACCEL if vel = vbase. */
        WRITE_MSG(SET_ACCEL, &acc);
    WRITE_MSG(MOVE_ABS, &pos);
    WRITE_MSG(GO, NULL);
    SEND_MSG();
}
/* No WRITE_MSG(MOVE_ABS, ); after this point */
#define MOVE_ABS #ErrorMOVE_ABS


/*****************************************************************************/
void devSupMoveRelRaw(axisRecord *pmr, double vel, double vbase,
                      double acc, double relpos)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    INIT_MSG();
    if (vel <= vbase)
        vel = vbase + 1;
    WRITE_MSG(SET_VELOCITY, &vel);
    WRITE_MSG(SET_VEL_BASE, &vbase);
    if (acc > 0.0)  /* Don't SET_ACCEL if vel = vbase. */
        WRITE_MSG(SET_ACCEL, &acc);
    WRITE_MSG(MOVE_REL, &relpos);
    WRITE_MSG(GO, NULL);
    SEND_MSG();
}
/* No WRITE_MSG(MOVE_REL, ); after this point */
#define MOVE_REL #ErrorMOVE_REL

/*****************************************************************************/
void devSupJogDial(axisRecord *pmr, double jogv, double jacc)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double jogvRaw = jogv / pmr->mres;
    double jaccRaw = jacc / fabs(pmr->mres);
    double vbaseRaw = pmr->vbas / fabs(pmr->mres);

    INIT_MSG();
    WRITE_MSG(SET_VEL_BASE, &vbaseRaw);
    WRITE_MSG(SET_ACCEL, &jaccRaw);
    WRITE_MSG(JOG, &jogvRaw);
    SEND_MSG();
    setCDIRfromRawMove(pmr, jogvRaw > 0);
}
/* No WRITE_MSG(JOG, ); after this point */
#define JOG #ErrorJOG

/*****************************************************************************/
void devSupUpdateJogRaw(axisRecord *pmr, double jogv, double jacc)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    INIT_MSG();
    WRITE_MSG(SET_ACCEL, &jacc);
    WRITE_MSG(JOG_VELOCITY, &jogv);
    SEND_MSG();
}
/* No WRITE_MSG(JOG_VELOCITY, ); after this point */
#define JOG_VELOCITY #ErrorJOG


/*****************************************************************************/
void devSupCNEN(axisRecord *pmr, double cnen)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double temp_dbl;
    INIT_MSG();
    if (cnen)
        WRITE_MSG(ENABLE_TORQUE, &temp_dbl);
    else
        WRITE_MSG(DISABL_TORQUE, &temp_dbl);
    SEND_MSG();
}

int devSupSetPID(axisRecord *pmr, motor_cmnd command, double *pcoeff)
{
  struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
  int changed = 0;
  RTN_STATUS rtnval;

  INIT_MSG();
  rtnval = (*pdset->build_trans)(command, pcoeff, pmr);
  /* If an error occured, build_trans() has reset the gain
   * parameter to a valid value for this controller. */
  if (rtnval != OK)
    changed = 1;

  SEND_MSG();
  return changed;
}

/*****************************************************************************/
void devSupSetEncRatio(axisRecord *pmr, double ep_mp[2])
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    INIT_MSG();
    WRITE_MSG(SET_ENC_RATIO, ep_mp);
    SEND_MSG();
}

/*
 * Set cdir dependent on the commanded move in "raw direction"
 * directionRaw > 0  is positive
 * directionRaw <= 0 is negative
 */
void setCDIRfromRawMove(axisRecord *pmr, int directionRaw)
{
    int cdirRaw = directionRaw > 0 ? 1 : 0; /* only 1 or 0 */
    if (pmr->cdir != cdirRaw)
    {
      //MARK_AUX(M_CDIR);
        pmr->cdir = cdirRaw;
    }
}
/*****************************************************************************

******************************************************************************/
/*
 * Set cdir dependent on the commanded move in "dial direction"
 * directionDial > 0  is positive
 * directionDial <= 0 is negative
 */
void setCDIRfromDialMove(axisRecord *pmr, int directionDial)
{
    int cdirRaw = directionDial > 0 ? 1 : 0;
    if (pmr->mres < 0.0)       /* mres < 0 means invert direction dial <-> raw */
        cdirRaw = !cdirRaw; /* If needed, 1 -> 0; 0 -> 1 */
    setCDIRfromRawMove(pmr, cdirRaw);
}

/*****************************************************************************/
void doHomeSetcdir(axisRecord *pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double vbase = pmr->vbas / fabs(pmr->mres);
    double hpos = 0;
    double hvel =  pmr->hvel / fabs(pmr->mres);
    double acc = (hvel - vbase) / pmr->accl;
    motor_cmnd command;

    INIT_MSG();
    WRITE_MSG(SET_VELOCITY, &hvel);
    WRITE_MSG(SET_VEL_BASE, &vbase);
    if (acc > 0.0)  /* Don't SET_ACCEL to zero. */
        WRITE_MSG(SET_ACCEL, &acc);

    if (((pmr->mip & MIP_HOMF) && (pmr->mres > 0.0)) ||
        ((pmr->mip & MIP_HOMR) && (pmr->mres < 0.0)))
        command = HOME_FOR;
    else
        command = HOME_REV;

    WRITE_MSG(command, &hpos);
    WRITE_MSG(GO, NULL);
    SEND_MSG();
    pmr->cdir = (command == HOME_FOR) ? 1 : 0;
}

