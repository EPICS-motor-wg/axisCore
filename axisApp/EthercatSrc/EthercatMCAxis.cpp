/*
  FILENAME... EthercatMCAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "EthercatMC.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

/* temporally definition */
#ifndef ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED
#define ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED 0x2001C
#endif

#ifndef ERROR_CONFIG_ERROR
#define ERROR_CONFIG_ERROR 0x30000
#endif

#define NCOMMANDHOME 10

//
// These are the EthercatMCAxis methods
//

/** Creates a new EthercatMCAxis object.
 * \param[in] pC Pointer to the EthercatMCController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
EthercatMCAxis::EthercatMCAxis(EthercatMCController *pC, int axisNo,
                     int axisFlags, const char *axisOptionsStr)
  : asynAxisAxis(pC, axisNo),
    pC_(pC)
{
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;
  drvlocal.axisFlags = axisFlags;
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    const char * const cfgDebug_str = "getDebugText=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.externalEncoderStr = strdup(pThisOption);
        setIntegerParam(pC->motorStatusHasEncoder_, 1);
      }  else if (!strncmp(pThisOption, cfgfile_str, strlen(cfgfile_str))) {
        pThisOption += strlen(cfgfile_str);
        drvlocal.cfgfileStr = strdup(pThisOption);
      } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
        pThisOption += strlen(cfgDebug_str);
        drvlocal.cfgDebug_str = strdup(pThisOption);
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
}


extern "C" int EthercatMCCreateAxis(const char *EthercatMCName, int axisNo,
                               int axisFlags, const char *axisOptionsStr)
{
  EthercatMCController *pC;

  pC = (EthercatMCController*) findAsynPortDriver(EthercatMCName);
  if (!pC)
  {
    printf("Error port %s not found\n", EthercatMCName);
    return asynError;
  }
  pC->lock();
  new EthercatMCAxis(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxis::handleDisconnect()
{
  asynStatus status = asynSuccess;
  if (!drvlocal.dirty.oldStatusDisconnected) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "Communication error(%d)\n", axisNo_);
  }
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  setIntegerParam(pC_->motorStatusCommsError_, 1);
  callParamCallbacksUpdateError();
  return status;
}

asynStatus EthercatMCAxis::readConfigFile(void)
{
  const char *setRaw_str = "setRaw ";
  const char *setADRinteger_str = "setADRinteger ";
  const char *setADRdouble_str  = "setADRdouble ";
  FILE *fp;
  char *ret = &pC_->outString_[0];
  int line_no = 0;
  asynStatus status = asynSuccess;
  const char *errorTxt = NULL;
  /* no config file, or successfully uploaded : return */
  if (!drvlocal.cfgfileStr) {
    drvlocal.dirty.readConfigFile = 0;
    return asynSuccess;
  }
  if (!drvlocal.dirty.readConfigFile) return asynSuccess;

  fp = fopen(drvlocal.cfgfileStr, "r");
  if (!fp) {
    int saved_errno = errno;
    char cwdbuf[4096];
    char errbuf[4196];

    char *mypwd = getcwd(cwdbuf, sizeof(cwdbuf));
    snprintf(errbuf, sizeof(errbuf)-1,
             "readConfigFile: %s\n%s/%s",
             strerror(saved_errno),
             mypwd ? mypwd : "",
             drvlocal.cfgfileStr);
    updateMsgTxtFromDriver(errbuf);
    return asynError;
  }
  while (ret && !status && !errorTxt) {
    char rdbuf[256];
    size_t i;
    size_t len;
    int nvals = 0;

    line_no++;
    ret = fgets(rdbuf, sizeof(rdbuf), fp);
    if (!ret) break;    /* end of file or error */
    len = strlen(ret);
    if (!len) continue; /* empty line, no LF */
    for (i=0; i < len; i++) {
      /* No LF, no CR , no ctrl characters, */
      if (rdbuf[i] < 32) rdbuf[i] = 0;
    }
    len = strlen(ret);
    if (!len) continue; /* empty line with LF */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "readConfigFile %s:%u %s\n",
              drvlocal.cfgfileStr, line_no, rdbuf);

    if (rdbuf[0] == '#') {
      continue; /*  Comment line */
    } else if (!strncmp(setRaw_str, rdbuf, strlen(setRaw_str))) {
      const char *cfg_txt_p = &rdbuf[strlen(setRaw_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;

      snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", cfg_txt_p);
      status = writeReadACK();
    } else if (!strncmp(setADRinteger_str, rdbuf, strlen(setADRinteger_str))) {
      unsigned adsport;
      unsigned indexGroup;
      unsigned indexOffset;
      int value;
      const char *cfg_txt_p = &rdbuf[strlen(setADRinteger_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;
      nvals = sscanf(cfg_txt_p, "%u %x %x %d",
                     &adsport, &indexGroup, &indexOffset, &value);
      if (nvals == 4) {
        status = setADRValueOnAxisVerify(adsport, indexGroup, indexOffset,
                                         value, 1);
      } else {
        errorTxt = "Need 4 values";
      }
    } else if (!strncmp(setADRdouble_str, rdbuf, strlen(setADRdouble_str))) {
      unsigned adsport;
      unsigned indexGroup;
      unsigned indexOffset;
      double value;
      const char *cfg_txt_p = &rdbuf[strlen(setADRdouble_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;
      nvals = sscanf(cfg_txt_p, "%u %x %x %lf",
                     &adsport, &indexGroup, &indexOffset, &value);
      if (nvals == 4) {
        status = setADRValueOnAxisVerify(adsport, indexGroup, indexOffset,
                                         value, 1);
      } else {
        errorTxt = "Need 4 values";
      }
    } else {
      errorTxt = "Illegal command";
    }
    if (status || errorTxt) {
      char errbuf[256];
      errbuf[sizeof(errbuf)-1] = 0;
      if (status) {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "%s:%d out=%s\nin=%s",
                 drvlocal.cfgfileStr, line_no, pC_->outString_, pC_->inString_);
      } else {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "%s:%d \"%s\"\n%s",
                 drvlocal.cfgfileStr, line_no, rdbuf, errorTxt);
      }

      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "readConfigFile %s\n", errbuf);
      updateMsgTxtFromDriver(errbuf);
    }
  } /* while */

  if (ferror(fp) || status || errorTxt) {
    if (ferror(fp)) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "readConfigFile ferror (%s)\n",
                drvlocal.cfgfileStr);
    }
    fclose(fp);
    return asynError;
  }

  drvlocal.dirty.readConfigFile = 0;
  return asynSuccess;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxis::initialUpdate(void)
{
  asynStatus status = asynSuccess;

  /*  Check for Axis ID */
  int axisID = getMotionAxisID();
  if (axisID != axisNo_) {
    updateMsgTxtFromDriver("ConfigError AxisID");
    return asynError;
  }
  status = readConfigFile();
  if (status) return status;
  (void)updateMresSoftLimitsIfDirty(__LINE__);
  if ((status == asynSuccess) &&
      (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS)) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }

  if (!status) drvlocal.dirty.initialUpdate = 0;
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynAxisAxis::report()
 */
void EthercatMCAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynAxisAxis::report(fp, level);
}


/** Writes a command to the axis, and expects a logical ack from the controller
 * Outdata is in pC_->outString_
 * Indata is in pC_->inString_
 * The communiction is logged ASYN_TRACE_INFO
 *
 * When the communictaion fails ot times out, writeReadOnErrorDisconnect() is called
 */
asynStatus EthercatMCAxis::writeReadACK(void)
{
  asynStatus status = pC_->writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
    {
      const char *semicolon = &pC_->outString_[0];
      unsigned int numOK = 1;
      int res = 1;
      while (semicolon && semicolon[0]) {
        semicolon = strchr(semicolon, ';');
        if (semicolon) {
          numOK++;
          semicolon++;
        }
      }
      switch(numOK) {
        case 1: res = strcmp(pC_->inString_, "OK");  break;
        case 2: res = strcmp(pC_->inString_, "OK;OK");  break;
        case 3: res = strcmp(pC_->inString_, "OK:OK;OK");  break;
        case 4: res = strcmp(pC_->inString_, "OK;OK;OK;OK");  break;
        default:
          ;
      }
      if (res) {
        status = asynError;
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s in=%s return=%s (%d)\n",
                  pC_->outString_, pC_->inString_,
                  pasynManager->strStatus(status), (int)status);
        if (!drvlocal.cmdErrorMessage[0]) {
          snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
                   "writeReadACK() out=%s in=%s\n",
                   pC_->outString_, pC_->inString_);
          /* The poller co-ordinates the writing into the parameter library */
        }
        return status;
      }
    }
    default:
      break;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "out=%s in=%s status=%s (%d)\n",
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status);
  return status;
}


/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValueOnAxis(const char* var, int value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%d", axisNo_, var, value);
  return writeReadACK();
}

/** Sets an integer or boolean value on an axis, read it back and retry if needed
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
asynStatus EthercatMCAxis::setValueOnAxisVerify(const char *var, const char *rbvar,
                                                int value, unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter <= retryCount) {
    sprintf(pC_->outString_, "Main.M%d.%s=%d;Main.M%d.%s?",
            axisNo_, var, value, axisNo_, rbvar);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
              axisNo_,pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);
    if (status) {
      return status;
    } else {
      int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "nvals=%d command=\"%s\" response=\"%s\"\n",
                  nvals, pC_->outString_, pC_->inString_);
        return asynError;
      }
      if (status) break;
      if (rbvalue == value) break;
      counter++;
      epicsThreadSleep(.1);
    }
  }
  /* Verification failed.
     Store the error (unless there was an error before) */
  if ((rbvalue != value) && !drvlocal.cmdErrorMessage[0]) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
               "setValueOnAxisV(%d) var=%s value=%d rbvalue=%d",
               axisNo_,var, value, rbvalue);
      snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
               "setValueOnAxisV(%s) value=%d rbvalue=%d",
               var, value, rbvalue);

      /* The poller co-ordinates the writing into the parameter library */
  }
  return status;
}

/** Sets a floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValueOnAxis(const char* var, double value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%g", axisNo_, var, value);
  return writeReadACK();
}

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValuesOnAxis(const char* var1, double value1,
                                           const char* var2, double value2)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%g;Main.M%d.%s=%g",
          axisNo_, var1, value1, axisNo_, var2, value2);
  return writeReadACK();
}


int EthercatMCAxis::getMotionAxisID(void)
{
  int ret = drvlocal.dirty.nMotionAxisID;
  if (ret < 0) {
    asynStatus comStatus = getValueFromAxis("nMotionAxisID", &ret);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "getMotionAxisID(%d) comStatus=%d ret=%d\n",
              axisNo_, (int)comStatus, ret);
    if (comStatus) return -1;
  }

  if (ret >= 0) drvlocal.dirty.nMotionAxisID = ret;

  return ret;
}

asynStatus EthercatMCAxis::setADRValueOnAxis(unsigned adsport,
                                        unsigned indexGroup,
                                        unsigned indexOffset,
                                        int value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
          adsport, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setADRValueOnAxisVerify(unsigned adsport,
                                              unsigned indexGroup,
                                              unsigned indexOffset,
                                              int value,
                                              unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getADRValueFromAxis(adsport, indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setValueOnAxisVerify(%d) out=%s in=%s\n",
              axisNo_,pC_->outString_, pC_->inString_);
    status = setADRValueOnAxis(adsport, indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::setADRValueOnAxis(unsigned adsport,
                                        unsigned indexGroup,
                                        unsigned indexOffset,
                                        double value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
          adsport, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setADRValueOnAxisVerify(unsigned adsport,
                                              unsigned indexGroup,
                                              unsigned indexOffset,
                                              double value,
                                              unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getADRValueFromAxis(adsport, indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setValueOnAxisVerify(%d) out=%s in=%s\n",
              axisNo_,pC_->outString_, pC_->inString_);
    status = setADRValueOnAxis(adsport, indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::getADRValueFromAxis(unsigned adsport,
                                          unsigned indexGroup,
                                          unsigned indexOffset,
                                          int *value)
{
  int res;
  int nvals;
  asynStatus comStatus;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
          adsport, indexGroup + axisID, indexOffset);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

asynStatus EthercatMCAxis::getADRValueFromAxis(unsigned adsport,
                                          unsigned indexGroup,
                                          unsigned indexOffset,
                                          double *value)
{
  double res;
  int nvals;
  asynStatus comStatus;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          adsport, indexGroup + axisID, indexOffset);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}


/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxis::getValueFromAxis(const char* var, int *value)
{
  asynStatus comStatus;
  int res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "command=\"%s\" response=\"%s\"\n",
                pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "nvals=%d command=\"%s\" response=\"%s\"\n",
                nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  *value = res;
  return asynSuccess;
}


/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
asynStatus EthercatMCAxis::getValueFromAxis(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Gets a string value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the string result
 *
 */
asynStatus EthercatMCAxis::getStringFromAxis(const char *var, char *value, size_t maxlen)
{
  asynStatus comStatus;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus) return comStatus;

  memcpy(value, pC_->inString_, maxlen);
  return asynSuccess;
}

asynStatus EthercatMCAxis::getValueFromController(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "%s?", var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Set velocity and acceleration for the axis
 * \param[in] maxVelocity, mm/sec
 * \param[in] acceleration ???
 *
 */
asynStatus EthercatMCAxis::sendVelocityAndAccelExecute(double maxVelocity, double acceleration_time)
{
  asynStatus status;
  /* We don't use minVelocity */
  double maxVelocityEGU = maxVelocity * drvlocal.mres;
  if (!drvlocal.mres) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "sendVelocityAndAccelExecute(%d) mres==0.0\n",  axisNo_);
    return asynError; /* No mres, no move */
  }
  if (acceleration_time > 0.0001) {
    double acc_in_seconds = maxVelocity / acceleration_time;
    double acc_in_EGU_sec2 = maxVelocityEGU / acc_in_seconds;
    if (acc_in_EGU_sec2  < 0) acc_in_EGU_sec2 = 0 - acc_in_EGU_sec2 ;
    status = setValuesOnAxis("fAcceleration", acc_in_EGU_sec2,
                             "fDeceleration", acc_in_EGU_sec2);
    if (status) return status;
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "sendVelocityAndAccelExecute(%d) maxVelocityEGU=%g acceleration_time=%g\n",
              axisNo_, maxVelocityEGU, acceleration_time);
  }
  status = setValueOnAxis("fVelocity", maxVelocityEGU);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
  drvlocal.waitNumPollsBeforeReady += 2;
  return status;
}

/** Move the axis to a position, either absolute or relative
 * \param[in] position in mm
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  int nCommand = relative ? 2 : 3;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = updateMresSoftLimitsIfDirty(__LINE__);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) drvlocal.nCommand = nCommand;
  if (status == asynSuccess) status = setValueOnAxis("fPosition", position * drvlocal.mres);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus EthercatMCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  int nCommand = NCOMMANDHOME;

  int procHom;
  double posHom;
  double velToHom;
  double velFrmHom;
  double accHom;
  double decHom;

  if (status == asynSuccess) status = pC_->getIntegerParam(axisNo_,
                                                           pC_->EthercatMCProcHom_,
                                                           &procHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCPosHom_,
                                                          &posHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCVelToHom_,
                                                          &velToHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCVelFrmHom_,
                                                          &velFrmHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCAccHom_,
                                                          &accHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCDecHom_,
                                                          &decHom);

  /* The controller will do the home search, and change its internal
     raw value to what we specified in fPosition. */
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if ((drvlocal.axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) &&
      (status == asynSuccess)) status = enableAmplifier(1);
  if (status == asynSuccess) status = setValueOnAxis("fHomePosition", posHom);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand );
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", procHom);

  if (status == asynSuccess) status = setADRValueOnAxis(501, 0x4000, 0x6,
                                                        velToHom);
  if (status == asynSuccess) status = setADRValueOnAxis(501, 0x4000, 0x7,
                                                        velFrmHom);
  if (status == asynSuccess)  status = setValuesOnAxis("fAcceleration", accHom,
                                                       "fDeceleration", decHom);

  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
  if (status == asynSuccess) drvlocal.nCommand = nCommand;
  drvlocal.waitNumPollsBeforeReady += 2;
  return status;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;

  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = updateMresSoftLimitsIfDirty(__LINE__);
  if (status == asynSuccess) setValueOnAxis("nCommand", 1);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}



/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCAxis::setPosition(double value)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "setPosition(%d position=%g egu=%g\n",
            axisNo_, value, value * drvlocal.mres);
  return asynSuccess;
}

/** Set the low soft-limit on an axis
 *
 */
asynStatus EthercatMCAxis::setMotorLimitsOnAxisIfDefined(void)
{
  asynStatus status = asynError;
  drvlocal.dirty.motorLimits = 0;

  if (drvlocal.defined.motorLowLimit &&
      drvlocal.defined.motorHighLimit && drvlocal.mres) {
    unsigned int adsport = 501;
    unsigned int indexGroupA;
    int enable;
    int axisID = getMotionAxisID();
    if (axisID < 0) return asynError;
    indexGroupA = 0x5000 + (unsigned int)axisID;
    enable = drvlocal.motorLowLimit < drvlocal.motorHighLimit ? 1 : 0;
    if (enable) {
      snprintf(pC_->outString_, sizeof(pC_->outString_),
               "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g;"
               "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g;"
               "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d;"
               "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
               adsport, indexGroupA, 0xD, drvlocal.motorLowLimit * drvlocal.mres,
               adsport, indexGroupA, 0xE, drvlocal.motorHighLimit * drvlocal.mres,
               adsport, indexGroupA, 0XB, 1,
               adsport, indexGroupA, 0XC, 1);
    } else {
      snprintf(pC_->outString_, sizeof(pC_->outString_),
               "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d;"
               "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
               adsport, indexGroupA, 0XB, 0,
               adsport, indexGroupA, 0XC, 0);
    }
    status = writeReadACK();
  }
  drvlocal.dirty.motorLimits =  (status != asynSuccess);
  return status;
}


/** Update the soft limits in the controller, if needed
 *
 */
asynStatus EthercatMCAxis::updateMresSoftLimitsIfDirty(int line)
{
  asynStatus status = asynSuccess;
  asynPrint(pC_->pasynUserController_, ASYN_TRACEIO_DRIVER,
            "called from %d\n",line);
  if (drvlocal.dirty.motorLimits && status == asynSuccess) status = setMotorLimitsOnAxisIfDefined();
  return status;
}

asynStatus EthercatMCAxis::resetAxis(void)
{
  asynStatus status = asynSuccess;
  int EthercatMCErr;
  /* Reset command error, if any */
  drvlocal.cmdErrorMessage[0] = 0;
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCErr_, &EthercatMCErr);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "resetAxis(%d status=%d EthercatMCErr)=%d\n",
            axisNo_, (int)status, EthercatMCErr);

  if (EthercatMCErr) {
    /* Soft reset of the axis */
    status = setValueOnAxis("bExecute", 0);
    if (status) goto resetAxisReturn;
    status = setValueOnAxisVerify("bReset", "bReset", 1, 20);
    if (status) goto resetAxisReturn;
    epicsThreadSleep(.1);
    status = setValueOnAxisVerify("bReset", "bReset", 0, 20);
  }
  resetAxisReturn:
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "resetAxis(%d) status=%s (%d)\n",
            axisNo_, pasynManager->strStatus(status), (int)status);
  return status;
}

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCAxis::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;
  int ret;
  on = on ? 1 : 0; /* either 0 or 1 */
  status = getValueFromAxis("bEnabled", &ret);
  /* Either it went wrong OR the amplifier IS as it should be */
  if (status || (ret == on)) return status;
    
  status = setValueOnAxis("bEnable", on);
  if (status || !on) return status; /* this went wrong OR it should be turned off */
  while (counter) {
    epicsThreadSleep(.1);
    sprintf(pC_->outString_, "Main.M%d.%s?;Main.M%d.%s?", axisNo_, "bBusy", axisNo_, "bEnabled");
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "out=%s in=%s status=%s (%d)\n",
              pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);
    if (status) return status;
    if (!strcmp("0;1", pC_->inString_)) {
      /* bBusy == 0; bEnabled == 1 */
      return status;
    }
    counter--;
  }
  /* if we come here, it went wrong */
  return asynError;

}

/** Stop the axis
 *
 */
asynStatus EthercatMCAxis::stopAxisInternal(const char *function_name, double acceleration)
{
  asynStatus status;
  drvlocal.nCommand = 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "stopAxisInternal(%d) (%s)\n",  axisNo_, function_name);
  status = setValueOnAxisVerify("bExecute", "bExecute", 0, 1);
  if (status) drvlocal.mustStop = 1;
  return status;
}

/** Stop the axis, called by motor Record
 *
 */
asynStatus EthercatMCAxis::stop(double acceleration )
{
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void EthercatMCAxis::callParamCallbacksUpdateError()
{
  int EPICS_nErrorId = drvlocal.MCU_nErrorId;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.dirty.readConfigFile) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    drvlocal.eeAxisError = eeAxisErrorIOCcfgError;
          updateMsgTxtFromDriver("ConfigError Config File");
  } else if (drvlocal.dirty.initialUpdate) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    updateMsgTxtFromDriver("ConfigError");
  } else if (drvlocal.dirty.nMotionAxisID != axisNo_) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    updateMsgTxtFromDriver("ConfigError: AxisID");
  } else if (drvlocal.dirty.motorLimits) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    updateMsgTxtFromDriver("ConfigError: Soft limits");
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
  } else if (!drvlocal.homed &&
	     drvlocal.nCommand != NCOMMANDHOME) {
    int procHom = -1;
    pC_->getIntegerParam(axisNo_,
                         pC_->EthercatMCProcHom_,
                         &procHom);
    if (procHom &&
        drvlocal.defined.motorHighLimit &&
        drvlocal.defined.motorLowLimit &&
        (drvlocal.motorHighLimit > drvlocal.motorLowLimit))
      drvlocal.eeAxisError = eeAxisErrorNotHomed;
  }

  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId) {

    if (!drvlocal.cfgDebug_str) {
      if (!EPICS_nErrorId)
        updateMsgTxtFromDriver(NULL);

      switch (drvlocal.eeAxisError) {
        case eeAxisErrorNoError:
          updateMsgTxtFromDriver(NULL);
          break;
        case eeAxisErrorIOCcomError:
          updateMsgTxtFromDriver("CommunicationError");
          break;
        case eeAxisErrorCmdError:
          updateMsgTxtFromDriver(drvlocal.cmdErrorMessage);
          break;
        case eeAxisErrorNotHomed:
          updateMsgTxtFromDriver("Axis not homed");
          break;
        default:
          ;
      }
    }
    /* Axis has a problem: Report to motor record */
    setIntegerParam(pC_->motorStatusProblem_,
                    drvlocal.eeAxisError != eeAxisErrorNoError);

    /* MCU has a problem: set the red light in CSS */
    setIntegerParam(pC_->EthercatMCErr_,
                    drvlocal.eeAxisError == eeAxisErrorMCUError);
    setIntegerParam(pC_->EthercatMCErrId_, EPICS_nErrorId);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) callParamCallbacksUpdateError eeAxisError=%d old=%d ErrID=0x%x old=0x%x\n",
              axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId);

    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
  }

  callParamCallbacks();
}


asynStatus EthercatMCAxis::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
{
  asynStatus comStatus;

  int motor_axis_no = 0;
  int nvals;

  /* Read the complete Axis status */
  sprintf(pC_->outString_, "Main.M%d.stAxisStatus?", axisNo_);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus) return comStatus;
  drvlocal.dirty.stAxisStatus_V00 = 0;
  nvals = sscanf(pC_->inString_,
                 "Main.M%d.stAxisStatus="
                 "%d,%d,%d,%u,%u,%lf,%lf,%lf,%lf,%d,"
                 "%d,%d,%d,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d",
                 &motor_axis_no,
                 &pst_axis_status->bEnable,        /*  1 */
                 &pst_axis_status->bReset,         /*  2 */
                 &pst_axis_status->bExecute,       /*  3 */
                 &pst_axis_status->nCommand,       /*  4 */
                 &pst_axis_status->nCmdData,       /*  5 */
                 &pst_axis_status->fVelocity,      /*  6 */
                 &pst_axis_status->fPosition,      /*  7 */
                 &pst_axis_status->fAcceleration,  /*  8 */
                 &pst_axis_status->fDecceleration, /*  9 */
                 &pst_axis_status->bJogFwd,        /* 10 */
                 &pst_axis_status->bJogBwd,        /* 11 */
                 &pst_axis_status->bLimitFwd,      /* 12 */
                 &pst_axis_status->bLimitBwd,      /* 13 */
                 &pst_axis_status->fOverride,      /* 14 */
                 &pst_axis_status->bHomeSensor,    /* 15 */
                 &pst_axis_status->bEnabled,       /* 16 */
                 &pst_axis_status->bError,         /* 17 */
                 &pst_axis_status->nErrorId,       /* 18 */
                 &pst_axis_status->fActVelocity,   /* 19 */
                 &pst_axis_status->fActPosition,   /* 20 */
                 &pst_axis_status->fActDiff,       /* 21 */
                 &pst_axis_status->bHomed,         /* 22 */
                 &pst_axis_status->bBusy           /* 23 */);

  if (nvals == 24) {
    if (axisNo_ != motor_axis_no) return asynError;
    if (!drvlocal.supported.stAxisStatus_V00) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "pollAll(%d) fActPosition=%f\n",
                axisNo_, pst_axis_status->fActPosition);

      setIntegerParam(pC_->motorStatusHomeOnLs_, 1);
      setIntegerParam(pC_->motorStopOnProblem_, 0);
    }
    drvlocal.supported.stAxisStatus_V00 = 1;
    return asynSuccess;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "pollAll(%d) line=%d nvals=%d\n",
            axisNo_, __LINE__, nvals);
  drvlocal.supported.stAxisStatus_V00 = 0;
  return asynError;
}


/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus EthercatMCAxis::poll(bool *moving)
{
  asynStatus comStatus = asynSuccess;
  int mvnNotRdy = 0;
  st_axis_status_type st_axis_status;

  /* Driver not yet initialized, do nothing */
  if (!drvlocal.mres) return comStatus;

  memset(&st_axis_status, 0, sizeof(st_axis_status));
  /* Try to read to see if the connection is up */
  if (drvlocal.dirty.nMotionAxisID < 0) {
    int ret;
    comStatus = getValueFromAxis("nMotionAxisID", &ret);
    if (comStatus) goto skip;
    if (ret >= 0) drvlocal.dirty.nMotionAxisID = ret;
  }

  /* Stop if the previous stop had been lost */
  if (drvlocal.mustStop) {
    comStatus = stopAxisInternal(__FUNCTION__, 0);
    if (comStatus) goto skip;
  }
  if (drvlocal.dirty.initialUpdate) {
    comStatus = initialUpdate();
    if (comStatus) {
      callParamCallbacksUpdateError();
      return asynError;
    }
    if (drvlocal.dirty.oldStatusDisconnected) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "connected(%d)\n", axisNo_);
      drvlocal.dirty.oldStatusDisconnected = 0;
    }
  }

  if (drvlocal.supported.stAxisStatus_V00 || drvlocal.dirty.stAxisStatus_V00) {
    comStatus = pollAll(moving, &st_axis_status);
  } else {
    comStatus = asynError;
  }
  if (comStatus) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s in=%s return=%s (%d)\n",
              pC_->outString_, pC_->inString_,
              pasynManager->strStatus(comStatus), (int)comStatus);
    goto skip;
  }

  if (drvlocal.cfgDebug_str) {
    asynStatus comStatus;
    sprintf(pC_->outString_, "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadOnErrorDisconnect();
    if (!comStatus) {
      updateMsgTxtFromDriver(pC_->inString_);
    }
  }

  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  drvlocal.homed = st_axis_status.bHomed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);

  setDoubleParam(pC_->EthercatMCVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->EthercatMCAccAct_, st_axis_status.fAcceleration);
  setDoubleParam(pC_->EthercatMCDecAct_, st_axis_status.fDecceleration);

  mvnNotRdy = st_axis_status.bBusy && st_axis_status.bEnabled;

  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  } else {
    *moving = mvnNotRdy ? true : false;
    if (!mvnNotRdy) drvlocal.nCommand = 0;
  }

  if (drvlocal.nCommand != NCOMMANDHOME) {
    double newPositionInSteps = st_axis_status.fActPosition / drvlocal.mres;
    /* If not moving, trigger a record processing at low rate */
    if (!mvnNotRdy) setDoubleParam(pC_->motorPosition_, newPositionInSteps + 1);
    setDoubleParam(pC_->motorPosition_, newPositionInSteps);
    /* Use previous fActPosition and current fActPosition to calculate direction.*/
    if (st_axis_status.fActPosition > drvlocal.old_st_axis_status.fActPosition) {
      setIntegerParam(pC_->motorStatusDirection_, 1);
    } else if (st_axis_status.fActPosition < drvlocal.old_st_axis_status.fActPosition) {
      setIntegerParam(pC_->motorStatusDirection_, 0);
    }
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
  }

  if (drvlocal.externalEncoderStr) {
    double fEncPosition;
    comStatus = getValueFromController(drvlocal.externalEncoderStr, &fEncPosition);
    if (!comStatus) setDoubleParam(pC_->motorEncoderPosition_, fEncPosition);
  }

  if (drvlocal.old_st_axis_status.bHomed != st_axis_status.bHomed) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) homed=%d\n", axisNo_, st_axis_status.bHomed);
    drvlocal.old_st_axis_status.bHomed =  st_axis_status.bHomed;
  }
  if (drvlocal.old_st_axis_status.bLimitBwd != st_axis_status.bLimitBwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) LLS=%d\n", axisNo_, !st_axis_status.bLimitBwd);
    drvlocal.old_st_axis_status.bLimitBwd =  st_axis_status.bLimitBwd;
  }
  if (drvlocal.old_st_axis_status.bLimitFwd != st_axis_status.bLimitFwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) HLS=%d\n", axisNo_,!st_axis_status.bLimitFwd);
    drvlocal.old_st_axis_status.bLimitFwd = st_axis_status.bLimitFwd;
  }

  if (drvlocal.oldMvnNotRdy != mvnNotRdy) {
    drvlocal.waitNumPollsBeforeReady = 0;
  }
  if (drvlocal.waitNumPollsBeforeReady) {
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) mvnNotRdy=%d bBusy=%d bExecute=%d bEnabled=%d waitNumPollsBeforeReady=%d\n",
              axisNo_, mvnNotRdy,
              st_axis_status.bBusy, st_axis_status.bExecute,
              st_axis_status.bEnabled, drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  } else {
    if (drvlocal.oldMvnNotRdy != mvnNotRdy) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "poll(%d) mvnNotRdy=%d bBusy=%d bExecute=%d bEnabled=%d fActPosition=%g\n",
                axisNo_, mvnNotRdy,
                st_axis_status.bBusy, st_axis_status.bExecute,
                st_axis_status.bEnabled, st_axis_status.fActPosition);
      drvlocal.oldMvnNotRdy = mvnNotRdy;
    }
    setIntegerParam(pC_->motorStatusMoving_, mvnNotRdy);
    setIntegerParam(pC_->motorStatusDone_, !mvnNotRdy);

    drvlocal.MCU_nErrorId = st_axis_status.nErrorId;

    if (drvlocal.cfgDebug_str) {
      ; /* Do not do the following */
    } else if (drvlocal.old_bError != st_axis_status.bError ||
        drvlocal.old_MCU_nErrorId != drvlocal.MCU_nErrorId ||
        drvlocal.dirty.sErrorMessage) {
      char sErrorMessage[256];
      memset(&sErrorMessage[0], 0, sizeof(sErrorMessage));
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "poll(%d) bError=%d st_axis_status.nErrorId=0x%x\n",
                axisNo_, st_axis_status.bError,
                st_axis_status.nErrorId);
      drvlocal.old_bError = st_axis_status.bError;
      drvlocal.old_MCU_nErrorId = st_axis_status.nErrorId;
      drvlocal.dirty.sErrorMessage = 0;
      switch(st_axis_status.nErrorId) {
        case 0x4223:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                   "%x Axis positioning enable (sensor? limit?)",
                   st_axis_status.nErrorId);
          break;
        case 0x4450:
        case 0x4451:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,"%x Following error",
                   st_axis_status.nErrorId);
          break;
        case 0x4260:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,"%x Amplifier off",
                   st_axis_status.nErrorId);
          break;
        case 0x4263:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                   "%x ...is still being processed",
                   st_axis_status.nErrorId);
          break;
        case 0x4B0A:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                   "%x Homing not successful or not started (home sensor?)",
                   st_axis_status.nErrorId);
          break;
        case 0x4460:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                   "%x Low soft limit", st_axis_status.nErrorId);
          break;
        case 0x4461:
          snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                   "%x High soft limit", st_axis_status.nErrorId);
          break;
        default:
          break;
      }
      if (sErrorMessage[0]) {
        updateMsgTxtFromDriver(sErrorMessage);
      } else if (!sErrorMessage[0] && st_axis_status.nErrorId) {
        asynStatus status;
        status = getStringFromAxis("sErrorMessage", (char *)&sErrorMessage[0], sizeof(sErrorMessage));

        if (status == asynSuccess) updateMsgTxtFromDriver(sErrorMessage);
      }
    }
    callParamCallbacksUpdateError();
  }
  return asynSuccess;

  skip:
  handleDisconnect();
  return asynError;
}

/** Set the motor closed loop status
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus EthercatMCAxis::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setClosedLoop(%d)=%d\n", axisNo_, value);
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus EthercatMCAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  if (function == pC_->motorClosedLoop_) {
    ; /* handled via setClosedLoop() */
#ifdef motorRecDirectionString
  } else if (function == pC_->motorRecDirection_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setIntegerParam(%d motorRecDirection_)=%d\n", axisNo_, value);
#endif
#ifdef EthercatMCProcHomString
  } else if (function == pC_->EthercatMCProcHom_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setIntegerParam(%d ProcHom_)=%d\n", axisNo_, value);
#endif
#ifdef EthercatMCErrRstString
  } else if (function == pC_->EthercatMCErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "setIntegerParam(%d ErrRst_)=%d\n", axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
#endif
  }

  //Call base class method
  status = asynAxisAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
 * \param[in] function, which parameter is updated
 * \param[in] value, the new value
 *
 * When the IOC starts, we will send the soft limits to the controller.
 * When a soft limit is changed, and update is send them to the controller.
 */
asynStatus EthercatMCAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  if (function == pC_->motorHighLimit_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorHighLimit_)=%g\n", axisNo_, value);
    drvlocal.motorHighLimit = value;
    drvlocal.defined.motorHighLimit = 1;
    drvlocal.dirty.motorLimits = 1;
    setMotorLimitsOnAxisIfDefined();
  } else if (function == pC_->motorLowLimit_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorLowLimit_)=%g\n", axisNo_, value);
    drvlocal.motorLowLimit = value;
    drvlocal.defined.motorLowLimit = 1;
    drvlocal.dirty.motorLimits = 1;
    setMotorLimitsOnAxisIfDefined();
#ifdef motorRecResolutionString
  } else if (function == pC_->motorRecResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorRecResolution_=%g\n", axisNo_, value);
    drvlocal.mres = value;
    status = setMotorLimitsOnAxisIfDefined();
#endif
  }

  if (function == pC_->motorMoveRel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorMoveRel_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorMoveAbs_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorMoveVel_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorHome__)=%g\n", axisNo_, value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorStop_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorVelocity_=%g\n", axisNo_, value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorVelBase_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorAccel_)=%g\n", axisNo_, value);
#if 0
  } else if (function == pC_->motorPosition_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPosition_=%g\n", axisNo_, value);
  } else if (function == pC_->motorEncoderPosition_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorEncoderPosition_=%g\n", axisNo_, value);
#endif
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motmotorDeferMoves_=%g\n", axisNo_, value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motmotorMoveToHome_=%g\n", axisNo_, value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorResolution_=%g\n", axisNo_, value);
  } else if (function == pC_->motorEncoderRatio_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorEncoderRatio_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorPGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPGain_oveRel_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorIGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorIGain_oveRel_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorDGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoublmotor(%d motorDGain_oveRel_)=%g\n", axisNo_, value);
    /* Limits handled above */

#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPowerAutoOnOff_%g\n", axisNo_, value);
#endif
#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPowerOnDelay_)=%g\n", axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPowerOffDelay_=%g\n", axisNo_, value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motomotorPowerOffFraction_=%g\n", axisNo_, value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPostMoveDelay_=%g\n", axisNo_, value);
#endif
  } else if (function == pC_->motorStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorStatus_)=%g\n", axisNo_, value);
  } else if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorUpdateStatus_)=%g\n", axisNo_, value);
#ifdef motorRecOffsetString
  } else if (function == pC_->motorRecOffset_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorRecOffset_)=%g\n", axisNo_, value);
#endif
#ifdef EthercatMCHVELFRMString
  } else if (function == pC_->EthercatMCHVELfrm_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d HVELfrm_)=%g\n", axisNo_, value);
#endif
#ifdef EthercatMCPosHomString
  } else if (function == pC_->EthercatMCPosHom_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d PosHom_)=%f\n", axisNo_, value);
#endif
  }

  // Call the base class method
  status = asynAxisAxis::setDoubleParam(function, value);
  return status;
}

asynStatus EthercatMCAxis::setStringParam(int function, const char *value)
{
  asynStatus status = asynSuccess;
#ifdef profileMoveModeString
  /* motorRecord 6.8.1 doesn't have setStringParam(),
     but modern have, as they have profileMoveModeString.
     We could check for VERSION, but this simple #ifdef
     works for our needs */

  /* Call base class method */
  status = asynAxisAxis::setStringParam(function, value);
#endif
  return status;
}
