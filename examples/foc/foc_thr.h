/****************************************************************************
 * examples/foc/foc_thr.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __EXAMPLES_FOC_FOC_THR_H
#define __EXAMPLES_FOC_FOC_THR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/foc/foc.h>
#include <nuttx/power/power_ioctl.h>

#include <mqueue.h>

#include "foc_device.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FOC_THR_MQNAME "mqf"

/****************************************************************************
 * Public Type Definition
 ****************************************************************************/

/* FOC example state */

enum foc_example_state_e
{
  FOC_EXAMPLE_STATE_INVALID = 0, /* Reserved */
  FOC_EXAMPLE_STATE_FREE    = 1, /* No current */
  FOC_EXAMPLE_STATE_STOP    = 2, /* Active break */
  FOC_EXAMPLE_STATE_CW      = 3, /* CW direction */
  FOC_EXAMPLE_STATE_CCW     = 4, /* CCW direction */
};

/* Thread message type */

enum foc_thr_msg_e
{
  FOC_THR_MSG_INVALID = 0,
  FOC_THR_MSG_VBUS    = 1,
  FOC_THR_MSG_STATE   = 2,
  FOC_THR_MSG_VEL     = 3,
  FOC_THR_MSG_MODE    = 4,
  FOC_THR_MSG_QPARAM  = 5,
  FOC_THR_MSG_KP      = 6,
  FOC_THR_MSG_KI      = 7,
  FOC_THR_MSG_VELMAX  = 8,
  FOC_THR_MSG_START   = 9,
  FOC_THR_MSG_KILL    = 10
};

/* FOC thread data */

struct foc_ctrl_env_s
{
  int id;                      /* Device id */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_POWER_FOC_USE_FLOAT
int foc_float_thr(FAR struct foc_ctrl_env_s *envp,
                  FAR struct foc_device_s *dev,
                  mqd_t mqd);
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
int foc_fixed16_thr(FAR struct foc_ctrl_env_s *envp,
                    FAR struct foc_device_s *dev,
                    mqd_t mqd);
#endif

#endif /* __EXAMPLES_FOC_FOC_THR_H */
