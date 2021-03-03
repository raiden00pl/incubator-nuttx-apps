/****************************************************************************
 * examples/foc/foc_state_thr.h
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

#ifndef __EXAMPLES_FOC_FOC_STATE_THR_H
#define __EXAMPLES_FOC_FOC_STATE_THR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/foc/foc.h>
#include <nuttx/power/power_ioctl.h>

#include "foc_device.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FOC_STATE_THR_MQNAME "mqfs"

/****************************************************************************
 * Public Type Definition
 ****************************************************************************/

/* Thread messages */

enum foc_state_msg_e
{
  FOC_STATE_MSG_INVAL = 0x00,
  FOC_STATE_MSG_KILL  = 0x01,
};

/* FOC state thread data */

struct foc_state_env_s
{
  int8_t en;                    /* Enabled instances */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *foc_state_thr(FAR void *arg);

#endif /* __EXAMPLES_FOC_FOC_STATE_THR_H */
