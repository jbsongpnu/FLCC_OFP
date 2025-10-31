/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_PMUCAN.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
