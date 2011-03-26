/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include "linux/init.h"
#include "linux/module.h"
#include "linux/kernel.h"
#include "linux/err.h"
#include "linux/power_supply.h"
#include "linux/platform_device.h"
#include "linux/wakelock.h"
#include "mach/msm_rpcrouter.h"
#include "mach/board.h"

#define SHBATT_REMOTE_A2MPROG  0x300000A0
#define SHBATT_REMOTE_A2MVERS  0x00010001

#define SHBATT_API_INITIALIZE_REMOTE_PROC                       2
#define SHBATT_API_GET_CHARGER_CABLE_STATUS_REMOTE_PROC         3
#define SHBATT_API_GET_BATTERY_CHARGING_STATUS_REMOTE_PROC      4
#define SHBATT_API_GET_BATTERY_CAPACITY_REMOTE_PROC             5
#define SHBATT_API_GET_BATTERY_VOLTAGE_REMOTE_PROC              6
#define SHBATT_API_GET_BATTERY_PRESENT_REMOTE_PROC              7
#define SHBATT_API_GET_BATTERY_TEMPERATURE_REMOTE_PROC          8
#define SHBATT_API_GET_BATTERY_HEALTH_REMOTE_PROC               9
#define SHBATT_API_GET_BATTERY_TECHNOLOGY_REMOTE_PROC           10
#define SHBATT_API_GET_FUELGAUGE_CURRENT_REMOTE_PROC            11
#define SHBATT_API_GET_FUELGAUGE_VOLTAGE_REMOTE_PROC            12
#define SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC                 13
#define SHBATT_API_SET_VBATT_CALIBRATION_DATA_REMOTE_PROC       14
#define SHBATT_API_REFRESH_VBATT_CALIBRATION_DATA_REMOTE_PROC   15
#define SHBATT_API_SET_FUELGAUGE_MODE_REMOTE_PROC               16
#define SHBATT_API_GET_FUELGAUGE_DEVICE_ID_REMOTE_PROC          17

#define SH_BATTERY_REMOTE_M2APROG  0x300000A1
#define SH_BATTERY_REMOTE_M2AVERS  0x00010001

#define SH_BATTERY_UPDATE_CHARGER_CABLE_STATUS_PROC     2
#define SH_BATTERY_UPDATE_BATTERY_CHARGING_STATUS_PROC  3
#define SH_BATTERY_UPDATE_BATTERY_CAPACITY_PROC         4

#define SHCHG_REMOTE_A2MPROG  0x300000A2
#define SHCHG_REMOTE_A2MVERS  0x00010001

#define SHCHG_API_INITIALIZE_REMOTE_PROC                             2
#define SHCHG_API_NOTIFY_USB_CHARGER_CONNECTED_REMOTE_PROC           3
#define SHCHG_API_NOTIFY_USB_CHARGER_DISCONNECTED_REMOTE_PROC        4
#define SHCHG_API_NOTIFY_USB_CHARGER_I_IS_AVAILABLE_REMOTE           5
#define SHCHG_API_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE_REMOTE_PROC  6

#define SHTHERM_REMOTE_A2MPROG  0x300000A3
#define SHTHERM_REMOTE_A2MVERS  0x00010001

#define SHTHERM_API_INITIALIZE_REMOTE_PROC 2
#define SHTHERM_API_GET_BATTERY_TEMPERATURE_REMOTE_PROC 3
#define SHTHERM_API_GET_CHARGER_TEMPERATURE_REMOTE_PROC 4
#define SHTHERM_API_GET_CAMERA_TEMPERATURE_REMOTE_PROC 5
#define SHTHERM_API_GET_PA_TEMPERATURE_REMOTE_PROC 6
#define SHTHERM_API_GET_PMIC_TEMPERATURE_REMOTE_PROC 7

#ifdef SH_BATTERY_TRACE

#define TRACE(x...) printk(KERN_INFO "[SH_BATTERY] " x)

#else

#define TRACE(x...) do {} while(0)

#endif /* SH_BATTERY_TRACE */

#define SH_BATTERY_ATTR(_name)							           \
{                                                                  \
  .attr  = { .name = #_name, .mode = 0444, .owner = THIS_MODULE }, \
  .show  = sh_battery_show_property,                               \
  .store = NULL,                                                   \
}

static ssize_t sh_battery_show_property( struct device* dev,
                                         struct device_attribute* attr,
                                         char* buf );

enum { BATTERY, USB, AC, FUELGAUGE, ADC };

static struct power_supply sh_battery_power_supplies[] =
{
  {
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
  },

  {
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
  },

  {
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
  },

  {
    .name = "fuelgauge",
    .type = POWER_SUPPLY_TYPE_BATTERY,
  },

  {
    .name = "adc",
    .type = POWER_SUPPLY_TYPE_BATTERY,
  }
};

enum
{
  SH_BATTERY_PROPERTY_ONLINE,
  SH_BATTERY_PROPERTY_STATUS,
  SH_BATTERY_PROPERTY_HEALTH,
  SH_BATTERY_PROPERTY_PRESENT,
  SH_BATTERY_PROPERTY_CAPACITY,
  SH_BATTERY_PROPERTY_BATT_VOL,
  SH_BATTERY_PROPERTY_BATT_TEMP,
  SH_BATTERY_PROPERTY_TECHNOLOGY,

  SH_BATTERY_PROPERTY_CURRENT,
  SH_BATTERY_PROPERTY_VOLTAGE,
  SH_BATTERY_PROPERTY_DEVICE_ID,

  SH_BATTERY_PROPERTY_VCOIN_MV,
  SH_BATTERY_PROPERTY_VBATT_MV,
  SH_BATTERY_PROPERTY_VCHG_MV,
  SH_BATTERY_PROPERTY_ICHG_MV,
  SH_BATTERY_PROPERTY_ICHG_OUT_MV,
  SH_BATTERY_PROPERTY_BATT_ID_MV,
  SH_BATTERY_PROPERTY_BATT_THERM_DEGC,
  SH_BATTERY_PROPERTY_USB_VBUS_MV,
  SH_BATTERY_PROPERTY_PMIC_THERM_DEGC,
  SH_BATTERY_PROPERTY_CHG_THERM_DEGC,
  SH_BATTERY_PROPERTY_PA_THERM_DEGC,
  SH_BATTERY_PROPERTY_MSM_THERM_DEGC,
  SH_BATTERY_PROPERTY_CAMERA_THERM_DEGC
};

static struct device_attribute sh_battery_attrs[] =
{
  SH_BATTERY_ATTR(online),
  SH_BATTERY_ATTR(status),
  SH_BATTERY_ATTR(health),
  SH_BATTERY_ATTR(present),
  SH_BATTERY_ATTR(capacity),
  SH_BATTERY_ATTR(batt_vol),
  SH_BATTERY_ATTR(batt_temp),
  SH_BATTERY_ATTR(technology),

  SH_BATTERY_ATTR(current),
  SH_BATTERY_ATTR(voltage),
  SH_BATTERY_ATTR(device_id),

  SH_BATTERY_ATTR(vcoin_mv),
  SH_BATTERY_ATTR(vbatt_mv),
  SH_BATTERY_ATTR(vchg_mv),
  SH_BATTERY_ATTR(ichg_mv),
  SH_BATTERY_ATTR(ichg_out_mv),
  SH_BATTERY_ATTR(batt_id_mv),
  SH_BATTERY_ATTR(batt_therm_degc),
  SH_BATTERY_ATTR(usb_vbus_mv),
  SH_BATTERY_ATTR(pmic_therm_degc),
  SH_BATTERY_ATTR(chg_therm_degc),
  SH_BATTERY_ATTR(pa_therm_degc),
  SH_BATTERY_ATTR(msm_therm_degc),
  SH_BATTERY_ATTR(camera_therm_degc)
};

enum
{
  SH_BATTERY_CABLE_STATUS_NONE,
  SH_BATTERY_CABLE_STATUS_USB,
  SH_BATTERY_CABLE_STATUS_AC
};

enum
{
  SH_BATTERY_CHG_STATUS_UNKNOWN,
  SH_BATTERY_CHG_STATUS_CHARGING,
  SH_BATTERY_CHG_STATUS_DISCHARGING,
  SH_BATTERY_CHG_STATUS_NOT_CHARGING,
  SH_BATTERY_CHG_STATUS_FULL
};

enum
{
  SH_BATTERY_ADC_CHANNEL_VBATT,
  SH_BATTERY_ADC_CHANNEL_VCHG,
  SH_BATTERY_ADC_CHANNEL_VCOIN_MV,
  SH_BATTERY_ADC_CHANNEL_VBATT_MV,
  SH_BATTERY_ADC_CHANNEL_VCHG_MV,
  SH_BATTERY_ADC_CHANNEL_ICHG_MV,
  SH_BATTERY_ADC_CHANNEL_ICHG_OUT_MV,
  SH_BATTERY_ADC_CHANNEL_BATT_ID_MV,
  SH_BATTERY_ADC_CHANNEL_BATT_THERM_DEGC,
  SH_BATTERY_ADC_CHANNEL_USB_VBUS_MV,
  SH_BATTERY_ADC_CHANNEL_PMIC_THERM_DEGC,
  SH_BATTERY_ADC_CHANNEL_CHG_THERM_DEGC,
  SH_BATTERY_ADC_CHANNEL_PA_THERM_DEGC,
  SH_BATTERY_ADC_CHANNEL_MSM_THERM_DEGC,
  SH_BATTERY_ADC_CHANNEL_CAMERA_THERM_DEGC,
};

static struct mutex sh_battery_lock;

static struct wake_lock sh_battery_vbus_wake_lock;

static struct msm_rpc_endpoint* ep_shbatt_p;

static struct msm_rpc_endpoint* ep_shchg_p;

static struct msm_rpc_endpoint* ep_shtherm_p;

static int sh_battery_charger_cable_status;

static int sh_battery_battery_charging_status;

static int sh_battery_battery_capacity;

static ssize_t sh_battery_show_property( struct device* dev,
                                         struct device_attribute* attr,
                                         char* buf )
{
  static char* status_text[] = 
  {
    "Unknown", "Charging", "Discharging", "Not charging", "Full" 
  };

  static char* health_text[] = 
  {
    "Unknown", "Good", "Overheat", "Dead", "Over voltage", "Unspecified failure", "Cold"
  };

  static char* technology_text[] = 
  {
    "Unknown", "NiMH", "Li-ion", "Li-poly", "LiFe", "NiCd", "LiMn"
  };

  struct shbatt_api_remote_req_t1
  {
    struct rpc_request_hdr hdr;
  } req_t1;

  struct shbatt_api_remote_req_t2
  {
    struct rpc_request_hdr hdr;
    int value;
  } req_t2;

  struct shbatt_api_remote_rep_t1
  {
    struct rpc_reply_hdr hdr;
    int result;
    int value;
  } rep_t1;

  const ptrdiff_t property = attr - sh_battery_attrs;

  ssize_t size;

  int result, value;

  TRACE("%s attr = %d\n",__FUNCTION__,property);

  mutex_lock(&sh_battery_lock);

  if(property == SH_BATTERY_PROPERTY_ONLINE)
  {
    if(dev == sh_battery_power_supplies[USB].dev)
    {
      value = (sh_battery_charger_cable_status == SH_BATTERY_CABLE_STATUS_USB) ? 1 : 0;
      TRACE("/usb/online = %d\n",value);
    }
    else if(dev == sh_battery_power_supplies[AC].dev)
    {
      value = (sh_battery_charger_cable_status == SH_BATTERY_CABLE_STATUS_AC) ? 1 : 0;
      TRACE("/ac/online = %d\n",value);
    }
    else
    {
      value = 0;
    }
    size = sprintf(buf,"%d\n",value);
  }
  else if(property == SH_BATTERY_PROPERTY_STATUS)
  {
    size = sprintf(buf,"%s\n",status_text[sh_battery_battery_charging_status]);
    TRACE("/battery/status = %s\n",status_text[sh_battery_battery_charging_status]);
  }
  else if(property == SH_BATTERY_PROPERTY_HEALTH)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_BATTERY_HEALTH_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get battery health failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%s\n",health_text[value]);
      TRACE("/battery/health = %s\n",health_text[value]);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_PRESENT)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_BATTERY_PRESENT_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get battery present failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/battery/present = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_CAPACITY)
  {
    size = sprintf(buf,"%d\n",sh_battery_battery_capacity);
    TRACE("/battery/capacity = %d\n",sh_battery_battery_capacity);
  }
  else if(property == SH_BATTERY_PROPERTY_BATT_VOL)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_BATTERY_VOLTAGE_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get battery voltage failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/battery/batt_vol = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_BATT_TEMP)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_BATTERY_TEMPERATURE_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get battery temperature failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      value = value * 10;
      size = sprintf(buf,"%d\n",value);
      TRACE("/battery/batt_temp = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_TECHNOLOGY)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_BATTERY_TECHNOLOGY_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get battery technology failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%s\n",technology_text[value]);
      TRACE("/battery/technology = %s\n",technology_text[value]);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_CURRENT)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_FUELGAUGE_CURRENT_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get fuelgauge current failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/fuelgauge/current = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_VOLTAGE)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_FUELGAUGE_VOLTAGE_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get fuelgauge voltage failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/fuelgauge/voltage = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_DEVICE_ID)
  {
    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_GET_FUELGAUGE_DEVICE_ID_REMOTE_PROC,
                                &req_t1,sizeof(req_t1),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : get fuelgauge device_id failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/fuelgauge/device_id = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_VCOIN_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_VCOIN_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (vcoin_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/vcoin_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_VBATT_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_VBATT_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (vbatt_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/vbatt_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_VCHG_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_VCHG_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (vchg_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/vchg_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_ICHG_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_ICHG_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (ichg_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/ichg_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_ICHG_OUT_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_ICHG_OUT_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (ichg_out_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/ichg_out_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_BATT_ID_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_BATT_ID_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (batt_id_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/batt_id_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_BATT_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_BATT_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (batt_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/batt_therm_degc = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_USB_VBUS_MV)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_USB_VBUS_MV);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (usb_vbus_mv) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/usb_vbus_mv = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_PMIC_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_PMIC_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (pmic_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/pmic_therm_degc = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_CHG_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_CHG_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (chg_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/chg_therm_degc = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_PA_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_PA_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (pa_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/pa_therm_degc = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_MSM_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_MSM_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (msm_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/msm_therm_degc = %d\n",value);
    }
  }
  else if(property == SH_BATTERY_PROPERTY_CAMERA_THERM_DEGC)
  {
    req_t2.value = cpu_to_be32(SH_BATTERY_ADC_CHANNEL_CAMERA_THERM_DEGC);

    result = msm_rpc_call_reply(ep_shbatt_p,
                                SHBATT_API_READ_ADC_CHANNEL_REMOTE_PROC,
                                &req_t2,sizeof(req_t2),
                                &rep_t1,sizeof(rep_t1),
                                5 * HZ);
    if(result < 0)
    {
      size = -1;
      TRACE("%s : read adc channel (camera_therm_degc) failed.\n",__FUNCTION__);
    }
    else
    {
      value = be32_to_cpu(rep_t1.value);
      size = sprintf(buf,"%d\n",value);
      TRACE("/adc/camera_therm_degc = %d\n",value);
    }
  }
  else
  {
    size = -1;
  }

  mutex_unlock(&sh_battery_lock);

  return size;
}

static int sh_battery_create_attrs( void )
{
  int i, result;

  TRACE("%s \n",__FUNCTION__);

  for(i = 1; i <= 7; i++)
  {
    result = device_create_file(sh_battery_power_supplies[BATTERY].dev,&sh_battery_attrs[i]);

    if(result != 0)
    {
      device_remove_file(sh_battery_power_supplies[BATTERY].dev,&sh_battery_attrs[i]);

      TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[i].attr.name);
      return -1;
    }
  }

  result = device_create_file(sh_battery_power_supplies[USB].dev,&sh_battery_attrs[0]);

  if(result != 0)
  {
    device_remove_file(sh_battery_power_supplies[USB].dev,&sh_battery_attrs[0]);

    TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[0].attr.name);
    return -1;
  }

  result = device_create_file(sh_battery_power_supplies[AC].dev,&sh_battery_attrs[0]);

  if(result != 0)
  {
    device_remove_file(sh_battery_power_supplies[AC].dev,&sh_battery_attrs[0]);

    TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[0].attr.name);
    return -1;
  }

  for(i = 8; i <= 10; i++)
  {
    result = device_create_file(sh_battery_power_supplies[FUELGAUGE].dev,&sh_battery_attrs[i]);

    if(result != 0)
    {
      device_remove_file(sh_battery_power_supplies[FUELGAUGE].dev,&sh_battery_attrs[i]);

      TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[i].attr.name);
      return -1;
    }
  }

  result = device_create_file(sh_battery_power_supplies[FUELGAUGE].dev,&sh_battery_attrs[4]);

  if(result != 0)
  {
    device_remove_file(sh_battery_power_supplies[FUELGAUGE].dev,&sh_battery_attrs[4]);

    TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[4].attr.name);
    return -1;
  }

  for(i = 11; i <= 23; i++)
  {
    result = device_create_file(sh_battery_power_supplies[ADC].dev,&sh_battery_attrs[i]);

    if(result != 0)
    {
      device_remove_file(sh_battery_power_supplies[ADC].dev,&sh_battery_attrs[i]);

      TRACE("%s : failed to %s property register.\n",__FUNCTION__,sh_battery_attrs[i].attr.name);
      return -1;
    }
  }

  return result;
}

static int sh_battery_probe( struct platform_device* dev_p )
{
  struct shbatt_api_remote_req_t1
  {
    struct rpc_request_hdr hdr;
  } req_t1;

  struct shbatt_api_remote_rep_t1
  {
    struct rpc_reply_hdr hdr;
    int result;
    int value;
  } rep_t1;

  int i, result;

  TRACE("%s \n",__FUNCTION__);

  ep_shbatt_p = msm_rpc_connect_compatible(SHBATT_REMOTE_A2MPROG,SHBATT_REMOTE_A2MVERS,0);

  if(IS_ERR(ep_shbatt_p))
  {
    TRACE("%s : rpc connect failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shbatt_p));
    return -1;
  }

  result = msm_rpc_call_reply(ep_shbatt_p,
                              SHBATT_API_INITIALIZE_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : initialize shbatt failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shbatt_p));
    return -1;
  }

  result = msm_rpc_call_reply(ep_shbatt_p,
                              SHBATT_API_GET_CHARGER_CABLE_STATUS_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : get charger cable status failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shbatt_p));
    return -1;
  }
  else
  {
    sh_battery_charger_cable_status = be32_to_cpu(rep_t1.value);
  }

  result = msm_rpc_call_reply(ep_shbatt_p,
                              SHBATT_API_GET_BATTERY_CHARGING_STATUS_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : get battery charging status failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shbatt_p));
    return -1;
  }
  else
  {
    sh_battery_battery_charging_status = be32_to_cpu(rep_t1.value);
  }

  result = msm_rpc_call_reply(ep_shbatt_p,
                              SHBATT_API_GET_BATTERY_CAPACITY_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : get battery capacity failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shbatt_p));
    return -1;
  }
  else
  {
    sh_battery_battery_capacity = be32_to_cpu(rep_t1.value);
  }

  for(i = 0; i < ARRAY_SIZE(sh_battery_power_supplies); i++)
  {
    result = power_supply_register(&dev_p->dev,&sh_battery_power_supplies[i]);

    if(result != 0)
    {
      TRACE("%s : failed to power supply register.\n",__FUNCTION__);
      return -1;
    }
  }

  result = sh_battery_create_attrs();

  if(result < 0)
  {
    TRACE("%s : failed to property register.\n",__FUNCTION__);
    return -1;
  }

  ep_shchg_p = msm_rpc_connect_compatible(SHCHG_REMOTE_A2MPROG,SHCHG_REMOTE_A2MVERS,0);

  if(IS_ERR(ep_shchg_p))
  {
    TRACE("%s : rpc connect failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shchg_p));
    return -1;
  }

  result = msm_rpc_call_reply(ep_shchg_p,
                              SHCHG_API_INITIALIZE_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : initialize shchg failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shchg_p));
    return -1;
  }

  ep_shtherm_p = msm_rpc_connect_compatible(SHTHERM_REMOTE_A2MPROG,SHTHERM_REMOTE_A2MVERS,0);

  if(IS_ERR(ep_shtherm_p))
  {
    TRACE("%s : rpc connect failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shtherm_p));
    return -1;
  }

  result = msm_rpc_call_reply(ep_shtherm_p,
                              SHTHERM_API_INITIALIZE_REMOTE_PROC,
                              &req_t1,sizeof(req_t1),
                              &rep_t1,sizeof(rep_t1),
                              5 * HZ);
  if(result < 0)
  {
    TRACE("%s : initialize shtherm failed. rc = %ld\n",__FUNCTION__,PTR_ERR(ep_shtherm_p));
    return -1;
  }

  return 0;
}

static struct platform_driver sh_battery_driver = 
{
  .probe  = sh_battery_probe,
  .driver = { .name = "sh_battery", .owner = THIS_MODULE, },
};

static void sh_battery_update_charger_cable_status( int status )
{
  TRACE("%s(%d) \n",__FUNCTION__,status);

  sh_battery_charger_cable_status = status;
#if 0
  msm_hsusb_set_vbus_state(status == SH_BATTERY_CABLE_STATUS_USB);

  if(status == SH_BATTERY_CABLE_STATUS_USB)
  {
    wake_lock(&sh_battery_vbus_wake_lock);
  }
  else
  {
    wake_lock_timeout(&sh_battery_vbus_wake_lock,HZ / 2);
  }
#endif
  power_supply_changed(&sh_battery_power_supplies[USB]);
  power_supply_changed(&sh_battery_power_supplies[AC]);
}

static void sh_battery_update_battery_charging_status( int status )
{
  TRACE("%s(%d) \n",__FUNCTION__,status);

  sh_battery_battery_charging_status = status;

  power_supply_changed(&sh_battery_power_supplies[BATTERY]);
}

static void sh_battery_update_battery_capacity( int capacity )
{
  TRACE("%s(%d) \n",__FUNCTION__,capacity);

  sh_battery_battery_capacity = capacity;

  power_supply_changed(&sh_battery_power_supplies[BATTERY]);
}

struct sh_battery_rpc_update_charger_cable_status_args
{
  int status;
};

struct sh_battery_rpc_update_battery_charging_status_args
{
  int status;
};

struct sh_battery_rpc_update_battery_capacity_args
{
  int capacity;
};

static int sh_battery_handle_rpc_call( struct msm_rpc_server* svr_p,
                                       struct rpc_request_hdr* req_p,
                                       unsigned len )
{
  int result = 0;

  TRACE("[S] %s %d\n",__FUNCTION__,req_p->procedure);

  switch(req_p->procedure)
  {
    case SH_BATTERY_UPDATE_CHARGER_CABLE_STATUS_PROC:
    {
      struct sh_battery_rpc_update_charger_cable_status_args* args;
      args = (struct sh_battery_rpc_update_charger_cable_status_args*)(req_p + 1);
      args->status = be32_to_cpu(args->status);
      sh_battery_update_charger_cable_status(args->status);
      break;
    }

    case SH_BATTERY_UPDATE_BATTERY_CHARGING_STATUS_PROC:
    {
      struct sh_battery_rpc_update_battery_charging_status_args* args;
      args = (struct sh_battery_rpc_update_battery_charging_status_args*)(req_p + 1);
      args->status = be32_to_cpu(args->status);
      sh_battery_update_battery_charging_status(args->status);
      break;
    }

    case SH_BATTERY_UPDATE_BATTERY_CAPACITY_PROC:
    {
      struct sh_battery_rpc_update_battery_capacity_args* args;
      args = (struct sh_battery_rpc_update_battery_capacity_args*)(req_p + 1);
      args->capacity = be32_to_cpu(args->capacity);
      sh_battery_update_battery_capacity(args->capacity);
      break;
    }

    default:
      result = -1;
      break;
  }

  TRACE("[E] %s %d\n",__FUNCTION__,req_p->procedure);

  return result;
}

static struct msm_rpc_server sh_battery_rpc_server = 
{
  .prog     = SH_BATTERY_REMOTE_M2APROG,
  .vers     = SH_BATTERY_REMOTE_M2AVERS,
  .rpc_call = sh_battery_handle_rpc_call,
};

static int __init sh_battery_init( void )
{
  wake_lock_init(&sh_battery_vbus_wake_lock,WAKE_LOCK_SUSPEND,"vbus_present");

  mutex_init(&sh_battery_lock);

  msm_rpc_create_server(&sh_battery_rpc_server);

  platform_driver_register(&sh_battery_driver);

  return 0;
}

module_init(sh_battery_init);

MODULE_DESCRIPTION("SH Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
