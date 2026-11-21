/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * wpa_supplicant private driver commands for Marvell PXA988 / SD8777 (mwifiex_sdio nl80211).
 */
#include "includes.h"

#include "common.h"
#include "driver_nl80211.h"
#include "linux_ioctl.h"
#include "wpa_supplicant_i.h"
#include "config.h"
#include "android_drv.h"

extern const struct wpa_driver_ops wpa_driver_nl80211_ops;

int wpa_driver_nl80211_driver_cmd(void *priv, char *cmd, char *buf, size_t buf_len)
{
	struct i802_bss *bss = priv;
	struct wpa_driver_nl80211_data *drv = bss->drv;
	int ret = 0;

	if (os_strcasecmp(cmd, "STOP") == 0) {
		linux_set_iface_flags(drv->global->ioctl_sock, bss->ifname, 0);
		wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STOPPED");
	} else if (os_strcasecmp(cmd, "START") == 0) {
		linux_set_iface_flags(drv->global->ioctl_sock, bss->ifname, 1);
		wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STARTED");
	} else if (os_strcasecmp(cmd, "MACADDR") == 0) {
		u8 macaddr[ETH_ALEN] = {};

		if (linux_get_ifhwaddr(drv->global->ioctl_sock, bss->ifname,
				       macaddr) < 0)
			return -1;
		ret = os_snprintf(buf, buf_len, "Macaddr = " MACSTR "\n",
				  MAC2STR(macaddr));
		if (os_snprintf_error(buf_len, ret))
			return -1;
	} else if (os_strncasecmp(cmd, "COUNTRY", 7) == 0) {
		char alpha2[3];

		if (os_strlen(cmd) < 10)
			return -1;
		alpha2[0] = cmd[8];
		alpha2[1] = cmd[9];
		alpha2[2] = '\0';
		if (wpa_driver_nl80211_ops.set_country)
			return wpa_driver_nl80211_ops.set_country(priv, alpha2);
		return -1;
	} else if (os_strcasecmp(cmd, "RSSI") == 0 ||
		   os_strcasecmp(cmd, "LINKSPEED") == 0) {
		struct wpa_signal_info sig;

		if (nl80211_get_link_signal(drv, &sig) < 0)
			return -1;
		if (os_strcasecmp(cmd, "RSSI") == 0) {
			ret = os_snprintf(buf, buf_len, "%s rssi %d\n",
					  bss->ifname, sig.current_signal);
		} else {
			ret = os_snprintf(buf, buf_len, "LinkSpeed %d\n",
					  sig.current_txrate / 1000);
		}
		if (os_snprintf_error(buf_len, ret))
			return -1;
	} else {
		/*
		 * No-op success for legacy Android private commands not exposed
		 * via private ioctl by upstream mwifiex (BTCOEXMODE, BTCOEXSCAN-*,
		 * RXFILTER-*, SETSUSPENDMODE, SCAN-ACTIVE, SCAN-PASSIVE, POWERMODE).
		 */
		ret = 0;
	}
	return ret;
}

int wpa_driver_set_p2p_noa(void *priv, u8 count, int start, int duration)
{
	return 0;
}

int wpa_driver_get_p2p_noa(void *priv, u8 *buf, size_t len)
{
	return 0;
}

int wpa_driver_set_p2p_ps(void *priv, int legacy_ps, int opp_ps, int ctwindow)
{
	return -1;
}

int wpa_driver_set_ap_wps_p2p_ie(void *priv, const struct wpabuf *beacon,
				 const struct wpabuf *proberesp,
				 const struct wpabuf *assocresp)
{
	return 0;
}
