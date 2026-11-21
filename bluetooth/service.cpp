/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linux HCI_CHANNEL_USER Bluetooth HAL service for Marvell PXA988 (btmrvl_sdio hci0).
 */

#define LOG_TAG "android.hardware.bluetooth@1.0-service.pxa988"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <android/hardware/bluetooth/1.0/IBluetoothHci.h>
#include <android/hardware/bluetooth/1.0/IBluetoothHciCallbacks.h>
#include <hidl/HidlTransportSupport.h>
#include <log/log.h>

#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH 31
#endif
#ifndef BTPROTO_HCI
#define BTPROTO_HCI 1
#endif
#ifndef HCI_CHANNEL_USER
#define HCI_CHANNEL_USER 1
#endif
#ifndef HCIDEVDOWN
#define HCIDEVDOWN _IOW('H', 202, int)
#endif

struct sockaddr_hci {
    sa_family_t hci_family;
    unsigned short hci_dev;
    unsigned short hci_channel;
};

namespace {

using android::sp;
using android::hardware::Return;
using android::hardware::Void;
using android::hardware::hidl_vec;
using android::hardware::bluetooth::V1_0::IBluetoothHci;
using android::hardware::bluetooth::V1_0::IBluetoothHciCallbacks;
using android::hardware::bluetooth::V1_0::Status;

constexpr uint8_t H4_TYPE_COMMAND = 0x01;
constexpr uint8_t H4_TYPE_ACL = 0x02;
constexpr uint8_t H4_TYPE_SCO = 0x03;
constexpr uint8_t H4_TYPE_EVENT = 0x04;
constexpr uint16_t HCI_DEV_ID = 0;

void unblock_bluetooth_rfkill() {
    DIR* dir = opendir("/sys/class/rfkill");
    if (!dir) {
        return;
    }
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        char type_path[128];
        snprintf(type_path, sizeof(type_path), "/sys/class/rfkill/%s/type", ent->d_name);
        int tfd = open(type_path, O_RDONLY | O_CLOEXEC);
        if (tfd < 0) {
            continue;
        }
        char buf[32] = {};
        ssize_t n = read(tfd, buf, sizeof(buf) - 1);
        close(tfd);
        if (n > 0 && strncmp(buf, "bluetooth", 9) == 0) {
            char state_path[128];
            snprintf(state_path, sizeof(state_path), "/sys/class/rfkill/%s/state", ent->d_name);
            int sfd = open(state_path, O_WRONLY | O_CLOEXEC);
            if (sfd >= 0) {
                if (write(sfd, "1", 1) < 0) {
                    ALOGW("Failed to unblock %s: %s", state_path, strerror(errno));
                }
                close(sfd);
            }
        }
    }
    closedir(dir);
}

class PxaBluetoothHci : public IBluetoothHci {
public:
    Return<void> initialize(const sp<IBluetoothHciCallbacks>& cb) override {
        std::lock_guard<std::mutex> lock(lock_);
        close_locked();

        if (!cb) {
            return Void();
        }
        cb_ = cb;
        unblock_bluetooth_rfkill();

        int fd = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
        if (fd < 0) {
            ALOGE("socket(AF_BLUETOOTH, BTPROTO_HCI) failed: %s", strerror(errno));
            cb_->initializationComplete(Status::INITIALIZATION_ERROR);
            return Void();
        }

        /* Bring hci0 down so HCI_CHANNEL_USER exclusive bind succeeds */
        if (ioctl(fd, HCIDEVDOWN, HCI_DEV_ID) < 0 && errno != EALREADY) {
            ALOGW("HCIDEVDOWN(hci%u) returned: %s", HCI_DEV_ID, strerror(errno));
        }

        struct sockaddr_hci addr = {};
        addr.hci_family = AF_BLUETOOTH;
        addr.hci_dev = HCI_DEV_ID;
        addr.hci_channel = HCI_CHANNEL_USER;

        if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ALOGE("bind(hci%u, HCI_CHANNEL_USER) failed: %s", HCI_DEV_ID, strerror(errno));
            ::close(fd);
            cb_->initializationComplete(Status::INITIALIZATION_ERROR);
            return Void();
        }

        hci_fd_ = fd;
        running_.store(true);
        reader_thread_ = std::thread(&PxaBluetoothHci::reader_loop, this);

        ALOGI("Bound hci%u via HCI_CHANNEL_USER (fd=%d)", HCI_DEV_ID, hci_fd_);
        cb_->initializationComplete(Status::SUCCESS);
        return Void();
    }

    Return<void> sendHciCommand(const hidl_vec<uint8_t>& command) override {
        send_h4_packet(H4_TYPE_COMMAND, command);
        return Void();
    }

    Return<void> sendAclData(const hidl_vec<uint8_t>& data) override {
        send_h4_packet(H4_TYPE_ACL, data);
        return Void();
    }

    Return<void> sendScoData(const hidl_vec<uint8_t>& data) override {
        send_h4_packet(H4_TYPE_SCO, data);
        return Void();
    }

    Return<void> close() override {
        std::lock_guard<std::mutex> lock(lock_);
        close_locked();
        return Void();
    }

private:
    void close_locked() {
        running_.store(false);
        if (hci_fd_ >= 0) {
            ::shutdown(hci_fd_, SHUT_RDWR);
            ::close(hci_fd_);
            hci_fd_ = -1;
        }
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        cb_ = nullptr;
    }

    void send_h4_packet(uint8_t type, const hidl_vec<uint8_t>& payload) {
        std::lock_guard<std::mutex> lock(write_lock_);
        int fd = hci_fd_;
        if (fd < 0 || payload.size() == 0) {
            return;
        }
        struct iovec iov[2];
        iov[0].iov_base = &type;
        iov[0].iov_len = 1;
        iov[1].iov_base = const_cast<uint8_t*>(payload.data());
        iov[1].iov_len = payload.size();

        ssize_t written = writev(fd, iov, 2);
        if (written < 0) {
            ALOGE("writev(type=0x%02x, len=%zu) failed: %s",
                  type, payload.size(), strerror(errno));
        }
    }

    void reader_loop() {
        std::vector<uint8_t> buf(4096);
        while (running_.load()) {
            int fd = hci_fd_;
            if (fd < 0) {
                break;
            }
            struct pollfd pfd = {fd, POLLIN, 0};
            int pret = poll(&pfd, 1, 500);
            if (pret < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            if (pret == 0 || !(pfd.revents & POLLIN)) {
                continue;
            }
            ssize_t n = read(fd, buf.data(), buf.size());
            if (n <= 1) {
                if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
                    continue;
                }
                break;
            }
            sp<IBluetoothHciCallbacks> cb = cb_;
            if (!cb) {
                continue;
            }
            hidl_vec<uint8_t> pkt;
            pkt.setToExternal(buf.data() + 1, static_cast<size_t>(n - 1));
            switch (buf[0]) {
                case H4_TYPE_EVENT:
                    cb->hciEventReceived(pkt);
                    break;
                case H4_TYPE_ACL:
                    cb->aclDataReceived(pkt);
                    break;
                case H4_TYPE_SCO:
                    cb->scoDataReceived(pkt);
                    break;
                default:
                    ALOGW("Unknown H4 packet type 0x%02x", buf[0]);
                    break;
            }
        }
    }

    std::mutex lock_;
    std::mutex write_lock_;
    int hci_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread reader_thread_;
    sp<IBluetoothHciCallbacks> cb_;
};

}  // namespace

int main() {
    android::hardware::configureRpcThreadpool(1, true);
    sp<IBluetoothHci> service = new PxaBluetoothHci();
    android::status_t status = service->registerAsService();
    if (status != android::OK) {
        ALOGE("Failed to register IBluetoothHci service: %d", status);
        return 1;
    }
    android::hardware::joinRpcThreadpool();
    return 0;
}
