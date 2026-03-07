#include "can_motor_driver/usb_can.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

UsbCan::UsbCan(const std::string& ifname) : can_socket_(-1) {
  int temp_socket;
  // 1. ソケットの作成
  if ((temp_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    fprintf(stderr, "[UsbCan] socket() failed: %s\n", strerror(errno));
    return;
  }

  // 2. インターフェースの検索 (can0など)
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, ifname.c_str());
  if (ioctl(temp_socket, SIOCGIFINDEX, &ifr) < 0) {
    fprintf(stderr, "[UsbCan] ioctl(SIOCGIFINDEX) failed for '%s': %s\n",
            ifname.c_str(), strerror(errno));
    close(temp_socket);  // ★失敗したら確実に閉じる
    return;
  }

  // 3. バインド (接続)
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(temp_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    fprintf(stderr, "[UsbCan] bind() failed for '%s': %s\n", ifname.c_str(),
            strerror(errno));
    close(temp_socket);  // ★失敗したら確実に閉じる
    return;
  }

  fprintf(stderr, "[UsbCan] opened '%s' (socket=%d, ifindex=%d)\n",
          ifname.c_str(), temp_socket, ifr.ifr_ifindex);

  // ★すべて成功した時だけ、メンバ変数に有効なソケットを代入する
  can_socket_ = temp_socket;
}

UsbCan::~UsbCan() {
  if (can_socket_ >= 0) close(can_socket_);
}

bool UsbCan::is_open() const { return can_socket_ >= 0; }

bool UsbCan::recv(struct can_frame& frame) {
  if (!is_open()) return false;
  ssize_t nbytes = ::recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT);
  return nbytes == static_cast<ssize_t>(sizeof(frame));
}

// 受信バッファを非ブロッキングで全て読み捨てる
void UsbCan::drain() {
  if (!is_open()) return;
  struct can_frame frame;
  // MSG_DONTWAITで受信キューが空になるまで読み続ける
  while (::recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {
  }
}

bool UsbCan::send(const struct can_frame& frame) {
  if (!is_open()) return false;
  ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
  if (nbytes != sizeof(struct can_frame)) {
    fprintf(stderr,
            "[UsbCan] send failed: id=0x%03X nbytes=%zd errno=%d (%s)\n",
            frame.can_id, nbytes, errno, strerror(errno));
    return false;
  }
  return true;
}
