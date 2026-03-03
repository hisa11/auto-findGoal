#pragma once
#include <linux/can.h>

#include <string>

class UsbCan {
 public:
  UsbCan(const std::string& ifname);
  ~UsbCan();
  bool is_open() const;
  bool send(const struct can_frame& frame);
  void drain();  // 受信バッファを全て読み捨ててバッファ溢れを防ぐ
 private:
  int can_socket_;
};
