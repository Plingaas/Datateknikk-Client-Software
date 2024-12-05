//
// Created by Pling on 03/12/2024.
//

#ifndef TCPCONNECTOR_HPP
#define TCPCONNECTOR_HPP

#include "simple_socket/TCPSocket.hpp"
#include <functional>
#include <iostream>
#include <thread>

class TCPConnector : public simple_socket::TCPClientContext {

public:
  using FailedToConnectHandler = std::function<void()>;
  using ConnectHandler = std::function<void()>;
  using ConnectionLostHandler = std::function<void()>;
  using DisconnectHandler = std::function<void()>;
  using ConnectionRestoredHandler = std::function<void()>;

  void setFailedToConnectHandler(FailedToConnectHandler handler) {failed_to_connect_handler = std::move(handler);};
  void setConnectHandler(ConnectHandler handler) {connect_handler = std::move(handler);};
  void setConnectionLostHandler(ConnectionLostHandler handler) {connection_lost_handler = std::move(handler);};
  void setDisconnectHandler(DisconnectHandler handler) {disconnect_handler = std::move(handler);};
  void setConnectionRestoredHandler(ConnectionRestoredHandler handler) {connection_restored_handler = std::move(handler);};

  TCPConnector(std::string& ip_, uint16_t port_, bool verbose_ = false);
  void connect();

  int read(std::vector<uint8_t>& buffer);
  int read(std::vector<uint8_t>& buffer, size_t bytes);

  void setIP(const std::string& ip_) {ip=ip_;};
  void setPort(const uint16_t port_) {port=port_;};
  void setName(const std::string& name_) {name=name_;};
  void setTimeout(const long milliseconds) {timeout=milliseconds;};
  void flush();
private:
  long getCurrentTime();
  void printMessage(std::string message) {std::cout << "(" << name << "): " << message << std::endl;};

  std::string ip;
  uint16_t port;
  bool connected;
  long timeout;
  long last_read_time;

  std::string name;
  bool verbose;

  FailedToConnectHandler failed_to_connect_handler;
  ConnectHandler connect_handler;
  ConnectionLostHandler connection_lost_handler;
  DisconnectHandler disconnect_handler;
  ConnectionRestoredHandler connection_restored_handler;

  std::unique_ptr<simple_socket::SimpleConnection> connection;
};
#endif //TCPCONNECTOR_HPP
