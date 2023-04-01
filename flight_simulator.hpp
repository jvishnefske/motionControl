#include <iostream>
#include <boost/asio.hpp>

class UdpClient {
public:
  using PacketHandler = std::function<void(const std::string&)>;

  UdpClient(boost::asio::io_service& io_service, const std::string& server_ip, int server_port)
    : socket_(io_service), server_endpoint_(boost::asio::ip::address::from_string(server_ip), server_port)
  {
    // Open the socket for the UDP protocol
    socket_.open(boost::asio::ip::udp::v4());

    // Start listening for incoming packets
    start_receive();
  }

  void send_id(const std::string& game_id) {
    std::string message = "#id " + game_id + "\n";
    socket_.send_to(boost::asio::buffer(message), server_endpoint_);
  }

  void set_packet_handler(PacketHandler handler) {
    packet_handler_ = std::move(handler);
  }

private:
  void start_receive() {
    socket_.async_receive_from(boost::asio::buffer(receive_buffer_), remote_endpoint_,
      [this](boost::system::error_code ec, std::size_t bytes_received) {
        if (!ec) {
          std::string packet(receive_buffer_.data(), bytes_received);
          packet_handler_(packet);
        }
        start_receive();
      });
  }

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint server_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  std::array<char, 1024> receive_buffer_;
  PacketHandler packet_handler_;
};


