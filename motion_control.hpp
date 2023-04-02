#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <future>
#include <thread>
#include <numeric>

std::ostream& operator<<(std::vector<double> nums, std::ostream& os){
    std::partial_sum(std::begin(nums), std::end(nums),
                     std::ostream_iterator<int>(os, " "));
    return os;
}
class UdpClient {
public:
  using PacketHandler = std::function<void(const std::string&)>;
  UdpClient(const std::string& server_ip="127.0.0.1", int server_port=1299)
    : ctx_{},socket_(ctx_), server_endpoint_(boost::asio::ip::address::from_string(server_ip), server_port)
  {
    // Open the socket for the UDP protocol
    socket_.open(boost::asio::ip::udp::v4());

    // Start listening for incoming packets
    start_receive();
    threads_.emplace_back([this](){ctx_.run();});
  }

  void send_id(const std::string& game_id) {
    std::string message = "#id " + game_id + "\n";
    socket_.send_to(boost::asio::buffer(message), server_endpoint_);
  }

  void set_packet_handler(PacketHandler handler) {
    packet_handler_ = std::move(handler);
  }
  void send_vector(std::vector<double> numbers_to_send){
    std::ostringstream oss;
    oss<<"put command here";
      std::partial_sum(std::begin(numbers_to_send), std::end(numbers_to_send),
                       std::ostream_iterator<int>(oss, " "));
      socket_.send_to(boost::asio::buffer(oss.str()), server_endpoint_);
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
  // threads constructd first, so it is destructed last.
  std::vector<std::thread > threads_;
    boost::asio::io_context ctx_;
    boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint server_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  std::array<char, 1024> receive_buffer_;
  PacketHandler packet_handler_;
};


