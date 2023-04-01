#include <iostream>
#include "flight_simulator.hpp"
#include <future>
#include <chrono>
#include <cmath>

int unused_test() {
    boost::asio::io_service io_service;

    UdpClient client(io_service, "127.0.0.1", 1288);

    // Set a handler for incoming packets
    client.set_packet_handler([](const std::string& packet) {
        std::cout << "Received packet: " << packet << std::endl;
    });

    // Send the #id message to the server
    client.send_id("myGame1");

    // Run the I/O service to start receiving packets
    io_service.run();

    return 0;
}
main(){
    UdpClient u;
    u.send_id("this is my name");
    u.set_packet_handler([](std::string content){
        std::cout << "packed recieved" << content << std::endl;
    });

    auto task = std::async([](){
        const auto fps=100;
        const auto delta=2*3.14159/fps;
        int frame=0;
        for(;;){
            auto my_cos = std::cos(i*delta);
            auto my_sin = std::sin(i*delta);
        }

    });
    task.wait_for(std::chrono::seconds{10});
    return 0;
}