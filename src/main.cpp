#include <iostream>
#include "networking.hpp"

#include <future>
#include <chrono>
#include <cmath>
#include <thread>

int unused_test() {

    UdpClient client("127.0.0.1", 1288);

    // Set a handler for incoming packets
    client.set_packet_handler([](const std::string& packet) {
        std::cout << "Received packet: " << packet << std::endl;
    });

    // Send the #id message to the server
    client.send_id("myGame1");

    // Run the I/O service to start receiving packets

    return 0;
}
int main(){


    auto task = std::async([](){
        UdpClient u;
        u.send_id("this is my name");
        u.set_packet_handler([](std::string content){
            std::cout << "packed recieved" << content << std::endl;
        });
        const auto fps=100;
        const auto delta=2*3.14159/fps;
        auto i = 0;
        std::vector<double> numbers{0,0};
        for(;;){
            auto my_cos = std::cos(i*delta);
            auto my_sin = std::sin(i*delta);
            numbers.at(0) = my_cos;
            numbers.at(1) = my_sin;
            u.send_vector(numbers);
            i++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }

    });
    task.wait_for(std::chrono::seconds{10});
    return 0;
}