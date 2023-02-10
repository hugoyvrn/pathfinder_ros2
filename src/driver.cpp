#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <chrono>

#include <iostream>
#include <math.h>
#include <functional>
using namespace std;
using namespace std::placeholders;

#include <rtac_asio/Stream.h>
using namespace rtac::asio;

//std::string msg = "Hello there !\n";

class DvlPubSub : public rclcpp::Node
{
public:
	DvlPubSub(): 
        Node("dvl_pub_sub"),
        count_(0),
        stream(Stream::CreateSerial("/dev/ttyUSB0", 115200)),
        data(1024u, '\0')
	{
		stream->start();
        stream->enable_io_dump();
		std::cout<<"Started"<<std::endl;

		pub_dvl_data_ = this->create_publisher<std_msgs::msg::String>("dvl/data",1000);


        //timer_ = this->create_wall_timer(500ms, std::bind(&DvlPubSub::timer_callback, this));
        


        stream->async_read_until(data.size(), (uint8_t*)data.c_str(), '\n',
                                 std::bind(&DvlPubSub::read_callback, this, stream, _1, _2));
        

        stream->write("==="); //break
        std::this_thread::sleep_for(1000ms);

        stream->write("#PD13\r"); //set ascii format
        stream->write("CS\r"); //ecoute
	}

// utiliser asynch_read_until qui appelle callback
    // mettre asynch_read_until dans callback

private:
	Stream::Ptr stream;
    std::string data;
    std_msgs::msg::String data_msg;
    std::string time_str;

/*
    void timer_callback()
    {
        stream->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);

        data_msg.data=data;
        std::cout<< "read" << std::endl;
        std::cout << data <<std::endl;

        pub_dvl_data_->publish(data_msg);
    }

*/
    void read_callback(Stream::Ptr stream,
                       const SerialStream::ErrorCode& err,
                       std::size_t count)
    {
        auto clock = this->get_clock();
        auto now = clock->now();
        //RCLCPP_INFO(this->get_logger(), "Current time: %ld.%09ld", now.seconds(), now.nanoseconds());
        double time_sec = now.seconds();
        double time_nano = now.seconds();
        double time = time_sec+time_nano*pow(10,-9);
        time_str = std::to_string(time);

        if(err)
        {
            std::cout<<"error"<<std::endl;
        }

        data_msg.data=time_str+"time_stamped"+data;
        //std::cout<<"read data"<<std::endl;
        //std::cout<<data<<std::endl;

        pub_dvl_data_->publish(data_msg);

        stream->async_read_until(data.size(), (uint8_t*)data.c_str(), '\n',
                                 std::bind(&DvlPubSub::read_callback, this, stream, _1, _2));
        



    }
    

    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dvl_data_;
   // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  
    /*
    std::string data(1024, '\0');

    auto stream = Stream::CreateSerial("/dev/ttyUSB0", 115200);
    stream->start();
    std::cout << "Started" << std::endl;
    
    //while(1) {
    for(int i = 0; i < 5; i++) {
        getchar();
        stream->write(msg.size(), (const uint8_t*)msg.c_str());
        //std::cout << "Read " << stream->read(msg.size(), (uint8_t*)data.c_str())
        //          << " bytes." << std::endl;
        std::cout << "Read " << stream->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000)
                  << " bytes." << std::endl << std::flush;

    */

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DvlPubSub>());
    rclcpp::shutdown();


    return 0;
}