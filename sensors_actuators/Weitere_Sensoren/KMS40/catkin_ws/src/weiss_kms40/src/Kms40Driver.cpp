#include <string>
#include <boost/asio.hpp>

#include <functional>
#include <future>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/SetBool.h>

using boost::asio::ip::tcp;

/*
 *
 *
 * TODO socket timeouts
 *
 *
 */

class Command {
public:
    virtual void run(tcp::iostream& connection) = 0;
};

class Tare : public Command {
public:
    // This captures the final result of the command (tared or untared)
    std::future<bool> result;
    // Construct a command for tare (true) or untare (false)
    Tare(bool apply_tare) : apply_tare_{apply_tare} {
        result = promise_.get_future();
    }
    // Communicate and signal state change
    void run(tcp::iostream& connection) {
        // We send TARE(1) or TARE(0)
        connection << "TARE(" << apply_tare_ << ")" << std::endl;
        std::string response;
        std::getline(connection, response);
        std::stringstream expected_response;
        // We expect TARE=1 or TARE=0
        expected_response << "TARE=" << apply_tare_;
        // Time to wake up the caller waiting for the result
        if(response.compare(expected_response.str()) == 0) {
            // Set the value
            promise_.set_value(apply_tare_);
        } else {
            // Or set an exception that will be thrown on calling result.get()
            promise_.set_exception(std::make_exception_ptr(std::runtime_error("Unexpected response:\n"+response)));
        }
    }
private:
    bool apply_tare_;
    std::promise<bool> promise_;
};

bool tare_callback(std::mutex* command_access, std::queue<std::shared_ptr<Command>>* command_queue, std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    command_access->lock();
    auto tare_command = std::shared_ptr<Tare>{new Tare(request.data)};
    command_queue->push(tare_command);
    command_access->unlock();
    tare_command->result.wait();
    try {
        bool is_tared = tare_command->result.get();
        response.success = (is_tared == request.data);
    } catch(std::runtime_error e) {
        response.success = false;
        response.message = e.what();
        ROS_ERROR("tare: %s", e.what());
    }

    return true;
}

// Continuous data stream
class DataStream {
public:
    struct Measurements {
        std::array<double, 6> wrench;
        unsigned long timestamp;

        Measurements(std::string msg) {
            auto scanned = sscanf(msg.c_str(),
                    "F={%lf,%lf,%lf,%lf,%lf,%lf},%lu",
                     &wrench.at(0), &wrench.at(1), &wrench.at(2),
                     &wrench.at(3), &wrench.at(4), &wrench.at(5),
                     &timestamp);
            if(scanned != 7) {
                throw std::runtime_error("Could not parse message:\n"+msg);
            }
        }
    };

    static void start(tcp::iostream& connection) {
        connection << "L1()" << std::endl;
        std::string response;
        std::getline(connection, response);
        assert(response.compare("L1") == 0);
    }

    static void stop(tcp::iostream& connection) {
        connection << "L0()" << std::endl;
        std::string response;
        // Consume measurements already on the way
        // until the constructor fails.
        while(true) {
            std::getline(connection, response);
            try {
                Measurements{response};
            }
            catch(std::runtime_error e) {
                break;
            }
        }
        assert(response.compare("L0") == 0);
    }

    static Measurements update(tcp::iostream& connection) {
        std::string message;
        std::getline(connection, message);
        return Measurements(message);
    }
};

void worker(std::mutex* command_access, std::queue<std::shared_ptr<Command>>* command_queue, std::string host, int port, std::string frame) {
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
    geometry_msgs::WrenchStamped msg;
    msg.header.frame_id = frame;

    while (ros::ok())
    {
        ROS_INFO("Trying to connect to KMS40 ...");
        try
        {
            // set up connection to kms
            tcp::iostream s(host, std::to_string(port));
            if (!s)
            {
                ROS_WARN("Connection failed, retry in 5 seconds...");
                ros::Duration(1, 0).sleep();
                continue;
            }
            ROS_INFO("Connection established. Starting data acquisition.");
            DataStream stream;
            stream.start(s);

            while(ros::ok())
            {
                // Check for commands
                command_access->lock();
                if(!command_queue->empty()) {
                    stream.stop(s);
                    // Process commands
                    while(!command_queue->empty()) {
                        command_queue->front()->run(s);
                        command_queue->pop();
                    }
                    stream.start(s);
                }
                command_access->unlock();

                auto m = stream.update(s);
                // TODO? use sensor timestamps
                // Only if NTP on sensor works
                msg.header.stamp = ros::Time::now();
                msg.wrench.force.x = m.wrench[0];
                msg.wrench.force.y = m.wrench[1];
                msg.wrench.force.z = m.wrench[2];
                msg.wrench.torque.x = m.wrench[3];
                msg.wrench.torque.y = m.wrench[4];
                msg.wrench.torque.z = m.wrench[5];
                pub.publish(msg);
            }

            stream.stop(s);
        }
        catch (std::exception& e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }

    }
}

int main(int argc, char **argv)
{
    std::mutex command_access;
    std::queue<std::shared_ptr<Command>> command_queue;

    ros::init(argc, argv, "kms40");
    ros::NodeHandle private_nh("~");
    ros::ServiceServer service = private_nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                "tare", std::bind(tare_callback, &command_access, &command_queue, std::placeholders::_1, std::placeholders::_2));

    // Get parameters
    std::string host, frame;
    int port;

    if( !private_nh.getParam("host", host) )
    {
        ROS_ERROR("Parameter ~host not set.");
        return -1;
    }

    if( !private_nh.getParam("port", port) )
    {
        ROS_ERROR("Parameter ~port not set.");
        return -1;
    }

    if( !private_nh.getParam("frame", frame) )
    {
        ROS_INFO("Parameter ~frame not set. Using default (kms40)");
        frame = "kms40";
    }

    std::thread worker_thread(worker, &command_access, &command_queue, host, port, frame);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    worker_thread.join();

    return 0;
}
